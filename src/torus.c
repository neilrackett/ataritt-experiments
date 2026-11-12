/*
 * Copyright (C) 2026 Neil Rackett
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * TORUS.C - (2,3) torus-knot raymarcher for the Atari TT
 *
 * Every pixel is sphere-traced against a signed distance field on the TT's
 * 68882. The knot is two strand circles per cross-section of a carrier
 * torus, with the pair pattern turning 1.5x per revolution (period pi, so it
 * closes seamlessly into a trefoil). The SDF needs cos/sin of 1.5*theta
 * around the main axis but uses NO atan2: cos(theta) and sin(theta) fall
 * out of x/rho and z/rho, the half angle comes from sqrt((1 +/- cos)/2) with
 * sin's sign, and the angle-sum formulas combine them - all mul/div/sqrt,
 * one FPU instruction apiece.
 *
 * The 68882 sits on the 68030's coprocessor bus, so this is plain C built
 * -m68881: ray setup, each march step and the shading compile to straight
 * runs of FPU instructions, intermediates held in FP0-FP7, with single-
 * precision multiplies and divides as FSGLMUL/FSGLDIV.
 *
 * Rendering: chunky blocks in TT low res (320x480, 256 colours), with 64
 * shades on the knot's ramp, double-buffered. TT low's pixels are twice as
 * wide as they are tall, so a block is twice as tall as it is wide:
 * 80x60 blocks of 4x8 by default, HIRES=1 for 160x120 of 2x4.
 */

#include <string.h>
#include <mint/osbind.h>

#include "atari_tt.h"
#include "sintab.h"

/* ---------- resolution ---------- */
#ifndef HIRES
#define HIRES 0
#endif

#if HIRES
#  define RENDER_W  160
#  define RENDER_H  120
#  define BLOCK_W     2
#  define BLOCK_H     4
#  define MAX_STEPS  12   /* fewer steps to keep fps alive at 4x pixels */
#else
#  define RENDER_W   80
#  define RENDER_H   60
#  define BLOCK_W     4
#  define BLOCK_H     8
#  define MAX_STEPS  16
#endif

/* ---------- screen: TT low res ---------- */
#define SCREEN_W       320
#define PLANES         8
#define SCANLINE_BYTES TT_LINE_BYTES
#define GROUPS         (SCREEN_W / 16)          /* 16-pixel groups a line */
#define PER_GROUP      (16 / BLOCK_W)           /* blocks per group       */

/* ---------- palette: black, then a cyan->white ramp for the knot ----------
 * The 16-colour demos' ramp as keys (4-bit levels), spread over NSHADE
 * entries; the 12-bit palette tops out at about 40 distinct steps along it,
 * which tt_ramp() reaches by moving one gun at a time. */
#define NSHADE 64                               /* palette 1..64          */
#define WHITE  255                              /* the ms digits          */

static const unsigned char ramp_keys[15][3] = {
    { 0, 2, 3 }, { 0, 3, 4 }, { 0, 4, 6 }, { 1, 6, 8 }, { 1, 7, 9 },
    { 2, 8, 10 }, { 2, 9, 11 }, { 3, 10, 12 }, { 4, 11, 13 }, { 5, 12, 14 },
    { 7, 13, 15 }, { 9, 14, 15 }, { 11, 14, 15 }, { 13, 15, 15 }, { 15, 15, 15 },
};
static unsigned short palette[256];

/* ---------- vec3 ---------- */
typedef struct { float x, y, z; } Vec3;

#define SQRT(x) __builtin_sqrtf(x)              /* one FSQRT              */
#define FABS(x) __builtin_fabsf(x)              /* one FABS               */

/* ---------- geometry ---------- */
#define TORUS_R    1.0f   /* major radius: the knot's carrier circle       */
#define KNOT_W     0.35f  /* winding radius: strand offset in cross-section */
#define TUBE_R     0.16f  /* strand tube radius                            */
#define SDF_SCALE  0.85f  /* Lipschitz safety: the twisting strand pattern
                             makes the cross-section field over-estimate by
                             up to sqrt(1 + (1.5*W)^2) ~= 1.13, so 1/1.13
                             with a little margin                           */
#define CAM_DIST   3.0f
#define BOUND_R    1.55f  /* R + W + tube + margin: bounding sphere        */
#define BOUND_C    (CAM_DIST * CAM_DIST - BOUND_R * BOUND_R)
#define MARCH_HIT  0.01f  /* on the scaled distance                        */

/* Two-level marching: the knot lies inside a shell of radius SHELL_R around
 * the carrier circle, and the plain-torus shell SDF is a valid lower bound
 * for the knot SDF at a third of the cost - and exact (Lipschitz 1), so
 * shell steps skip SDF_SCALE too. Only within BOUND_NEAR of the shell is the
 * full knot field evaluated. */
#define SHELL_R    (KNOT_W + TUBE_R)
#define BOUND_NEAR 0.06f

/* Temporal warm start: the rotation moves any surface point by at most
 * ~0.06/frame, so a pixel that hit last frame can start marching just short
 * of last frame's hit distance instead of at the bounding sphere. The
 * backoff covers two frames of motion because of the checkerboard below. */
#define WARM_BACKOFF 0.15f

/* Checkerboard: march only alternate pixels each frame (lattice parity flips
 * per frame) and hold the other half from last frame's colours - halves the
 * per-frame FPU work for a one-frame lag on alternate pixels. Set to 0 for
 * every-pixel rendering. */
#ifndef CHECKERBOARD
#define CHECKERBOARD 1
#endif

/* ---------- rotation helpers (LUT-based, per-frame only) ---------- */
static void rot_y(Vec3 *v, int angle) {
    float s = lut_sin(angle);
    float c = lut_cos(angle);
    float nx = c * v->x + s * v->z;
    float nz = c * v->z - s * v->x;     /* -s for right-hand */
    v->x = nx;
    v->z = nz;
}

static void rot_x(Vec3 *v, int angle) {
    float s = lut_sin(angle);
    float c = lut_cos(angle);
    float ny = c * v->y - s * v->z;
    float nz = s * v->y + c * v->z;
    v->y = ny;
    v->z = nz;
}

/*
 * Ray setup: rd = normalize(row + fx*right), then the bounding sphere test.
 * For an orthonormal camera basis dot(fwd, row + fx*right) == 1, and
 * ro = -CAM_DIST*fwd, so -b = -dot(ro, rd) = CAM_DIST/|rd_unnorm| - the
 * sphere test needs no second dot product. Returns 0 on a miss, else fills
 * rd and the march interval [t0, t1].
 */
static int ray_setup(const Vec3 *right, const Vec3 *row, float fx,
                     Vec3 *rd, float *t0, float *t1)
{
    float x = row->x + fx * right->x;
    float y = row->y + fx * right->y;
    float z = row->z + fx * right->z;
    float inv = 1.0f / SQRT(x * x + y * y + z * z);
    float nb = CAM_DIST * inv;                          /* -b             */
    float disc = nb * nb - BOUND_C;
    float sq;

    if (!(disc > 0.0f))
        return 0;
    rd->x = x * inv;
    rd->y = y * inv;
    rd->z = z * inv;
    sq = SQRT(disc);
    *t0 = nb - sq;
    *t1 = nb + sq;
    return 1;
}

/*
 * Scaled distance to the knot at ro + t*rd. Two-level: first the cheap
 * carrier-shell lower bound; only if the point is within BOUND_NEAR of the
 * shell does it go on to the knot. There, instead of building the strand
 * offsets, it uses d{0,1}^2 = q^2 + W^2 -/+ 2W*(c.u): with
 * e = s*cos(psi) + y*sin(psi) regrouped by half-angle terms as
 * e = ct*(s*ch + y*sh) + st*(y*ch - s*sh), the nearer strand is simply
 * q^2 + W^2 - |2W*e| - one FABS instead of two distances and a compare.
 */
static inline float knot_dist(const Vec3 *ro, const Vec3 *rd, float t)
{
    float px = ro->x + rd->x * t;
    float py = ro->y + rd->y * t;
    float pz = ro->z + rd->z * t;
    float rho = SQRT(px * px + pz * pz);
    float s = rho - TORUS_R;
    float q2 = s * s + py * py;
    float db = SQRT(q2) - SHELL_R;
    float ct, st, h, ch, sh, e;

    if (db > BOUND_NEAR)                    /* far: the bound suffices    */
        return db;
    /* cos/sin(theta); half angle via two square roots, not
     * sin/(2*cos(theta/2)), which loses it to cancellation near pi */
    ct = px / rho;
    st = pz / rho;
    h  = 0.5f * ct;
    ch = SQRT(0.5f + h);                    /* cos(theta/2) >= 0          */
    sh = SQRT(0.5f - h);                    /* |sin(theta/2)|             */
    if (st < 0.0f)
        sh = -sh;                           /* sin(theta)'s sign          */
    e = ct * (s * ch + py * sh) + st * (py * ch - s * sh);
    return SDF_SCALE * (SQRT(q2 + KNOT_W * KNOT_W - FABS(2.0f * KNOT_W * e)) - TUBE_R);
}

/* ---------- raymarch over [t0, t1] ---------- */
static int raymarch(const Vec3 *ro, const Vec3 *rd, float t, float t1, float *hit_t)
{
    int i;

    for (i = 0; i < MAX_STEPS; i++) {
        float d = knot_dist(ro, rd, t);
        if (d < MARCH_HIT) { *hit_t = t; return 1; }
        t += d;
        if (t > t1) return 0;
    }
    return 0;
}

/* ---------- shade: diffuse + specular, to palette index 1..NSHADE ---------- */
static const Vec3 LIGHT = { 0.577f, 0.577f, 0.577f }; /* normalised */

/*
 * Recompute the cross-section decomposition at the hit, pick the nearer
 * strand, then work the normal in cylindrical coordinates: n = (a*ct, b,
 * a*st) with a = s -/+ ux and b = y -/+ uy, so |n|^2 = a^2 + b^2 and
 * nothing leaves the cross-section plane until the light dots. The
 * nearest-strand-centre normal is exact for an untwisted tube and
 * indistinguishable for a twisted one at this size - and avoids the six
 * SDF evaluations of central differences. Blinn-Phong specular, ^8.
 */
static int shade(const Vec3 *ro, const Vec3 *rd, float t)
{
    float px = ro->x + rd->x * t;
    float py = ro->y + rd->y * t;
    float pz = ro->z + rd->z * t;
    float rho = SQRT(px * px + pz * pz);
    float s = rho - TORUS_R;
    float ct = px / rho, st = pz / rho;
    float h = 0.5f * ct;
    float ch = SQRT(0.5f + h);
    float sh = SQRT(0.5f - h);
    float ux, uy, a, b, a1, b1, inv, diff, hx, hy, hz, nh, intensity;
    int idx;

    if (st < 0.0f)
        sh = -sh;
    ux = KNOT_W * (ct * ch - st * sh);      /* W cos(1.5 theta)           */
    uy = KNOT_W * (st * ch + ct * sh);      /* W sin(1.5 theta)           */
    a  = s - ux;  b  = py - uy;             /* to one strand's centre...  */
    a1 = s + ux;  b1 = py + uy;             /* ...and the other's         */
    if (a1 * a1 + b1 * b1 < a * a + b * b) {
        a = a1;
        b = b1;
    }
    inv = 1.0f / SQRT(a * a + b * b);
    a *= inv;
    b *= inv;

    /* diffuse = a*(ct*Lx + st*Lz) + b*Ly */
    diff = a * (ct * LIGHT.x + st * LIGHT.z) + b * LIGHT.y;
    if (diff < 0.0f)
        diff = 0.0f;

    /* n.h, h = LIGHT - rd normalised */
    hx = LIGHT.x - rd->x;
    hy = LIGHT.y - rd->y;
    hz = LIGHT.z - rd->z;
    nh = (a * (ct * hx + st * hz) + b * hy) / SQRT(hx * hx + hy * hy + hz * hz);
    if (nh < 0.0f)
        nh = 0.0f;
    nh *= nh;
    nh *= nh;
    nh *= nh;

    /* The 16-colour ramp's colour k covers intensity*14 in [k, k+1): take
     * the shade at the middle of it, half a step on, so the knot keeps the
     * brightness it has in 16 colours and only the steps get finer */
    intensity = 0.7f * diff + 0.3f * nh;
    idx = 1 + (int)((intensity - 0.5f / 14) * (NSHADE - 1));  /* FINTRZ */
    if (idx < 1) idx = 1;
    if (idx > NSHADE) idx = NSHADE;
    return idx;
}

/* ---------- chunky -> planar, a row of blocks at a time ---------- */
/* s_c2p[k][c]: colour c's planes 2k and 2k+1 as a block's worth of bits,
 * plane 2k in the high word and 2k+1 in the low, so shifting a block in at
 * a time builds a 16-pixel group of both planes in one long - in the order
 * they sit in memory. No clear and no read-modify-write: every line of the
 * screen is written once a frame. */
static unsigned long s_c2p[PLANES / 2][256];

static void init_c2p(void)
{
    const unsigned long fill = (1UL << BLOCK_W) - 1;
    int k, c;

    for (k = 0; k < PLANES / 2; k++)
        for (c = 0; c < 256; c++)
            s_c2p[k][c] = ((c >> (2 * k) & 1) ? fill << 16 : 0)
                        | ((c >> (2 * k + 1) & 1) ? fill : 0);
}

static void emit_row(unsigned char *scr, int cy, const unsigned char *colours)
{
    unsigned long *line = (unsigned long *)(scr + cy * BLOCK_H * SCANLINE_BYTES);
    int g, j, y;

    for (g = 0; g < GROUPS; g++) {
        unsigned long p01 = 0, p23 = 0, p45 = 0, p67 = 0;
        unsigned long *dst;
        for (j = 0; j < PER_GROUP; j++) {
            int c = *colours++;
            p01 = p01 << BLOCK_W | s_c2p[0][c];
            p23 = p23 << BLOCK_W | s_c2p[1][c];
            p45 = p45 << BLOCK_W | s_c2p[2][c];
            p67 = p67 << BLOCK_W | s_c2p[3][c];
        }
        dst = line + g * (PLANES / 2);
        for (y = 0; y < BLOCK_H; y++, dst += SCANLINE_BYTES / 4) {
            dst[0] = p01;
            dst[1] = p23;
            dst[2] = p45;
            dst[3] = p67;
        }
    }
}

/* ---------- per-pixel ray fan, precomputed once ---------- */
static float fx_tab[RENDER_W];
static float fy_tab[RENDER_H];

/* ---------- temporal state: warm-start distances and colours ---------- */
static float warm_t[RENDER_H][RENDER_W];          /* last hit t (0 = miss)  */
static unsigned char colours[RENDER_H][RENDER_W]; /* this frame's, and last
                                                     frame's for held ones  */

static void init_ray_tables(void) {
    /* blocks are square on screen, so the fan's aspect is the grid's */
    float aspect = (float)RENDER_W / (float)RENDER_H;
    int i;
    for (i = 0; i < RENDER_W; i++)
        fx_tab[i] = (float)(i - RENDER_W / 2) / (float)RENDER_W * aspect;
    for (i = 0; i < RENDER_H; i++)
        fy_tab[i] = (float)(RENDER_H / 2 - i) / (float)RENDER_H;
}

/* ---------- render one frame ---------- */
static void render_frame(unsigned char *scr, int angle_y, int angle_x, int parity) {
    /* Rotate the camera basis once per frame; every ray is a linear
     * combination of it. */
    Vec3 right = { 1.0f, 0.0f, 0.0f };
    Vec3 up    = { 0.0f, 1.0f, 0.0f };
    Vec3 fwd   = { 0.0f, 0.0f, -1.0f };
    Vec3 ro;
    int cx, cy;

    rot_y(&right, angle_y); rot_x(&right, angle_x);
    rot_y(&up,    angle_y); rot_x(&up,    angle_x);
    rot_y(&fwd,   angle_y); rot_x(&fwd,   angle_x);
    ro.x = -CAM_DIST * fwd.x;   /* camera at (0,0,CAM_DIST), rotated */
    ro.y = -CAM_DIST * fwd.y;
    ro.z = -CAM_DIST * fwd.z;

    for (cy = 0; cy < RENDER_H; cy++) {
        float fy = fy_tab[cy];
        Vec3 row = {
            fwd.x + fy * up.x,
            fwd.y + fy * up.y,
            fwd.z + fy * up.z
        };
        for (cx = 0; cx < RENDER_W; cx++) {
            Vec3 rd;
            float t0, t1, hit_t, tw;
#if CHECKERBOARD
            if ((cx + cy + parity) & 1)     /* held pixel: last frame's   */
                continue;
#else
            (void)parity;
#endif
            colours[cy][cx] = 0;
            if (!ray_setup(&right, &row, fx_tab[cx], &rd, &t0, &t1))
                continue;   /* missed the bounding sphere: stays background */
            tw = warm_t[cy][cx];
            if (tw > 0.0f) {
                tw -= WARM_BACKOFF;
                if (tw > t0)
                    t0 = tw;
            }
            if (!raymarch(&ro, &rd, t0, t1, &hit_t)) {
                warm_t[cy][cx] = 0.0f;
                continue;
            }
            warm_t[cy][cx] = hit_t;
            colours[cy][cx] = (unsigned char)shade(&ro, &rd, hit_t);
        }
        emit_row(scr, cy, colours[cy]);
    }
}

/* ---------- main ---------- */
int main(void)
{
    unsigned char *screens[2];
    tt_state saved;
    int cur = 0, angle_y = 0, angle_x = 0, parity = 0;
    long ssp;

#if HIRES
    if (!tt_check("TORUSHI.TOS"))
#else
    if (!tt_check("TORUS.TOS"))
#endif
        return 1;
    screens[0] = tt_alloc_screen();
    screens[1] = tt_alloc_screen();
    if (!screens[0] || !screens[1]) {
        (void)Cconws("Not enough ST-RAM for two screens.\r\n");
        tt_free_screens();
        return 1;
    }

    init_ray_tables();
    init_c2p();
    tt_ramp(palette + 1, NSHADE, ramp_keys, 15);
    palette[WHITE] = 0xFFF;

    ssp = Super(0L);
    tt_save(&saved);
    tt_set_cacr(TT_CACR_ON);
    tt_start_mode(TT_MODE_TTLOW, screens[1], palette, 256);

    /* main loop - ESC exits */
    while (1) {
        /* render off-screen, then show the buffer we just drew */
        unsigned char *draw = screens[cur];
        unsigned long tick = TT_HZ200;

        render_frame(draw, angle_y, angle_x, parity);
        parity ^= 1;
        tt_draw_ms(draw, TT_MODE_TTLOW, 0, (TT_HZ200 - tick) * 5UL);

        /* base register latches at the NEXT VBL: point it at the finished
         * buffer BEFORE waiting, so when the wait returns the swap has
         * actually happened and the other buffer is safely off-screen */
        tt_set_screen(draw);
        tt_wait_vbl();
        cur ^= 1;

        /* advance rotation: ~360/512 degrees a step per frame */
        angle_y = (angle_y + 2) & 511;
        angle_x = (angle_x + 1) & 511;

        /* ESC exits; poll through BIOS - TOS's keyboard ISR consumes the
         * ACIA byte, so reading $FFFC00/02 directly never sees it */
        if (Bconstat(2)) {
            if ((Bconin(2) & 0xFF) == 27) break;
        }
    }

    tt_restore(&saved);
    Super((void *)ssp);
    tt_free_screens();
    return 0;
}
