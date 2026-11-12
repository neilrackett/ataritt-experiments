/*
 * Copyright (C) 2026 Neil Rackett
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * KNOTMESH.C - faceted (2,3) torus-knot mesh for the Atari TT
 *
 * The companion to torus.c's raymarcher: same trefoil, but as a low-poly
 * flat-shaded mesh (TSEG x RSEG facets, Three.js TorusKnotGeometry-style)
 * targeting real frame rates. The 68882 runs the per-vertex rotate/project
 * pipeline; the 68030 does everything integer: backface culling, a
 * depth-bucket painter's sort, fixed-point lighting, and span fills straight
 * into the four bitplanes of TT medium res, 640x480. TTLOW=1 builds it for
 * TT low res instead: 320x480 with 48 shades of the 256 colours on the
 * light's ramp, so neighbouring facets rarely share one.
 *
 * The TT's 68882 sits on the 68030's coprocessor bus, so the transform is
 * plain C built -m68881: the compiler issues real FPU instructions and keeps
 * the intermediates in FP0-FP7 by itself.
 */

#include <string.h>
#include <mint/osbind.h>

#include "atari_tt.h"
#include "sintab.h"

/* ---------- mesh density (the "segments" sliders) ---------- */
#ifndef TSEG
#define TSEG 26           /* tubular segments (along the knot)   */
#endif
#ifndef RSEG
#define RSEG 6            /* radial segments (around the tube)   */
#endif
#define NV (TSEG * RSEG)  /* vertices */
#define NF (TSEG * RSEG)  /* quad faces */

/* ---------- geometry (matches the raymarcher's knot) ---------- */
#define TORUS_R  1.0f
#define KNOT_W   0.35f
#define TUBE_R   0.28f  /* chunky, like the reference shot */
#define CAM_DIST 3.0f
#define PROJ_F   360.0f   /* focal: max |y'|/zc over the r=1.63 sphere at
                             CAM_DIST 3 is 0.648, so +/-233 of the 480 lines;
                             x as wide, in TT med's square pixels or half
                             as many of TT low's wide ones */

/* ---------- screen: TT medium res, or TT low with -DTTLOW=1 ---------- */
#ifndef TTLOW
#define TTLOW 0
#endif
#if !TTLOW
#  define SCREEN_W   640
#  define PLANES     4
#  define VIDEO_MODE TT_MODE_TTMED
#  define X_ASPECT   1.0f             /* square pixels                  */
#  define SHADE_LO   2                /* lit faces: palette 2..15       */
#  define SHADE_N    14
#  define SHADE_BIAS 0
#  define DIGITS_W   48
#else
#  define SCREEN_W   320
#  define PLANES     8
#  define VIDEO_MODE TT_MODE_TTLOW
#  define X_ASPECT   0.5f             /* pixels twice as wide as tall   */
#  define SHADE_LO   1                /* lit faces: palette 1..48       */
#  define SHADE_N    48
/* The 16 colours' shade k covers n.l*13 in [k, k+1): TT low's shades are
 * taken from the middle of each, half a step on, so faces keep the
 * brightness they have in 16 colours and only the steps get finer */
#  define SHADE_BIAS (0x10000L / 26)
#  define DIGITS_W   32
#endif
#define SCREEN_H       480
#define CENTRE_X       (SCREEN_W / 2)
#define CENTRE_Y       (SCREEN_H / 2)
#define SCANLINE_BYTES TT_LINE_BYTES

#include "poly.h"

/* ---------- palette: same cyan->white ramp as the raymarcher ---------- */
/* Its 14 lit colours as 4-bit levels: TT med shows them as they are, TT low
 * spreads SHADE_N shades along them */
static const unsigned char ramp_keys[14][3] = {
    { 0, 3, 4 }, { 0, 4, 6 }, { 1, 6, 8 }, { 1, 7, 9 }, { 2, 8, 10 },
    { 2, 9, 11 }, { 3, 10, 12 }, { 4, 11, 13 }, { 5, 12, 14 }, { 7, 13, 15 },
    { 9, 14, 15 }, { 11, 14, 15 }, { 13, 15, 15 }, { 15, 15, 15 },
};
static unsigned short palette[256];

static void build_palette(void)
{
#if !TTLOW
    int i;
    for (i = 0; i < SHADE_N; i++)           /* 15, the last, is white */
        palette[SHADE_LO + i] = (unsigned short)(ramp_keys[i][0] << 8
                                | ramp_keys[i][1] << 4 | ramp_keys[i][2]);
#else
    tt_ramp(palette + SHADE_LO, SHADE_N, ramp_keys, 14);
    palette[255] = 0xFFF;                   /* the ms digits          */
#endif
}

/* ---------- vec3 ---------- */
typedef struct { float x, y, z; } Vec3;

static Vec3 v_sub(Vec3 a, Vec3 b) {
    Vec3 r = { a.x - b.x, a.y - b.y, a.z - b.z };
    return r;
}

static Vec3 v_cross(Vec3 a, Vec3 b) {
    Vec3 r = {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
    return r;
}

static float v_dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static Vec3 v_norm(Vec3 v) {
    float inv = 1.0f / __builtin_sqrtf(v_dot(v, v));
    Vec3 r = { v.x * inv, v.y * inv, v.z * inv };
    return r;
}

/* ---------- mesh (built once at startup) ---------- */
static Vec3 overts[NV];               /* object-space vertices              */
static short fnorm_fix[NF][3];        /* object-space unit normals, 8.8     */
static unsigned short fvert[NF][4];   /* quad vertex indices, fixed winding */

/* per-frame results */
static short vsx[NV], vsy[NV], vsz[NV];   /* screen x, y, depth (zc * 64)   */

/* ring offsets for RSEG = 6 (exact, no LUT rounding) */
static const float ring_c[6] = { 1.0f, 0.5f, -0.5f, -1.0f, -0.5f, 0.5f };
static const float ring_s[6] = { 0.0f, 0.8660254f, 0.8660254f,
                                 0.0f, -0.8660254f, -0.8660254f };

/*
 * Strand-centre curve of the same (2,3) knot the raymarcher fields:
 *   C(theta) = ((R + W cos(1.5 theta)) cos(theta),
 *                W sin(1.5 theta),
 *               (R + W cos(1.5 theta)) sin(theta)),  theta in [0, 4pi).
 * theta over two revolutions is 1024 LUT steps, psi = 1.5*theta is 1536;
 * both close exactly at i = TSEG. Tube rings sit in the plane spanned by
 * N (re-orthogonalised cross-section normal) and B = T x N.
 */
static void build_mesh(void)
{
    Vec3 curve[TSEG], cnorm[TSEG];
    int i, j;

    for (i = 0; i < TSEG; i++) {
        int thi = (i * 1024) / TSEG;
        int psi = (i * 1536) / TSEG;
        float cth = lut_cos(thi), sth = lut_sin(thi);
        float cps = lut_cos(psi), sps = lut_sin(psi);
        float wc  = TORUS_R + KNOT_W * cps;
        curve[i].x = wc * cth;
        curve[i].y = KNOT_W * sps;
        curve[i].z = wc * sth;
        cnorm[i].x = cth * cps;          /* cross-section radial direction */
        cnorm[i].y = sps;
        cnorm[i].z = sth * cps;
    }

    for (i = 0; i < TSEG; i++) {
        Vec3 t = v_norm(v_sub(curve[(i + 1) % TSEG], curve[(i + TSEG - 1) % TSEG]));
        Vec3 b = v_norm(v_cross(t, cnorm[i]));
        Vec3 n = v_cross(b, t);          /* unit: b, t unit and orthogonal */
        for (j = 0; j < RSEG; j++) {
            Vec3 *v = &overts[i * RSEG + j];
            float rc = TUBE_R * ring_c[j];
            float rs = TUBE_R * ring_s[j];
            v->x = curve[i].x + n.x * rc + b.x * rs;
            v->y = curve[i].y + n.y * rc + b.y * rs;
            v->z = curve[i].z + n.z * rc + b.z * rs;
        }
    }

    for (i = 0; i < TSEG; i++) {
        int i1 = (i + 1) % TSEG;
        for (j = 0; j < RSEG; j++) {
            int j1 = (j + 1) % RSEG;
            int f = i * RSEG + j;
            Vec3 e1, e2, n;
            fvert[f][0] = (unsigned short)(i  * RSEG + j);
            fvert[f][1] = (unsigned short)(i1 * RSEG + j);
            fvert[f][2] = (unsigned short)(i1 * RSEG + j1);
            fvert[f][3] = (unsigned short)(i  * RSEG + j1);
            /* geometric normal from the diagonals, oriented outward */
            e1 = v_sub(overts[fvert[f][2]], overts[fvert[f][0]]);
            e2 = v_sub(overts[fvert[f][3]], overts[fvert[f][1]]);
            n  = v_norm(v_cross(e1, e2));
            if (v_dot(n, v_sub(overts[fvert[f][0]], curve[i])) < 0.0f) {
                n.x = -n.x; n.y = -n.y; n.z = -n.z;
                /* flip winding too, so screen-space culling stays uniform */
                fvert[f][1] = (unsigned short)(i  * RSEG + j1);
                fvert[f][3] = (unsigned short)(i1 * RSEG + j);
            }
            fnorm_fix[f][0] = (short)(int)(n.x * 256.0f);
            fnorm_fix[f][1] = (short)(int)(n.y * 256.0f);
            fnorm_fix[f][2] = (short)(int)(n.z * 256.0f);
        }
    }
}

/* ---------- per-frame rotation matrix (Rx * Ry, same feel as torus.c) ---------- */
static float mrot[9];

static void build_matrix(int ay, int ax)
{
    float cy = lut_cos(ay), sy = lut_sin(ay);
    float cx = lut_cos(ax), sx = lut_sin(ax);
    /* Ry: x' = cy x + sy z ; z' = -sy x + cy z. Rx: y' = cx y - sx z ; z' = sx y + cx z */
    mrot[0] = cy;         mrot[1] = 0.0f; mrot[2] = sy;
    mrot[3] = sx * sy;    mrot[4] = cx;   mrot[5] = -sx * cy;
    mrot[6] = -cx * sy;   mrot[7] = sx;   mrot[8] = cx * cy;
}

/* ---------- vertex transform + perspective, on the 68882 ---------- */
/* Rotate, project and truncate (FINTRZ) every vertex, plus its depth for
 * the painter's buckets. Returns the screen bounding box for the next clear. */
static void xform_vertices(short box[4])
{
    const float m0 = mrot[0] * X_ASPECT, m2 = mrot[2] * X_ASPECT;   /* x' in pixels */
    const float m3 = mrot[3], m4 = mrot[4], m5 = mrot[5];
    const float m6 = mrot[6], m7 = mrot[7], m8 = mrot[8];
    short x0 = SCREEN_W, y0 = SCREEN_H, x1 = -1, y1 = -1;
    int v;

    for (v = 0; v < NV; v++) {
        const float x = overts[v].x, y = overts[v].y, z = overts[v].z;
        /* mrot[1] is structurally 0 for Rx*Ry: term dropped */
        float zc = CAM_DIST - (m6 * x + m7 * y + m8 * z);
        float f = PROJ_F / zc;
        short sx = (short)(CENTRE_X + (int)((m0 * x + m2 * z) * f));
        short sy = (short)(CENTRE_Y - (int)((m3 * x + m4 * y + m5 * z) * f));
        vsx[v] = sx;
        vsy[v] = sy;
        vsz[v] = (short)(int)(zc * 64.0f);    /* depth for the painter buckets */
        if (sx < x0) x0 = sx;
        if (sx > x1) x1 = sx;
        if (sy < y0) y0 = sy;
        if (sy > y1) y1 = sy;
    }
    box[0] = x0; box[1] = y0; box[2] = x1; box[3] = y1;
}

/* ---------- per-face lighting, integer ---------- */
/* With 8.8 fixed-point normals a lighting dot product is three 68030 muls,
 * cheaper than moving the normal through the FPU and the result back. The
 * light is rotated into object space once per frame, so face normals never
 * transform at all. */
static const Vec3 LIGHT = { 0.577f, 0.577f, 0.577f };
static short lobj_fix[3];        /* 8.8 light in object space     */

static void build_light(void)
{
    /* L_obj = R^T * L (rows of R dotted as columns), then to 8.8 */
    lobj_fix[0] = (short)(int)(256.0f * (mrot[0] * LIGHT.x + mrot[3] * LIGHT.y
                                         + mrot[6] * LIGHT.z));
    lobj_fix[1] = (short)(int)(256.0f * (mrot[1] * LIGHT.x + mrot[4] * LIGHT.y
                                         + mrot[7] * LIGHT.z));
    lobj_fix[2] = (short)(int)(256.0f * (mrot[2] * LIGHT.x + mrot[5] * LIGHT.y
                                         + mrot[8] * LIGHT.z));
}

static int face_colour(int f)
{
    long d = (long)fnorm_fix[f][0] * lobj_fix[0]
           + (long)fnorm_fix[f][1] * lobj_fix[1]
           + (long)fnorm_fix[f][2] * lobj_fix[2];   /* 16.16 */
    long idx;
    if (d <= SHADE_BIAS)
        return SHADE_LO;             /* facing away from the light */
    idx = SHADE_LO + (((d - SHADE_BIAS) * (SHADE_N - 1)) >> 16);
    if (idx > SHADE_LO + SHADE_N - 1) idx = SHADE_LO + SHADE_N - 1;
    return (int)idx;
}

/* ---------- one face through poly.h's span filler ---------- */
static void fill_quad(unsigned char *scr, int f, int c)
{
    short xs[4], ys[4];
    int k;

    for (k = 0; k < 4; k++) {
        xs[k] = vsx[fvert[f][k]];
        ys[k] = vsy[fvert[f][k]];
    }
    fill_poly(scr, xs, ys, 4, c);
}

/* ---------- painter's sort: depth buckets, far-to-near ---------- */
#define NBUCKET 256
static short bucket_head[NBUCKET];
static short face_next[NF];

/* ---------- render one frame ---------- */
static unsigned long xform_ms;   /* vertex-pipeline share of the frame */
static unsigned long paint_ms;   /* lighting + span-fill share          */

/* No blitter on the TT, so the 68030 clears - only the box last drawn in */
static void render_frame(unsigned char *scr, short box[4], int angle_y, int angle_x)
{
    unsigned long t0, t1;
    int f, b;

    tt_clear_box(scr, PLANES, box[0], box[1], box[2], box[3]);
    t0 = TT_HZ200;
    build_matrix(angle_y, angle_x);
    build_light();
    xform_vertices(box);
    xform_ms = (TT_HZ200 - t0) * 5UL;

    /* cull + depth-bucket the visible faces (integer only) */
    for (b = 0; b < NBUCKET; b++) bucket_head[b] = -1;
    for (f = 0; f < NF; f++) {
        int v0 = fvert[f][0], v1 = fvert[f][1], v2 = fvert[f][2];
        long area = (long)(vsx[v1] - vsx[v0]) * (vsy[v2] - vsy[v0])
                  - (long)(vsy[v1] - vsy[v0]) * (vsx[v2] - vsx[v0]);
        int depth;
        if (area >= 0)
            continue;                     /* back-facing                    */
        depth = vsz[v0] + vsz[v1] + vsz[v2] + vsz[fvert[f][3]];
        depth >>= 4;                      /* zc*64 x4 summed: /16 -> 0..291 */
        if (depth < 0) depth = 0;
        if (depth > NBUCKET - 1) depth = NBUCKET - 1;
        face_next[f] = bucket_head[depth];
        bucket_head[depth] = (short)f;
    }

    t1 = TT_HZ200;
    /* painter: far (large zc) first */
    for (b = NBUCKET - 1; b >= 0; b--) {
        short f2 = bucket_head[b];
        while (f2 >= 0) {
            fill_quad(scr, f2, face_colour(f2));
            f2 = face_next[f2];
        }
    }
    paint_ms = (TT_HZ200 - t1) * 5UL;
}

/* ---------- main ---------- */
int main(void)
{
    static short boxes[2][4] = { { 0, 0, -1, -1 }, { 0, 0, -1, -1 } };
    unsigned char *screens[2];
    tt_state saved;
    int cur = 0, angle_y = 0, angle_x = 0;
    long ssp;

    if (!tt_check("KNOTMESH.TOS"))
        return 1;
    screens[0] = tt_alloc_screen();
    screens[1] = tt_alloc_screen();
    if (!screens[0] || !screens[1]) {
        (void)Cconws("KNOTMESH.TOS: not enough ST-RAM for two screens.\r\n");
        tt_free_screens();
        return 1;
    }

    ssp = Super(0L);
    tt_save(&saved);
    tt_set_cacr(TT_CACR_ON);
    build_mesh();
    build_palette();
    tt_start_mode(VIDEO_MODE, screens[1], palette, 256);

    while (1) {
        unsigned char *draw = screens[cur];
        unsigned long tick = TT_HZ200;

        render_frame(draw, boxes[cur], angle_y, angle_x);
        tt_clear_box(draw, PLANES, 0, 0, DIGITS_W - 1, 47);
        tt_draw_ms(draw, VIDEO_MODE, 0, (TT_HZ200 - tick) * 5UL); /* frame */
        tt_draw_ms(draw, VIDEO_MODE, 1, xform_ms);                /* FPU   */
        tt_draw_ms(draw, VIDEO_MODE, 2, paint_ms);                /* fills */

        /* base register latches at the NEXT VBL: point it at the finished
         * buffer BEFORE waiting, so when the wait returns the swap has
         * actually happened and the other buffer is safely off-screen */
        tt_set_screen(draw);
        tt_wait_vbl();
        cur ^= 1;

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
