/*
 * Copyright (C) 2026 Neil Rackett
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * JETMESH.C - flat-shaded F-22 for the Atari TT
 *
 * The low-poly F-22 from xms0g's 3DOS (an MS-DOS VGA renderer; model data
 * and its BSD licence in f22.h), flown on the knotmesh.c pipeline: the 68882
 * transforms and projects its 102 vertices; the 68030 does integer backface
 * culling, a depth-bucket painter's sort of the 200 triangles, fixed-point
 * lighting with a ramp per material, and span fills at 640x480 in TT medium
 * res. TTLOW=1 builds it for TT low res instead: 320x480 with 40 shades of
 * the 256 colours for each of the four materials.
 *
 * White numbers top left: total frame ms, FPU transform ms, paint ms
 * (200 Hz system tick).
 */

#include <string.h>
#include <mint/osbind.h>

#include "atari_tt.h"
#include "sintab.h"
#include "f22.h"

#define NV F22_NV
#define NF F22_NF

/* ---------- geometry ---------- */
#define JET_CX   0.1f     /* x of the model's bounding-sphere centre...     */
#define JET_R    1.92f    /* ...and its radius                              */
#define CAM_DIST 4.0f
#define PROJ_F   432.0f   /* max |y'|/zc over that sphere is 0.547: +/-236
                             of the 480 lines; x as wide, in TT med's square
                             pixels or half as many of TT low's wide ones */

/* painter's depth: a triangle's three zc*64 values sum to 399..1134 over
 * zc = CAM_DIST -/+ JET_R; less the near end, >> 2 gives buckets 0..183 */
#define DEPTH_NEAR 399

/* ---------- motion: turntable spin, camera swinging above the wings ---------- */
#define ELEV_MID   24      /* LUT units: 17 degrees...                     */
#define ELEV_SWING 40.0f   /* ...+/-28, from just below the wings to 45    */

/* ---------- screen: TT medium res, or TT low with -DTTLOW=1 ---------- */
#ifndef TTLOW
#define TTLOW 0
#endif
#if !TTLOW
#  define SCREEN_W   640
#  define PLANES     4
#  define VIDEO_MODE TT_MODE_TTMED
#  define X_ASPECT   1.0f             /* square pixels                  */
#  define DIGITS_W   48
#else
#  define SCREEN_W   320
#  define PLANES     8
#  define VIDEO_MODE TT_MODE_TTLOW
#  define X_ASPECT   0.5f             /* pixels twice as wide as tall   */
#  define DIGITS_W   32
#  define SHADES     40               /* per material: palette 1..160   */
#endif
#define SCREEN_H       480
#define CENTRE_X       (SCREEN_W / 2)
#define CENTRE_Y       (SCREEN_H / 2)
#define SCANLINE_BYTES TT_LINE_BYTES

#include "poly.h"

/* ---------- palette: 3DOS's teal sky and paint scheme ---------- */
static const unsigned short scheme[16] = {
    0x188,                                 /* 0      sky                 */
    0x711, 0xD22,                          /* 1-2    engine red          */
    0x127, 0x24B, 0x57F,                   /* 3-5    canopy blue         */
    0x334, 0x445, 0x556, 0x667, 0x889,     /* 6-15   grey ramp, shared   */
    0x99A, 0xBBB, 0xCCC, 0xEEE, 0xFFF,     /*        by hull and wings   */
};

/* Each material's nine lighting levels, darkest first, as colours of the
 * scheme. TT low spreads SHADES shades of its own along each. */
static const unsigned char ramp[4][9] = {
    {  8,  9, 10, 11, 12, 12, 13, 14, 15 },   /* F22_HULL:   light grey */
    {  6,  6,  7,  7,  8,  8,  9,  9, 10 },   /* F22_WING:   dark grey  */
    {  3,  3,  3,  4,  4,  4,  5,  5,  5 },   /* F22_CANOPY: blue       */
    {  1,  1,  1,  1,  2,  2,  2,  2,  2 },   /* F22_ENGINE: red        */
};

#if TTLOW
static unsigned short palette[256];
#else
#define palette scheme
#endif

static void build_palette(void)
{
#if TTLOW
    unsigned char keys[9][3];
    int m, k;

    palette[0] = scheme[0];                 /* sky                   */
    for (m = 0; m < 4; m++) {
        for (k = 0; k < 9; k++) {
            unsigned short c = scheme[ramp[m][k]];
            keys[k][0] = (unsigned char)(c >> 8 & 15);
            keys[k][1] = (unsigned char)(c >> 4 & 15);
            keys[k][2] = (unsigned char)(c & 15);
        }
        tt_ramp(palette + 1 + m * SHADES, SHADES, (const unsigned char (*)[3])keys, 9);
    }
    palette[255] = 0xFFF;                   /* the ms digits         */
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

static Vec3 v_norm(Vec3 v) {
    float inv = 1.0f / __builtin_sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    Vec3 r = { v.x * inv, v.y * inv, v.z * inv };
    return r;
}

/* ---------- mesh (centred and lit once at startup) ---------- */
static Vec3 overts[NV];               /* object-space vertices, centred */
static short fnorm_fix[NF][3];        /* 8.8 object-space face normals  */

/* per-frame results */
static short vsx[NV], vsy[NV], vsz[NV];   /* screen x, y, depth (zc * 64)   */

static void build_mesh(void)
{
    int v, f;

    for (v = 0; v < NV; v++) {
        overts[v].x = f22_verts[v][0] - JET_CX;
        overts[v].y = f22_verts[v][1];
        overts[v].z = f22_verts[v][2];
    }

    /* 3DOS's mesh is closed and consistently wound (every edge shared by
     * two triangles, counter-clockwise from outside), so the edge cross
     * product is the outward normal and screen-space culling needs no
     * winding fix-ups */
    for (f = 0; f < NF; f++) {
        const unsigned char *t = f22_faces[f];
        Vec3 n = v_norm(v_cross(v_sub(overts[t[1]], overts[t[0]]),
                                v_sub(overts[t[2]], overts[t[0]])));
        fnorm_fix[f][0] = (short)(int)(n.x * 256.0f);
        fnorm_fix[f][1] = (short)(int)(n.y * 256.0f);
        fnorm_fix[f][2] = (short)(int)(n.z * 256.0f);
    }
}

/* ---------- per-frame rotation matrix (Rx * Ry) ---------- */
/* Ry spins the jet about its own vertical axis, then Rx tilts the result
 * towards the camera: a turntable, viewed from angle_x above the wings */
static float mrot[9];

static void build_matrix(int ay, int ax)
{
    float cy = lut_cos(ay), sy = lut_sin(ay);
    float cx = lut_cos(ax), sx = lut_sin(ax);
    mrot[0] = cy;         mrot[1] = 0.0f; mrot[2] = sy;
    mrot[3] = sx * sy;    mrot[4] = cx;   mrot[5] = -sx * cy;
    mrot[6] = -cx * sy;   mrot[7] = sx;   mrot[8] = cx * cy;
}

/* ---------- vertex transform + perspective, on the 68882 (as knotmesh.c) ---------- */
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
/* As knotmesh.c, three 68030 muls of 8.8 normals against the light rotated
 * into object space. Half-Lambert: n.l spreads over a material's whole
 * ramp from -1 to 1 rather than clamping at 0, so faces turned from the
 * light still show the airframe's shape. */
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
           + (long)fnorm_fix[f][2] * lobj_fix[2];   /* 16.16, -1..1 */
#if !TTLOW
    int lvl = (int)((d + 0x10000L) >> 14);         /* 0..8          */
    if (lvl < 0) lvl = 0;
    if (lvl > 8) lvl = 8;
    return ramp[f22_faces[f][3]][lvl];
#else
    /* level k of the nine covers (d+1)*4 in [k, k+1): take the shade at
     * the middle of it, (d+1)*4 - 1/2, so faces keep the brightness they
     * have in 16 colours and only the steps get finer */
    long pos = (d + 0x10000L) * 4 - 0x8000L;        /* 16.16, 0..8   */
    int lvl = pos <= 0 ? 0 : (int)((pos * (SHADES - 1)) >> 19);
    if (lvl > SHADES - 1) lvl = SHADES - 1;
    return 1 + f22_faces[f][3] * SHADES + lvl;
#endif
}

/* ---------- one triangle through poly.h's span filler ---------- */
static void fill_tri(unsigned char *scr, int f, int c)
{
    short xs[3], ys[3];
    int k;

    for (k = 0; k < 3; k++) {
        xs[k] = vsx[f22_faces[f][k]];
        ys[k] = vsy[f22_faces[f][k]];
    }
    fill_poly(scr, xs, ys, 3, c);
}

/* ---------- painter's sort: depth buckets, far-to-near ---------- */
#define NBUCKET 256
static short bucket_head[NBUCKET];
static short face_next[NF];

/* ---------- render one frame ---------- */
static unsigned long xform_ms;   /* vertex-pipeline share of the frame */
static unsigned long paint_ms;   /* lighting + span-fill share          */

/* The sky is colour 0, so a clear paints it. No blitter on the TT: the
 * 68030 clears, and only the box last drawn in. */
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

    /* cull + depth-bucket the visible triangles (integer only) */
    for (b = 0; b < NBUCKET; b++) bucket_head[b] = -1;
    for (f = 0; f < NF; f++) {
        int v0 = f22_faces[f][0], v1 = f22_faces[f][1], v2 = f22_faces[f][2];
        long area = (long)(vsx[v1] - vsx[v0]) * (vsy[v2] - vsy[v0])
                  - (long)(vsy[v1] - vsy[v0]) * (vsx[v2] - vsx[v0]);
        int depth;
        if (area >= 0)
            continue;                     /* back-facing */
        depth = (vsz[v0] + vsz[v1] + vsz[v2] - DEPTH_NEAR) >> 2;
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
            fill_tri(scr, f2, face_colour(f2));
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
    int cur = 0;
    int spin = 0;   /* yaw (LUT units, 512 = 360 degrees)  */
    int bob = 0;    /* phase of the camera-elevation swing */
    long ssp;

    if (!tt_check("JETMESH.TOS"))
        return 1;
    screens[0] = tt_alloc_screen();
    screens[1] = tt_alloc_screen();
    if (!screens[0] || !screens[1]) {
        (void)Cconws("JETMESH.TOS: not enough ST-RAM for two screens.\r\n");
        tt_free_screens();
        return 1;
    }

    ssp = Super(0L);
    tt_save(&saved);
    tt_set_cacr(TT_CACR_ON);
    build_mesh();
    build_palette();
    tt_start_mode(VIDEO_MODE, screens[1], palette,
                  (int)(sizeof(palette) / sizeof(palette[0])));

    while (1) {
        unsigned char *draw = screens[cur];
        unsigned long tick = TT_HZ200;
        int elev = ELEV_MID + (int)(lut_sin(bob) * ELEV_SWING);

        render_frame(draw, boxes[cur], spin, elev);
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

        spin = (spin + 2) & LUT_MASK;
        bob  = (bob + 3) & LUT_MASK;

        if (Bconstat(2)) {
            if ((Bconin(2) & 0xFF) == 27) break;   /* ESC exits */
        }
    }

    tt_restore(&saved);
    Super((void *)ssp);
    tt_free_screens();
    return 0;
}
