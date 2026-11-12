/*
 * Copyright (C) 2026 Neil Rackett
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * CUBEMESH.C - flat-shaded spinning cube for the Atari TT
 *
 * The traditional one: a benchmark-friendly companion to knotmesh.c. Same
 * pipeline, minimum geometry: the 68882 transforms and projects 8 vertices
 * a frame; the 68030 does integer backface culling, fixed-point lighting,
 * and span fills at 640x480 in TT medium res (or in TT low, 320x480 in 256
 * colours, with TTLOW=1). A cube is convex, so backface culling alone gives
 * correct visibility - no depth sort, and no per-vertex depth either.
 *
 * The TT's 68882 sits on the 68030's coprocessor bus, so the transform is
 * plain C built -m68881: the compiler issues real FPU instructions and keeps
 * the intermediates in FP0-FP7 by itself.
 *
 * White numbers top left: total frame ms, FPU transform ms, paint ms
 * (200 Hz system tick).
 */

#include <string.h>
#include <mint/osbind.h>

#include "atari_tt.h"
#include "sintab.h"

#define NV 8
#define NF 6

/* ---------- geometry ---------- */
#define CUBE_S   0.75f    /* half edge: bounding sphere r = 1.30            */
#define CAM_DIST 3.0f
#define PROJ_F   432.0f   /* max |y'|/zc over that sphere is 0.481: +/-208
                             of the 480 lines; x as wide, in TT med's square
                             pixels or half as many of TT low's wide ones   */

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

/* ---------- palette: the cyan->white ramp of the other demos ---------- */
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

/* ---------- mesh ---------- */
static Vec3 overts[NV];
static short fnorm_fix[NF][3];        /* 8.8 object-space face normals */
static unsigned short fvert[NF][4];   /* quad vertex indices, outward winding */

static short vsx[NV], vsy[NV];        /* per-frame screen coordinates */

static void build_mesh(void)
{
    /* vertex i: bit 0 = +x, bit 1 = +y, bit 2 = +z */
    static const unsigned char face[NF][4] = {
        { 0, 1, 3, 2 },   /* z- */
        { 4, 5, 7, 6 },   /* z+ */
        { 0, 1, 5, 4 },   /* y- */
        { 2, 3, 7, 6 },   /* y+ */
        { 0, 2, 6, 4 },   /* x- */
        { 1, 3, 7, 5 },   /* x+ */
    };
    int i, f, k;

    for (i = 0; i < NV; i++) {
        overts[i].x = (i & 1) ? CUBE_S : -CUBE_S;
        overts[i].y = (i & 2) ? CUBE_S : -CUBE_S;
        overts[i].z = (i & 4) ? CUBE_S : -CUBE_S;
    }

    for (f = 0; f < NF; f++) {
        Vec3 e1, e2, n, v0;
        for (k = 0; k < 4; k++)
            fvert[f][k] = face[f][k];
        /* geometric normal from the diagonals; the cube is centred on the
         * origin, so n.v0 > 0 means outward - flip winding to match */
        v0 = overts[fvert[f][0]];
        e1.x = overts[fvert[f][2]].x - v0.x;
        e1.y = overts[fvert[f][2]].y - v0.y;
        e1.z = overts[fvert[f][2]].z - v0.z;
        e2.x = overts[fvert[f][3]].x - overts[fvert[f][1]].x;
        e2.y = overts[fvert[f][3]].y - overts[fvert[f][1]].y;
        e2.z = overts[fvert[f][3]].z - overts[fvert[f][1]].z;
        n.x = e1.y * e2.z - e1.z * e2.y;
        n.y = e1.z * e2.x - e1.x * e2.z;
        n.z = e1.x * e2.y - e1.y * e2.x;
        if (n.x * v0.x + n.y * v0.y + n.z * v0.z < 0.0f) {
            unsigned short t = fvert[f][1];
            fvert[f][1] = fvert[f][3];
            fvert[f][3] = t;
            n.x = -n.x; n.y = -n.y; n.z = -n.z;
        }
        /* the cross product of an axis-aligned face's diagonals is itself
         * axis-aligned, so the unit normal is just the component signs */
        fnorm_fix[f][0] = (short)(n.x > 0.0f ? 256 : n.x < 0.0f ? -256 : 0);
        fnorm_fix[f][1] = (short)(n.y > 0.0f ? 256 : n.y < 0.0f ? -256 : 0);
        fnorm_fix[f][2] = (short)(n.z > 0.0f ? 256 : n.z < 0.0f ? -256 : 0);
    }
}

/* ---------- per-frame rotation matrix (Rx * Ry) ---------- */
static float mrot[9];

static void build_matrix(int ay, int ax)
{
    float cy = lut_cos(ay), sy = lut_sin(ay);
    float cx = lut_cos(ax), sx = lut_sin(ax);
    mrot[0] = cy;         mrot[1] = 0.0f; mrot[2] = sy;
    mrot[3] = sx * sy;    mrot[4] = cx;   mrot[5] = -sx * cy;
    mrot[6] = -cx * sy;   mrot[7] = sx;   mrot[8] = cx * cy;
}

/* ---------- vertex transform + perspective, on the 68882 ---------- */
/* No depth out: a convex solid is fully visibility-sorted by backface
 * culling. Returns the vertices' screen bounding box for the next clear. */
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
        if (sx < x0) x0 = sx;
        if (sx > x1) x1 = sx;
        if (sy < y0) y0 = sy;
        if (sy > y1) y1 = sy;
    }
    box[0] = x0; box[1] = y0; box[2] = x1; box[3] = y1;
}

/* ---------- per-face lighting, integer (as knotmesh.c) ---------- */
static const Vec3 LIGHT = { 0.577f, 0.577f, 0.577f };
static short lobj_fix[3];        /* 8.8 light in object space */

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

/* ---------- render one frame ---------- */
static unsigned long xform_ms;
static unsigned long paint_ms;

/* No blitter on the TT, so the 68030 clears - only the box last drawn in */
static void render_frame(unsigned char *scr, short box[4], int angle_y, int angle_x)
{
    unsigned long t0, t1;
    int f;

    tt_clear_box(scr, PLANES, box[0], box[1], box[2], box[3]);
    t0 = TT_HZ200;
    build_matrix(angle_y, angle_x);
    build_light();
    xform_vertices(box);
    xform_ms = (TT_HZ200 - t0) * 5UL;

    t1 = TT_HZ200;
    /* convex: backface culling is the whole visibility story */
    for (f = 0; f < NF; f++) {
        int v0 = fvert[f][0], v1 = fvert[f][1], v2 = fvert[f][2];
        long area = (long)(vsx[v1] - vsx[v0]) * (vsy[v2] - vsy[v0])
                  - (long)(vsy[v1] - vsy[v0]) * (vsx[v2] - vsx[v0]);
        if (area >= 0)
            continue;
        fill_quad(scr, f, face_colour(f));
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

    if (!tt_check("CUBEMESH.TOS"))
        return 1;
    screens[0] = tt_alloc_screen();
    screens[1] = tt_alloc_screen();
    if (!screens[0] || !screens[1]) {
        (void)Cconws("CUBEMESH.TOS: not enough ST-RAM for two screens.\r\n");
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

        if (Bconstat(2)) {
            if ((Bconin(2) & 0xFF) == 27) break;   /* ESC exits */
        }
    }

    tt_restore(&saved);
    Super((void *)ssp);
    tt_free_screens();
    return 0;
}
