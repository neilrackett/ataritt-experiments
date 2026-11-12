/*
 * Copyright (C) 2026 Neil Rackett
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * File: poly.h
 * Description: span rasteriser shared by the polygon demos (cube, knot, jet).
 *
 * Include after defining SCREEN_W, SCREEN_H, SCANLINE_BYTES and PLANES:
 * 8 for TT low res (256 colours), 4 for TT medium (16). Integer only.
 *
 * No clipping: each demo's focal length keeps its model's bounding sphere,
 * and so every vertex, on screen.
 */

#ifndef POLY_H
#define POLY_H

#define POLY_LONGS (PLANES / 2)   /* longs per 16-pixel group: plane pairs */

static short span_min[SCREEN_H], span_max[SCREEN_H];

/* edge masks for both words of a plane pair, indexed by x&15 */
static const unsigned long edge_lmask[16] = {
    0xFFFFFFFFUL, 0x7FFF7FFFUL, 0x3FFF3FFFUL, 0x1FFF1FFFUL,
    0x0FFF0FFFUL, 0x07FF07FFUL, 0x03FF03FFUL, 0x01FF01FFUL,
    0x00FF00FFUL, 0x007F007FUL, 0x003F003FUL, 0x001F001FUL,
    0x000F000FUL, 0x00070007UL, 0x00030003UL, 0x00010001UL,
};
static const unsigned long edge_rmask[16] = {
    0x80008000UL, 0xC000C000UL, 0xE000E000UL, 0xF000F000UL,
    0xF800F800UL, 0xFC00FC00UL, 0xFE00FE00UL, 0xFF00FF00UL,
    0xFF80FF80UL, 0xFFC0FFC0UL, 0xFFE0FFE0UL, 0xFFF0FFF0UL,
    0xFFF8FFF8UL, 0xFFFCFFFCUL, 0xFFFEFFFEUL, 0xFFFFFFFFUL,
};

/* Per-scanline DDA over the edge: one 16.16 x step per line - for span
 * extraction only the x at each scanline matters, so walking every pixel
 * (Bresenham) is wasted work. Endpoints land exactly: the divide is by the
 * full height and the last line adds the remainder implicitly. */
static void trace_edge(int xa, int ya, int xb, int yb)
{
    long x, dx;
    int y;

    if (ya == yb) {
        if ((unsigned)ya < SCREEN_H) {
            if (xa > xb) { int t = xa; xa = xb; xb = t; }
            if (xa < span_min[ya]) span_min[ya] = (short)xa;
            if (xb > span_max[ya]) span_max[ya] = (short)xb;
        }
        return;
    }
    if (ya > yb) {
        int t = xa; xa = xb; xb = t;
        t = ya; ya = yb; yb = t;
    }
    x  = (long)xa << 16;
    dx = ((long)(xb - xa) << 16) / (yb - ya);
    for (y = ya; y <= yb; y++, x += dx) {
        if ((unsigned)y < SCREEN_H) {
            int xi = (int)(x >> 16);
            if (xi < span_min[y]) span_min[y] = (short)xi;
            if (xi > span_max[y]) span_max[y] = (short)xi;
        }
    }
}

/* Fill a convex polygon of n screen vertices in colour c: DDA the edges into
 * span_min/max, then fill the spans a plane pair (one long) at a time, with
 * the colour's plane patterns worked out once per polygon */
static void fill_poly(unsigned char *scr, const short *xs, const short *ys, int n, int c)
{
    unsigned long pat[POLY_LONGS];
    unsigned char *base;
    int ymin = SCREEN_H, ymax = -1;
    int k, y;

    for (k = 0; k < n; k++) {
        if (ys[k] < ymin) ymin = ys[k];
        if (ys[k] > ymax) ymax = ys[k];
    }
    if (ymax < 0 || ymin > SCREEN_H - 1) return;
    for (y = ymin; y <= ymax; y++) { span_min[y] = SCREEN_W; span_max[y] = -1; }
    for (k = 0; k < n; k++) {
        int k1 = k + 1 < n ? k + 1 : 0;
        trace_edge(xs[k], ys[k], xs[k1], ys[k1]);
    }

    /* plane 2k in the high word, plane 2k+1 in the low */
    for (k = 0; k < POLY_LONGS; k++)
        pat[k] = ((c >> (2 * k) & 1) ? 0xFFFF0000UL : 0UL)
               | ((c >> (2 * k + 1) & 1) ? 0x0000FFFFUL : 0UL);

    base = scr + ymin * SCANLINE_BYTES;
    for (y = ymin; y <= ymax; y++, base += SCANLINE_BYTES) {
        int x0 = span_min[y], x1 = span_max[y];
        unsigned long m0, m1, *g, *g1;

        if (x1 < x0)
            continue;
        g  = (unsigned long *)base + (x0 >> 4) * POLY_LONGS;
        g1 = (unsigned long *)base + (x1 >> 4) * POLY_LONGS;
        m0 = edge_lmask[x0 & 15];
        m1 = edge_rmask[x1 & 15];
        if (g == g1) {
            m0 &= m1;
            for (k = 0; k < POLY_LONGS; k++)
                g[k] = (g[k] & ~m0) | (pat[k] & m0);
            continue;
        }
        for (k = 0; k < POLY_LONGS; k++)
            g[k] = (g[k] & ~m0) | (pat[k] & m0);
        for (g += POLY_LONGS; g < g1; g += POLY_LONGS)
            for (k = 0; k < POLY_LONGS; k++)
                g[k] = pat[k];
        for (k = 0; k < POLY_LONGS; k++)
            g1[k] = (g1[k] & ~m1) | (pat[k] & m1);
    }
}

#endif /* POLY_H */
