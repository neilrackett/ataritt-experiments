/*
 * Copyright (C) 2026 Neil Rackett
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * MANDEL.C - an endless, shining Mandelbrot zoom for the Atari TT
 *
 * Dives into the Mandelbrot set on its own, forever. Each frame zooms in 2x
 * on a Misiurewicz point, where the set repeats its spirals and branches at
 * every depth, down to where the FPU's 64-bit mantissa runs out (2^46 in);
 * then the next dive starts, at another point. The 68882, on the 68030's
 * coprocessor bus, iterates z = z^2 + c in its 80-bit registers in a loop
 * of inline asm and branches on its own compare (FBOGT) for escape; the
 * 68030 turns where z escaped into a smooth colour value. The picture is
 * TT low res, 320x480 in 256 colours: 208 of them make the colour bands,
 * and a VBL interrupt rotates those palette entries all the while, so the
 * bands shine outwards even while a frame is coming in.
 *
 * Zooming in 2x on the screen's centre pixel puts every other pixel of the
 * new frame on a pixel of the old one, so a frame starts as the old one's
 * middle, magnified, and only the pixels in between are new. Those whose
 * neighbours agree to within a colour step are guessed rather than worked
 * out; guesses are never carried into the next frame, so a filament that
 * slips between neighbours is back a frame later.
 *
 * Any key shows or hides the info bar, ESC quits.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <mint/osbind.h>

#include "atari_tt.h"

/* ---------- screen: TT low res, pixels twice as wide as they are tall ---------- */
#define SCREEN_W       320
#define SCREEN_H       480
#define BAR_H          32       /* the info bar: two lines of the 8x16 font */
#define BAR_Y          (SCREEN_H - BAR_H)
#define CENTRE_X       160      /* the pixel on the target                */
#define CENTRE_Y       240
#define SCANLINE_BYTES TT_LINE_BYTES

/* ---------- the zoom ---------- */
#define START_W   3.36          /* width of the first frame of a dive     */
#define MAX_STEP  46            /* 2^46 in: about where 80 bits run out   */
#define BAILOUT   256           /* |z|^2 to escape, big for smooth colours */
#define GUESS_T   16            /* neighbours within one colour step       */

/* ---------- colours ---------- */
#define RAMP       208          /* shining colours: palette entries 1-208 */
#define PER_ITER   8            /* ramp steps per iteration: 26 a cycle    */
#define SHINE_STEP 2            /* ramp steps the shine moves each VBL     */
#define COL_BAR    254          /* info bar background                     */
#define COL_TEXT   255          /* info bar text: differs from it in plane 0 */

/* s_val: ramp steps, with these special values */
#define INSIDE    0x7ffe        /* a point in the set                      */
#define UNKNOWN   0xffff        /* not worked out yet                      */
#define GUESSED   0x8000        /* flag: guessed, not worked out           */
#define VALUE(v)  ((v) & 0x7fff)

/* Circular key colours (4 bits per channel) that the shine runs through */
typedef struct {
    const char *name;
    int n;
    unsigned char rgb[8][3];
} palette_t;

static const palette_t s_palettes[] = {
    { "ICE",     6, { {1,2,8}, {3,6,13}, {7,11,15}, {15,15,15}, {8,12,15}, {3,6,12} } },
    { "SUNSET",  6, { {3,0,8}, {9,0,9}, {15,3,4}, {15,9,0}, {15,15,6}, {14,6,2} } },
    { "GOLD",    6, { {4,1,0}, {10,4,0}, {15,10,2}, {15,15,9}, {13,9,2}, {8,3,0} } },
    { "NEON",    6, { {5,0,8}, {12,0,12}, {15,6,10}, {10,15,4}, {15,15,10}, {6,13,2} } },
    { "PINK",    6, { {6,0,4}, {12,2,8}, {15,8,12}, {15,14,15}, {12,6,14}, {7,2,10} } },
};

/* Misiurewicz points: the orbit of 0 falls onto a repelling cycle, so the
 * set looks much the same around them at every depth, and escape counts
 * grow only slowly as the zoom goes in */
typedef struct {
    const char *name;
    double re, im;
    int pal;
} target_t;

static const target_t s_targets[] = {
    { "NORTH ANTENNA",  -0.10109636384562219, 0.95628651080914151, 0 },
    { "ELEPHANT VALLEY", 0.31323493159149535, 0.02548432002954753, 1 },
    { "SPIKE",          -1.54368901269207637, 0.00000000000000000, 2 },
    { "QUARTER BULB",    0.29705677781801204, 0.61042818300878698, 3 },
    { "WEST SPIRAL",    -1.41911618645799287, 0.12569543584899998, 4 },
};
#define NUM_TARGETS ((int)(sizeof(s_targets) / sizeof(s_targets[0])))

/* ---------- state ---------- */
static unsigned short s_val[SCREEN_H][SCREEN_W];
static unsigned char *s_screen;
static int  s_maxiter;
static int  s_bar;                      /* info bar shown */
static int  s_quit;

/* the view: c(x, y) = (re0 + x * dx, im0 - y * dy), dy = dx / 2 */
static double s_re0, s_im0, s_dx, s_ndy;
typedef union { float f; unsigned long u; } fbits;

/* ---------- shine: the VBL rotates palette entries 1-RAMP ---------- */
static unsigned short s_fine[2][RAMP];          /* TT 0RGB colours        */
static const unsigned short * volatile s_ramp;  /* the one being shown    */

static void shine_vbl(void)
{
    static int phase;
    const unsigned short *ramp = (const unsigned short *)s_ramp;
    volatile unsigned short *pal = TT_PALETTE + 1;
    int k;

    if (!ramp)
        return;
    phase += SHINE_STEP;
    if (phase >= RAMP)
        phase -= RAMP;
    for (k = phase; k < RAMP; k++)
        *pal++ = ramp[k];
    for (k = 0; k < phase; k++)
        *pal++ = ramp[k];
}

/* Interpolate a palette's key colours round into RAMP steps, and show it */
static void set_palette(const palette_t *p)
{
    static int which;
    unsigned short *fine = s_fine[which ^= 1];
    unsigned short round[RAMP + 1];
    unsigned char keys[9][3];

    /* the first key again at the end, to come full circle */
    memcpy(keys, p->rgb, sizeof(keys[0]) * p->n);
    memcpy(keys[p->n], p->rgb[0], sizeof(keys[0]));
    tt_ramp(round, RAMP + 1, (const unsigned char (*)[3])keys, p->n + 1);
    memcpy(fine, round, sizeof(s_fine[0]));
    s_ramp = fine;          /* one long write: the VBL sees old or new */
}

/* ---------- smooth colour ---------- */
static unsigned char s_log2frac[256];   /* 256 * log2(1 + i/256) */

/*
 * The smooth escape value n + 2 - log2(log2 |z|^2), in ramp steps, for z
 * escaping at iteration n. |z|^2 comes as a single's bits; each log2 is its
 * exponent plus a table lookup on the top bits of its mantissa, in 8.8
 * fixed point - plenty for colours, and cheaper than two FLOG2s.
 */
static unsigned short smooth(int n, unsigned long zz)
{
    long l = ((long)((zz >> 23) & 0xff) - 127) * 256 + s_log2frac[(zz >> 15) & 0xff];
    long ll, v;
    int p = 14;

    while (!(l >> p))                   /* l >= 8 * 256: |z|^2 > 256 */
        p--;
    ll = (long)(p - 8) * 256 + s_log2frac[(l << (15 - p) >> 7) & 0xff];
    v = (((long)(n + 2) * 256 - ll) * PER_ITER) >> 8;
    return (unsigned short)(v < 0 ? 0 : v > INSIDE - 1 ? INSIDE - 1 : v);
}

/* ---------- the fractal on the 68882 ---------- */
#define STR_(x) #x
#define STR(x)  STR_(x)

/* An 80-bit register-to-register FPU instruction */
#define FX(op, src, dst) "f" op ".x %%fp" #src ",%%fp" #dst "\n\t"

/* One iteration, zr in FPa and zi in FPb: FPf = zr^2, FPa = zr zi,
 * FPb = zi^2, then zi' = 2 zr zi + ci in FPa and zr' = zr^2 - zi^2 + cr in
 * FPf. The new z is (FPf, FPa) and FPb is free, so three iterations take z
 * round FP0, FP1 and FP4 and back, with no register copied back. */
#define SQUARE(a, b, f) FX("move", a, f) FX("mul", a, f) FX("mul", b, a) FX("mul", b, b)
#define ADD_C(a, b, f)  FX("add", a, a) FX("add", 3, a) FX("sub", b, f) FX("add", 2, f)

/*
 * Iterate z = z^2 + c from z = c, all in the FPU's registers: z in two of
 * FP0, FP1 and FP4, c in FP2/FP3 (from the view's doubles, then 80-bit
 * throughout), the bailout in FP6 and |z|^2 in FP7. Escape is tested every
 * third iteration, which the smooth value doesn't notice: |z|^2 is further
 * past the bailout by then, and its log-log has gone up by one for each
 * iteration over.
 */
static unsigned short sample(int x, int y)
{
    long n;
    unsigned long zz;

    __asm__ __volatile__(
        "fmove.l  %[x],%%fp2\n\t"
        "fmul.d   %[dx],%%fp2\n\t"
        "fadd.d   %[re0],%%fp2\n\t"             /* cr = re0 + x * dx   */
        "fmove.l  %[y],%%fp3\n\t"
        "fmul.d   %[ndy],%%fp3\n\t"
        "fadd.d   %[im0],%%fp3\n\t"             /* ci = im0 - y * dy   */
        FX("move", 2, 0)
        FX("move", 3, 1)                        /* z = c               */
        "fmove.l  #" STR(BAILOUT) ",%%fp6\n\t"
        "moveq    #1,%[n]\n"
        "1:\n\t"
        SQUARE(0, 1, 4)
        FX("move", 4, 7) FX("add", 1, 7)        /* |z|^2 > bailout?    */
        FX("cmp", 6, 7)
        "fbogt    2f\n\t"
        ADD_C(0, 1, 4)                          /* z in FP4, FP0       */
        SQUARE(4, 0, 1) ADD_C(4, 0, 1)          /* z in FP1, FP4       */
        SQUARE(1, 4, 0) ADD_C(1, 4, 0)          /* z in FP0, FP1       */
        "addq.l   #3,%[n]\n\t"
        "cmp.l    %[max],%[n]\n\t"
        "jlt      1b\n\t"
        "moveq    #0,%[n]\n\t"                  /* never escaped       */
        "jra      3f\n"
        "2:\n\t"
        "fmove.s  %%fp7,%[zz]\n"                /* |z|^2 as a single   */
        "3:"
        : [n] "=&d"(n), [zz] "=&d"(zz)
        : [x] "d"(x), [y] "d"(y), [max] "d"((long)s_maxiter),
          [dx] "m"(s_dx), [re0] "m"(s_re0), [ndy] "m"(s_ndy), [im0] "m"(s_im0)
        : "fp0", "fp1", "fp2", "fp3", "fp4", "fp6", "fp7", "cc");

    return n ? smooth((int)n, zz) : INSIDE;
}

/* ---------- drawing ---------- */
/* s_colour[v]: a value's palette entry, 1 + its place on the ramp, or 0
 * (black) inside the set. s_c2p[k][c]: colour c's planes 2k and 2k+1 as
 * bit 16 and bit 0, so a line is built 16 pixels at a time, two planes to
 * a long, already in the order they sit in memory. */
static unsigned char s_colour[0x8000];
static unsigned long s_c2p[4][256];

static void init_tables(void)
{
    int v, c, k, i, place = 0;

    for (v = 0; v < 0x8000; v++) {
        s_colour[v] = (unsigned char)(1 + place);
        if (++place == RAMP)
            place = 0;
    }
    s_colour[INSIDE] = s_colour[VALUE(UNKNOWN)] = 0;

    for (k = 0; k < 4; k++)
        for (c = 0; c < 256; c++)
            s_c2p[k][c] = (unsigned long)(c >> (2 * k) & 1) << 16
                        | (unsigned long)(c >> (2 * k + 1) & 1);

    for (i = 0; i < 256; i++)
        s_log2frac[i] = (unsigned char)(256.0 * log(1.0 + i / 256.0) / M_LN2 + 0.5);
}

/* Draw screen line y from a row of values, pixel x showing src[x & mask]
 * (a mask of ~1 doubles them across). Not over the info bar. */
static void draw_line(int y, const unsigned short *src, unsigned short mask)
{
    unsigned long *dst;
    int x, i;

    if (s_bar && y >= BAR_Y)
        return;
    dst = (unsigned long *)(s_screen + y * SCANLINE_BYTES);
    for (x = 0; x < SCREEN_W; x += 16) {
        unsigned long p01 = 0, p23 = 0, p45 = 0, p67 = 0;
        for (i = 0; i < 16; i++) {
            int c = s_colour[VALUE(src[(x + i) & mask])];
            p01 = p01 << 1 | s_c2p[0][c];
            p23 = p23 << 1 | s_c2p[1][c];
            p45 = p45 << 1 | s_c2p[2][c];
            p67 = p67 << 1 | s_c2p[3][c];
        }
        *dst++ = p01;
        *dst++ = p23;
        *dst++ = p45;
        *dst++ = p67;
    }
}

/* ---------- info bar, in TOS's own 8x16 font ---------- */
static const unsigned char *s_font;     /* glyph rows, form_width apart  */
static const unsigned short *s_font_off; /* each glyph's x, in bits      */
static int s_font_first, s_font_last, s_font_form;

/* Line-A init hands back the system fonts' headers: 6x6, 8x8, 8x16 */
static void font_init(void)
{
    const unsigned char *h;

    __asm__ __volatile__(".word 0xA000\n\t"
                         "move.l 8(%%a1),%0"
                         : "=d"(h) : : "d0", "d1", "d2", "a0", "a1", "a2", "cc", "memory");
    if (!h || *(const unsigned short *)(h + 82) != 16)
        return;                         /* no 8x16 font: no text         */
    s_font_first = *(const unsigned short *)(h + 36);
    s_font_last  = *(const unsigned short *)(h + 38);
    s_font_off   = *(const unsigned short * const *)(h + 72);
    s_font       = *(const unsigned char * const *)(h + 76);
    s_font_form  = *(const unsigned short *)(h + 80);
}

/* 40 characters at bar line `line`: the text and bar colours differ only
 * in plane 0, so that plane is the glyph and the other seven are set */
static void draw_text(int line, const char *s)
{
    int col, r, p;

    for (col = 0; col < SCREEN_W / 8; col++) {
        int ch = *s ? (unsigned char)*s++ : ' ';
        unsigned char *dst = s_screen + (BAR_Y + line * 16) * SCANLINE_BYTES
                           + (col >> 1) * 16 + (col & 1);
        const unsigned char *glyph = 0;

        if (s_font && ch >= s_font_first && ch <= s_font_last)
            glyph = s_font + (s_font_off[ch - s_font_first] >> 3);
        for (r = 0; r < 16; r++, dst += SCANLINE_BYTES) {
            dst[0] = glyph ? glyph[r * s_font_form] : 0;
            for (p = 1; p < 8; p++)
                dst[p * 2] = 0xff;
        }
    }
}

static int  s_info_t, s_info_step;
static double s_info_w;
static long s_info_secs;

/* [-]i.dddddddd */
static char *fmt_coord(char *p, double v)
{
    double a = v < 0 ? -v : v;
    long i = (long)a;
    long f = (long)((a - i) * 100000000.0 + 0.5);
    if (f >= 100000000L) { i++; f -= 100000000L; }
    return p + sprintf(p, "%c%ld.%08ld", v < 0 ? '-' : '+', i, f);
}

/* d.dE-ee */
static char *fmt_sci(char *p, double v)
{
    int e = 0;
    long m;
    while (v < 1.0) { v *= 10.0; e--; }
    while (v >= 10.0) { v /= 10.0; e++; }
    m = (long)(v * 10.0 + 0.5);
    if (m >= 100) { m = 10; e++; }
    return p + sprintf(p, "%ld.%ldE%c%02d", m / 10, m % 10, e < 0 ? '-' : '+', e < 0 ? -e : e);
}

/* Show the current dive and depth, and the last frame's time (or ...) */
static void show_info(void)
{
    char line[64], *p = line;
    const target_t *tg = &s_targets[s_info_t];

    if (!s_bar)
        return;
    p += sprintf(p, "%d %-15s ZOOM 2^%-2d %-7s", s_info_t, tg->name, s_info_step,
                 s_palettes[tg->pal].name);
    if (s_info_secs >= 0)
        sprintf(p, "%4ldS", s_info_secs > 9999 ? 9999 : s_info_secs);
    else
        sprintf(p, "  ...");
    draw_text(0, line);

    p = line;
    p += sprintf(p, "RE ");
    p = fmt_coord(p, tg->re);
    p += sprintf(p, " IM ");
    p = fmt_coord(p, tg->im);
    p += sprintf(p, " W ");
    fmt_sci(p, s_info_w);
    draw_text(1, line);
}

/* Any key shows or hides the info bar; ESC quits */
static void poll_keys(void)
{
    unsigned short row[SCREEN_W];
    int x, y;

    while (Bconstat(2)) {
        if ((Bconin(2) & 0xFF) == 27) {
            s_quit = 1;
            continue;
        }
        s_bar = !s_bar;
        if (s_bar) {
            show_info();
            continue;
        }
        /* the fractal comes back from under the bar; pixels not worked
         * out yet show the block they're in */
        for (y = BAR_Y; y < SCREEN_H; y++) {
            for (x = 0; x < SCREEN_W; x++) {
                int m = 0;
                while (m < 8 && s_val[y & ~m][x & ~m] == UNKNOWN)
                    m = m * 2 + 1;
                row[x] = m < 8 ? s_val[y & ~m][x & ~m] : INSIDE;
            }
            draw_line(y, row, 0xffff);
        }
    }
}

/* ---------- frames ---------- */

/* The value at (x, y), between neighbours a-d: if they agree to within a
 * colour step, guess their average; otherwise work it out */
static unsigned short guess(unsigned short a, unsigned short b,
                            unsigned short c, unsigned short d, int x, int y)
{
    unsigned short lo, hi;

    a = VALUE(a); b = VALUE(b); c = VALUE(c); d = VALUE(d);
    lo = hi = a;
    if (b < lo) lo = b;
    if (b > hi) hi = b;
    if (c < lo) lo = c;
    if (c > hi) hi = c;
    if (d < lo) lo = d;
    if (d > hi) hi = d;
    if (hi == INSIDE ? lo == INSIDE : hi - lo <= GUESS_T)
        return (unsigned short)(GUESSED | (hi == INSIDE ? INSIDE : (a + b + c + d + 2) >> 2));
    return sample(x, y);
}

/* The first frame of a dive: blocks of 8, then 4, 2 and 1, each pixel
 * between the corners of the block of twice its size */
static void render_full(void)
{
    static const int steps[] = { 8, 4, 2, 1 };
    int p, x, y, i;

    memset(s_val, 0xff, sizeof(s_val));     /* all UNKNOWN */
    for (p = 0; p < 4 && !s_quit; p++) {
        int s = steps[p], s2 = s * 2;
        for (y = 0; y < SCREEN_H && !s_quit; y += s) {
            int y0 = y & ~(s2 - 1);
            int y1 = y0 == y || y0 + s2 >= SCREEN_H ? y0 : y0 + s2;
            for (x = 0; x < SCREEN_W; x += s) {
                int x0 = x & ~(s2 - 1), x1;
                if (p == 0) {
                    s_val[y][x] = sample(x, y);
                    continue;
                }
                if (x0 == x && y0 == y)
                    continue;               /* there from a pass before */
                x1 = x0 == x || x0 + s2 >= SCREEN_W ? x0 : x0 + s2;
                s_val[y][x] = guess(s_val[y0][x0], s_val[y0][x1],
                                    s_val[y1][x0], s_val[y1][x1], x, y);
            }
            for (i = 0; i < s && y + i < SCREEN_H; i++)
                draw_line(y + i, s_val[y], (unsigned short)~(s - 1));
            poll_keys();
        }
    }
}

/* Move the old frame's middle onto every other pixel of the new one. Each
 * new pixel's source is nearer the centre than it is, so going in from the
 * edges never reads a pixel already moved. */
static void magnify(void)
{
    int y, x, i;

    for (i = 0; i < SCREEN_H / 2; i += 2) {
        int rows[2] = { i, SCREEN_H - 2 - i };      /* from the top and bottom in */
        for (y = 0; y < 2; y++) {
            unsigned short *dst = s_val[rows[y]];
            const unsigned short *src = s_val[rows[y] / 2 + CENTRE_Y / 2];
            for (x = 0; x < CENTRE_X; x += 2)       /* left half, from the left */
                dst[x] = src[x / 2 + CENTRE_X / 2];
            for (x = SCREEN_W - 2; x >= CENTRE_X; x -= 2)   /* right, from the right */
                dst[x] = src[x / 2 + CENTRE_X / 2];
            for (x = 1; x < SCREEN_W; x += 2)
                dst[x] = UNKNOWN;
        }
    }
    for (y = 1; y < SCREEN_H; y += 2)
        memset(s_val[y], 0xff, sizeof(s_val[y]));
}

/* Zoom in 2x on the centre: the old frame's middle, magnified, gives every
 * other pixel; the ones in between are new */
static void render_zoom(void)
{
    int x, y;

    magnify();
    for (y = 0; y < SCREEN_H; y += 2) {
        draw_line(y, s_val[y], 0xfffe);
        draw_line(y + 1, s_val[y], 0xfffe);
    }

    /* work out the guesses carried over, and points the old frame gave
     * up on, which may escape with more iterations */
    for (y = 0; y < SCREEN_H && !s_quit; y += 2) {
        unsigned short *row = s_val[y];
        int changed = 0;
        for (x = 0; x < SCREEN_W; x += 2) {
            unsigned short v = row[x];
            if ((v & GUESSED) || v == INSIDE) {
                row[x] = sample(x, y);
                changed |= row[x] != VALUE(v);
            }
        }
        if (changed) {
            draw_line(y, row, 0xfffe);
            draw_line(y + 1, row, 0xfffe);
        }
        poll_keys();
    }

    /* then the pixels in between, from the worked-out ones around them */
    for (y = 0; y < SCREEN_H && !s_quit; y++) {
        unsigned short *row = s_val[y];
        if (!(y & 1)) {
            for (x = 1; x < SCREEN_W; x += 2) {
                unsigned short a = row[x - 1], b = x + 1 < SCREEN_W ? row[x + 1] : a;
                row[x] = guess(a, b, a, b, x, y);
            }
        } else {
            const unsigned short *up = s_val[y - 1];
            const unsigned short *dn = y + 1 < SCREEN_H ? s_val[y + 1] : up;
            for (x = 0; x < SCREEN_W; x++) {
                int l = x & ~1, r = (x & 1) && x + 1 < SCREEN_W ? x + 1 : l;
                row[x] = guess(up[l], up[r], dn[l], dn[r], x, y);
            }
        }
        draw_line(y, row, 0xffff);
        poll_keys();
    }
}

/* Dive into one target, from the first frame down to MAX_STEP */
static void dive(int t)
{
    const target_t *tg = &s_targets[t];
    int step;

    set_palette(&s_palettes[tg->pal]);

    for (step = 0; step <= MAX_STEP && !s_quit; step++) {
        double w = ldexp(START_W, -step);
        unsigned long t0 = TT_HZ200;

        /* TT low's pixels are twice as wide as they are tall */
        s_dx = w / SCREEN_W;
        s_ndy = -s_dx / 2;
        s_re0 = tg->re - CENTRE_X * s_dx;
        s_im0 = tg->im - CENTRE_Y * s_ndy;
        s_maxiter = 100 + 10 * step;

        s_info_t = t;
        s_info_step = step;
        s_info_w = w;
        s_info_secs = -1;
        show_info();
        if (step == 0)
            render_full();
        else
            render_zoom();
        if (!s_quit) {
            s_info_secs = (long)((TT_HZ200 - t0) / 200);
            show_info();
        }
    }
}

/* ---------- main ---------- */
int main(void)
{
    static unsigned short pal[256];
    void (**slot)(void) = NULL;
    tt_state saved;
    int i, t;
    long ssp;

    if (!tt_check("MANDEL.TOS"))
        return 1;
    s_screen = tt_alloc_screen();
    if (!s_screen) {
        (void)Cconws("MANDEL.TOS: not enough ST-RAM for a screen.\r\n");
        return 1;
    }
    init_tables();
    font_init();

    ssp = Super(0L);
    tt_save(&saved);
    tt_set_cacr(TT_CACR_ON);
    pal[COL_BAR] = 0x223;
    pal[COL_TEXT] = 0xFFF;              /* entries 1-RAMP: the shine's */
    tt_start_mode(TT_MODE_TTLOW, s_screen, pal, 256);

    /* the shine runs in the VBL queue, beside TOS's own */
    {
        void (**queue)(void) = *(void (***)(void))0x456;
        int n = *(volatile short *)0x454;
        for (i = 0; i < n; i++)
            if (!queue[i]) {
                slot = &queue[i];
                *slot = shine_vbl;
                break;
            }
    }

    for (t = 0; !s_quit; t = (t + 1) % NUM_TARGETS)
        dive(t);

    if (slot)
        *slot = NULL;
    tt_restore(&saved);
    Super((void *)ssp);
    tt_free_screens();
    return 0;
}
