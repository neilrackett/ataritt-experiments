/*
 * Copyright (C) 2026 Neil Rackett
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * File: atari_tt.c
 * Description: Atari TT helpers shared by the demos (see atari_tt.h).
 */

#include <string.h>
#include <mint/osbind.h>
#include <mint/cookie.h>

#include "atari_tt.h"

/* ---------- machine checks ---------- */

static long cookie(long id)
{
    long value = 0;
    if (C_FOUND != Getcookie(id, &value))
        return 0;
    return value;
}

/* _VDO high word: 0 ST, 1 STE, 2 TT, 3 Falcon */
int tt_has_video(void)
{
    return (cookie(C__VDO) >> 16) == 2;
}

int tt_check(const char *name)
{
    if (tt_has_video())
        return 1;
    (void)Cconws("\r\n");
    (void)Cconws(name);
    (void)Cconws(" needs an Atari TT: it runs in the TT's own video modes.\r\n");
    (void)Cconws("Press any key.\r\n");
    (void)Cnecin();
    return 0;
}

/* ---------- 68030 caches ---------- */

unsigned long tt_get_cacr(void)
{
    unsigned long cacr;
    __asm__ __volatile__("movec %%cacr,%0" : "=d"(cacr));
    return cacr;
}

void tt_set_cacr(unsigned long cacr)
{
    __asm__ __volatile__("movec %0,%%cacr" : : "d"(cacr) : "memory");
}

/* ---------- video ---------- */

#define VIDEO_BASE_HI  (*(volatile unsigned char *)0xFFFF8201UL)
#define VIDEO_BASE_MID (*(volatile unsigned char *)0xFFFF8203UL)
#define VIDEO_BASE_LO  (*(volatile unsigned char *)0xFFFF820DUL)
#define VBLCLOCK       (*(volatile unsigned long *)0x462L)

void tt_save(tt_state *s)
{
    int i;

    s->shift = TT_SHIFT;
    for (i = 0; i < 256; i++)
        s->palette[i] = TT_PALETTE[i];
    s->phys = (unsigned long)VIDEO_BASE_HI << 16 | (unsigned long)VIDEO_BASE_MID << 8
            | VIDEO_BASE_LO;
    s->cacr = tt_get_cacr();
}

void tt_restore(const tt_state *s)
{
    int i;

    /* the base address only latches at the VBL, the mode at once: black
     * out the frame in between, rather than show a screen in the wrong
     * mode, then bring the desktop's colours back */
    for (i = 0; i < 256; i++)
        TT_PALETTE[i] = 0;
    tt_set_screen((const void *)s->phys);
    TT_SHIFT = s->shift;
    tt_wait_vbl();
    tt_wait_vbl();
    for (i = 0; i < 256; i++)
        TT_PALETTE[i] = s->palette[i];
    tt_set_cacr(s->cacr | 0x0808UL);    /* flushing as it goes, like ours */
}

#define MAX_SCREENS 4
static void *s_blocks[MAX_SCREENS];

unsigned char *tt_alloc_screen(void)
{
    int i;
    long block;

    for (i = 0; i < MAX_SCREENS && s_blocks[i]; i++)
        ;
    if (i == MAX_SCREENS)
        return 0;
    block = (long)Mxalloc(TT_SCREEN_BYTES + 256, 0);   /* 0: ST-RAM only */
    if (block <= 0)
        return 0;
    s_blocks[i] = (void *)block;
    block = (block + 255) & ~255L;
    memset((void *)block, 0, TT_SCREEN_BYTES);
    return (unsigned char *)block;
}

void tt_free_screens(void)
{
    int i;

    for (i = 0; i < MAX_SCREENS; i++)
        if (s_blocks[i]) {
            (void)Mfree(s_blocks[i]);
            s_blocks[i] = 0;
        }
}

void tt_set_screen(const void *addr)
{
    unsigned long a = (unsigned long)addr;

    VIDEO_BASE_HI  = (unsigned char)(a >> 16);
    VIDEO_BASE_MID = (unsigned char)(a >> 8);
    VIDEO_BASE_LO  = (unsigned char)a;
}

void tt_wait_vbl(void)
{
    unsigned long t = VBLCLOCK;
    while (VBLCLOCK == t)
        ;
}

void tt_start_mode(unsigned short mode, const void *screen,
                   const unsigned short *palette, int n)
{
    int i;

    /* the new screen first, so the new mode never shows the desktop's
     * memory in the wrong format; then the palette, black where unused */
    tt_set_screen(screen);
    tt_wait_vbl();
    for (i = 0; i < 256; i++)
        TT_PALETTE[i] = i < n ? palette[i] : 0;
    TT_SHIFT = mode;
}

void tt_ramp(unsigned short *out, int n, const unsigned char (*keys)[3], int nkeys)
{
    static const int round_at[3] = { 64, 64, 192 };     /* r, g, b, of 256 */
    int i, c;

    for (i = 0; i < n; i++) {
        long pos = n > 1 ? (long)i * (nkeys - 1) * 256 / (n - 1) : 0;
        int k = (int)(pos >> 8), f = (int)(pos & 255);
        const unsigned char *a = keys[k], *b = keys[k + 1 < nkeys ? k + 1 : k];
        unsigned short rgb = 0;
        for (c = 0; c < 3; c++)
            rgb = (unsigned short)(rgb << 4
                                   | ((a[c] * (256 - f) + b[c] * f + round_at[c]) >> 8));
        out[i] = rgb;
    }
}

void tt_clear_box(unsigned char *scr, int planes, int x0, int y0, int x1, int y1)
{
    int bytes;
    unsigned char *row;

    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (y1 > TT_SCREEN_LINES - 1) y1 = TT_SCREEN_LINES - 1;
    x0 >>= 4;
    x1 >>= 4;
    if (x1 > TT_LINE_BYTES / 2 / planes - 1)
        x1 = TT_LINE_BYTES / 2 / planes - 1;
    if (x1 < x0 || y1 < y0)
        return;
    bytes = (x1 - x0 + 1) * planes * 2;
    row = scr + y0 * TT_LINE_BYTES + x0 * planes * 2;
    for (; y0 <= y1; y0++, row += TT_LINE_BYTES)
        memset(row, 0, bytes);
}

/* ---------- ms/frame readout ---------- */

/* 3x5 digits, one 3-bit row per byte */
static const unsigned char s_digits[10][5] = {
    { 7, 5, 5, 5, 7 }, { 2, 6, 2, 2, 7 }, { 7, 1, 7, 4, 7 }, { 7, 1, 7, 1, 7 },
    { 5, 5, 7, 1, 1 }, { 7, 4, 7, 1, 7 }, { 7, 4, 7, 5, 7 }, { 7, 1, 2, 2, 2 },
    { 7, 5, 7, 5, 7 }, { 7, 5, 7, 1, 7 },
};

/* One white pixel: all planes set */
static void put_pixel(unsigned char *scr, int planes, int x, int y)
{
    unsigned short mask = (unsigned short)(0x8000U >> (x & 15));
    unsigned short *w = (unsigned short *)(scr + y * TT_LINE_BYTES) + (x >> 4) * planes;
    int p;

    for (p = 0; p < planes; p++)
        w[p] |= mask;
}

void tt_draw_ms(unsigned char *scr, unsigned short mode, int row, unsigned long ms)
{
    /* each font pixel 2x2 in TT med; 1x2 in TT low, whose pixels are
     * twice as wide as they are tall */
    int planes = mode == TT_MODE_TTLOW ? 8 : 4;
    int sx = mode == TT_MODE_TTLOW ? 1 : 2;
    int y0 = 4 + row * 16;
    int col = 2 * sx + 5 * 4 * sx;     /* five digits, drawn low digit first */
    int digits = 0;

    if (ms > 99999UL)
        ms = 99999UL;
    do {
        const unsigned char *glyph = s_digits[ms % 10];
        int gx, gy, i, j;
        col -= 4 * sx;
        for (gy = 0; gy < 5; gy++)
            for (gx = 0; gx < 3; gx++)
                if (glyph[gy] & (4 >> gx))
                    for (j = 0; j < 2; j++)
                        for (i = 0; i < sx; i++)
                            put_pixel(scr, planes, col + gx * sx + i, y0 + gy * 2 + j);
        ms /= 10;
        digits++;
    } while (ms && digits < 5);
}
