/*
 * Copyright (C) 2026 Neil Rackett
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * File: atari_tt.h
 * Description: Atari TT helpers shared by the demos: machine and FPU checks,
 *              the 68030 caches, and the TT shifter's modes, palette and
 *              screen base.
 */

#ifndef ATARI_TT_H
#define ATARI_TT_H

/* ---------- machine check (cookie jar: user or supervisor mode) ---------- */

/* 1 if _VDO reports the TT shifter, so the TT video modes exist */
int tt_has_video(void);

/* tt_has_video(), or explain on the console, wait for a key and return 0.
 * Call first thing in main(). There's no FPU check to go with it: built
 * -m68030 -m68881, the demos link MintLib's 68020-60 libraries, whose
 * start-up code won't run main() without a 68020+ and a 68881/68882. */
int tt_check(const char *name);

/* ---------- 68030 caches (supervisor mode) ---------- */

/* Instruction and data caches with burst fill and write allocate, the CACR
 * TOS 3.06 boots with; the clear bits (CD, CI) flush both as they go on. */
#define TT_CACR_ON 0x3919UL

unsigned long tt_get_cacr(void);
void tt_set_cacr(unsigned long cacr);

/* ---------- video (supervisor mode) ---------- */

#define TT_SHIFT   (*(volatile unsigned short *)0xFFFF8262UL)
#define TT_PALETTE ((volatile unsigned short *)0xFFFF8400UL)  /* 256 x 0RGB */

/* TT shift mode register values (palette bank 0) */
#define TT_MODE_TTMED 0x0400  /* 640x480, 16 colours: 4 interleaved planes  */
#define TT_MODE_TTLOW 0x0700  /* 320x480, 256 colours: 8 interleaved planes */

/* Both TT modes take 320 bytes a line and 153,600 a screen */
#define TT_LINE_BYTES   320
#define TT_SCREEN_LINES 480
#define TT_SCREEN_BYTES (TT_LINE_BYTES * TT_SCREEN_LINES)

/* What a demo changes, to hand back to TOS on exit */
typedef struct {
    unsigned short shift;
    unsigned short palette[256];
    unsigned long phys;
    unsigned long cacr;
} tt_state;

void tt_save(tt_state *s);
void tt_restore(const tt_state *s);

/* A screen in ST-RAM (the shifter can't see TT-RAM), 256-byte aligned and
 * cleared, or NULL. User or supervisor mode; tt_free_screens() gives every
 * one back. */
unsigned char *tt_alloc_screen(void);
void tt_free_screens(void);

/* The base address latches at the next VBL: set it, then tt_wait_vbl() */
void tt_set_screen(const void *addr);
void tt_wait_vbl(void);

/* Show a cleared screen and switch to a TT mode with the given palette:
 * 0RGB words, 4 bits per gun, n entries from 0 */
void tt_start_mode(unsigned short mode, const void *screen,
                   const unsigned short *palette, int n);

/* n palette entries evenly along key colours (4-bit r, g, b levels), the
 * first key to the last. The TT's guns have only 16 levels, so blue rounds
 * up half a level ahead of red and green: a ramp then steps twice as often,
 * and a grey one alternates neutral and cool, the least visible tint. */
void tt_ramp(unsigned short *out, int n, const unsigned char (*keys)[3], int nkeys);

/* Clear the rectangle x0..x1, y0..y1 (inclusive, whole 16-pixel groups) of
 * a TT med (4 planes) or TT low (8 planes) screen */
void tt_clear_box(unsigned char *scr, int planes, int x0, int y0, int x1, int y1);

/* 200 Hz system timer, _hz_200 (supervisor mode) */
#define TT_HZ200 (*(volatile unsigned long *)0x4BAL)

/* Milliseconds as up to five white digits (a 3x5 font, 6x10 on screen) at
 * text row 0, 1, 2... top left, in colour 15 (TT med) or 255 (TT low) */
void tt_draw_ms(unsigned char *scr, unsigned short mode, int row, unsigned long ms);

#endif /* ATARI_TT_H */
