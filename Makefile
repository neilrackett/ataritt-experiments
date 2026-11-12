# ataritt-experiments - the atarist-sfp004 demos, rebuilt for the Atari TT
# by Neil Rackett
#
# Sources are in src/, the built programs go in dist/.
#
#   make            build all the demos
#   make lores      dist/TORUS.TOS     raymarcher, 80x60 chunky blocks
#   make hires      dist/TORUSHI.TOS   raymarcher, 160x120 chunky blocks
#   make mesh       dist/KNOTMESH.TOS  polygon mesh, 640x480
#   make cube       dist/CUBEMESH.TOS  classic spinning cube (benchmark)
#   make jet        dist/JETMESH.TOS   F-22 model from 3DOS, 640x480
#   make mandel     dist/MANDEL.TOS    endless shining Mandelbrot zoom
#   make clean      remove dist/
#
# The polygon demos run in TT medium res (640x480, 16 colours); build them
# with `make TTLOW=1` for TT low res instead (320x480, 256 colours, finer
# shading for coarser edges). `make clean` first: the binaries don't
# remember which they were.
#
# Build with the m68k-atari-mint cross toolchain on your PATH, or with
# atarist-toolkit-docker: `STCMD_NO_TTY=1 stcmd make`. The newer ELF
# toolchain works too: `make CC=m68k-atari-mintelf-gcc`.

SRC  := src
DIST := dist

CC    := m68k-atari-mint-gcc
FLAGS := $(CC:-gcc=-flags)

# The TT's 68882 sits on the 68030's coprocessor bus, so unlike the ST's
# memory-mapped SFP-004 it runs real FPU instructions: build for the 68030
# with the 68881 instruction set (which links the 68020-60 multilib) and let
# the compiler issue them. -fno-math-errno lets sqrtf() be one FSQRT.
CFLAGS := -m68030 -m68881 -O2 -fomit-frame-pointer -fno-math-errno -Wall

# Load into TT-RAM and Malloc from it where a TT has some: the 68030 runs
# 32-bit from it with no video contention. The screens are Mxalloc'ed from
# ST-RAM, the only RAM the shifter can see.
PRGFLAGS := --mfastram --mfastalloc

DEPS := $(SRC)/atari_tt.c $(SRC)/atari_tt.h

TTLOW ?= 0
POLY  := $(SRC)/poly.h $(SRC)/sintab.h $(DEPS)

# Compile and link a program from the .c files among its prerequisites,
# then mark it for TT-RAM.
define build
	$(CC) $(CFLAGS) $(1) -o $@ $(filter %.c,$^) $(2)
	$(FLAGS) -s $(PRGFLAGS) $@
endef

.PHONY: all lores hires mesh cube jet mandel clean

all: lores hires mesh cube jet mandel

lores: $(DIST)/TORUS.TOS

hires: $(DIST)/TORUSHI.TOS

mesh: $(DIST)/KNOTMESH.TOS

cube: $(DIST)/CUBEMESH.TOS

jet: $(DIST)/JETMESH.TOS

mandel: $(DIST)/MANDEL.TOS

$(DIST):
	mkdir -p $@

## Real-time SDF raymarcher in TT low res (256 colours): every pixel is
## sphere-traced signed-distance work on the 68882. Slow and glorious.
$(DIST)/TORUS.TOS: $(SRC)/torus.c $(SRC)/sintab.h $(DEPS) | $(DIST)
	$(call build)

$(DIST)/TORUSHI.TOS: $(SRC)/torus.c $(SRC)/sintab.h $(DEPS) | $(DIST)
	$(call build,-DHIRES=1)

## Faceted mesh in TT medium res: the 68882 does vertex transform and
## projection, the 68030 integer cull/sort/span-fill. Built for frame rate.
$(DIST)/KNOTMESH.TOS: $(SRC)/knotmesh.c $(POLY) | $(DIST)
	$(call build,-O3 -DTTLOW=$(TTLOW))

## The traditional spinning cube: minimum geometry, same pipeline.
$(DIST)/CUBEMESH.TOS: $(SRC)/cubemesh.c $(POLY) | $(DIST)
	$(call build,-O3 -DTTLOW=$(TTLOW))

## The low-poly F-22 from 3DOS (model data in f22.h): the mesh pipeline
## with four materials and a 200-triangle painter's sort.
$(DIST)/JETMESH.TOS: $(SRC)/jetmesh.c $(SRC)/f22.h $(POLY) | $(DIST)
	$(call build,-O3 -DTTLOW=$(TTLOW))

## Endless Mandelbrot zoom in TT low res: the 68882 iterates in its 80-bit
## registers and branches on its own compare; a VBL shines 208 colours.
$(DIST)/MANDEL.TOS: $(SRC)/mandel.c $(DEPS) | $(DIST)
	$(call build,-O3,-lm)

clean:
	rm -rf $(DIST)
