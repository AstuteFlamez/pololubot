# Host build for the maze-logic tests: no pico-sdk, no cross-compiler and
# no robot attached.
#
# This works because logic/maze_logic.c is pure logic over plain structs.
# It includes no SDK headers and touches no hardware, so an ordinary host
# `cc` compiles the very same file the firmware build cross-compiles —
# one source, two targets, and the host copy answers in milliseconds.
#
#   make        → build + run the unit tests AND the maze simulator
#                 (nonzero exit if anything fails)
#   make clean
#
# Both suites are held to the rule that a check must have been observed
# failing. The earliest checks were written against stubs and were red
# until maze_logic.c was implemented function by function. The later
# hardening checks were added to finished code, where a fresh test that
# passes proves nothing about the test, so each was verified by
# deliberately breaking the function it covers and confirming it went red.
# A red line here is a regression, not noise.

CC     ?= cc
CFLAGS  = -Wall -Wextra -g -I../../include -I../../logic

# The reference classifier's dark threshold, extracted from its source.
#
# logic/maze_logic_ref.c keeps REF_DARK file-local on purpose — no header,
# no tuning.h knob — because being readable end to end in one sitting is
# that file's whole job. The simulator's marginal sensor model needs the
# number anyway (its guards prove worn tape still reads dark for BOTH
# classifiers), so sim_maze.c carries a mirrored copy of it.
#
# A hand-copied constant is only correct while someone remembers it. This
# turns it into something the compiler checks: sed lifts the literal out
# of maze_logic_ref.c, -D hands it to the preprocessor, and an #if in
# sim_maze.c fails the BUILD if the mirror ever drifts from the source.
# Nothing else in the tree compares the two numbers — before this line
# existed, moving REF_DARK from 600 to 700 left every test green.
#
# (The '\#' is not a typo: make would treat a bare '#' as the start of a
# comment even inside the quotes, and swallow the rest of the line.)
REF_DARK_FROM_SOURCE := $(shell sed -n 's/^\#define REF_DARK  *\([0-9][0-9]*\).*/\1/p' ../../logic/maze_logic_ref.c)

# Both binaries ALWAYS run: a red unit suite must not hide a red
# simulator (or vice versa), so the recipe records failure instead of
# stopping at the first nonzero exit — and still exits nonzero at the end.
run: test_maze_logic sim_maze
	rc=0; ./test_maze_logic || rc=1; ./sim_maze || rc=1; exit $$rc

test_maze_logic: test_maze_logic.c ../../logic/maze_logic.c ../../logic/maze_logic.h ../../include/tuning.h
	$(CC) $(CFLAGS) -o $@ test_maze_logic.c ../../logic/maze_logic.c

# The simulator links the reference classifier as well, so every run
# exercises both classify_junction() and classify_junction_ref(). The
# firmware selects one of them with USE_MY_CLASSIFIER; proving both here
# keeps the unselected one from rotting unnoticed.
sim_maze: sim_maze.c ../../logic/maze_logic.c ../../logic/maze_logic_ref.c ../../logic/maze_logic.h ../../include/tuning.h
	@test -n "$(REF_DARK_FROM_SOURCE)" || { \
	  echo "tests/logic: no '#define REF_DARK <number>' found in ../../logic/maze_logic_ref.c."; \
	  echo "  sim_maze.c mirrors that constant and this build verifies the mirror."; \
	  echo "  If the define was renamed or moved, fix the sed above AND the mirror."; \
	  exit 1; }
	$(CC) $(CFLAGS) -DREF_DARK_FROM_SOURCE=$(REF_DARK_FROM_SOURCE) -o $@ sim_maze.c ../../logic/maze_logic.c ../../logic/maze_logic_ref.c

clean:
	rm -f test_maze_logic sim_maze

.PHONY: run clean
