# host_tests — prove the brain on your LAPTOP, before any wheel turns.
#
# This compiles logic/maze_logic.c with your Mac's plain C compiler — no
# pico-sdk, no cross-compiler, no robot. It can do that ONLY because
# maze_logic.c is pure logic over plain structs. The robot build compiles
# the exact same file. Same code, two worlds; the tests make one of those
# worlds instant.
#
#   make        → build + run the unit tests AND the maze simulator
#                 (nonzero exit if anything fails)
#   make clean
#
# This suite was the starting gun, and it got green two different ways.
# The ORIGINAL checks were written against stubs: they compiled, ran, and
# FAILED first, and the brain was then built one function at a time until
# they turned green — tests first, code second, in that order in the git
# history. The last hardening pass (18 unit + 10 simulator checks) could
# not work that way, because the code was already finished and a fresh
# test that passes proves nothing about the test: each of those had to be
# shown going RED under a deliberate mutation of the brain before it
# counted. It stays green now: a red line here is a regression, and
# nothing merges over it.

CC     ?= cc
CFLAGS  = -Wall -Wextra -g -I../../include -I../../logic

# The reference classifier's threshold, lifted OUT OF THE SOURCE.
#
# maze_logic_ref.c keeps REF_DARK file-local on purpose — no header, no
# tuning.h knob, because "crude and readable in one sitting" is that
# file's whole job. The simulator's marginal sensor model has to know the
# number anyway (its guards prove worn tape still reads dark for BOTH
# classifiers), so sim_maze.c has always carried a hand-typed mirror of
# it. A hand copy is a promise that someone will remember; this line
# turns it into something the compiler checks. sed extracts the literal,
# -D hands it to the preprocessor, and an #if in sim_maze.c fails the
# BUILD if the mirror ever drifts from the source.
#
# (The audit's mutation probe moved REF_DARK 600 -> 700 and every test
# stayed green — the mirror was never compared to anything.)
#
# (The '\#' is not a typo: make would treat a bare '#' as the start of a
# comment even inside the quotes, and swallow the rest of the line.)
REF_DARK_FROM_SOURCE := $(shell sed -n 's/^\#define REF_DARK  *\([0-9][0-9]*\).*/\1/p' ../../logic/maze_logic_ref.c)

# Both binaries ALWAYS run: a red unit suite must not hide a red
# simulator (or vice versa), so the recipe remembers failure instead of
# stopping at the first nonzero exit — and still exits nonzero at the end.
run: test_maze_logic sim_maze
	rc=0; ./test_maze_logic || rc=1; ./sim_maze || rc=1; exit $$rc

test_maze_logic: test_maze_logic.c ../../logic/maze_logic.c ../../logic/maze_logic.h ../../include/tuning.h
	$(CC) $(CFLAGS) -o $@ test_maze_logic.c ../../logic/maze_logic.c

# The simulator links the reference classifier too: every run proves the
# capstone brain (the "student lane" by name only — see sim_maze.c's
# header) AND the I4 fallback lane (classify_junction_ref).
sim_maze: sim_maze.c ../../logic/maze_logic.c ../../logic/maze_logic_ref.c ../../logic/maze_logic.h ../../include/tuning.h
	@test -n "$(REF_DARK_FROM_SOURCE)" || { \
	  echo "host_tests: no '#define REF_DARK <number>' found in ../../logic/maze_logic_ref.c."; \
	  echo "  sim_maze.c mirrors that constant and this build verifies the mirror."; \
	  echo "  If the define was renamed or moved, fix the sed above AND the mirror."; \
	  exit 1; }
	$(CC) $(CFLAGS) -DREF_DARK_FROM_SOURCE=$(REF_DARK_FROM_SOURCE) -o $@ sim_maze.c ../../logic/maze_logic.c ../../logic/maze_logic_ref.c

clean:
	rm -f test_maze_logic sim_maze

.PHONY: run clean
