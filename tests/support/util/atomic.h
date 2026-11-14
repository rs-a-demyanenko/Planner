#pragma once
// Host-only stand-in: tests run in one thread with no hardware ISR.
#define ATOMIC_RESTORESTATE 0
#define ATOMIC_BLOCK(mode) for (bool mockAtomicOnce = true; mockAtomicOnce; mockAtomicOnce = false)
