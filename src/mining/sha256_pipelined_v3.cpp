/*
 * SparkMiner - Optimized Pipelined SHA-256 Assembly v3 for ESP32
 *
 * Same as v2 - register caching optimization was not achievable due to
 * Xtensa register pressure constraints. The SHA hardware latency is the
 * bottleneck, not CPU instructions.
 *
 * Note: ESP32 doesn't support midstate caching (no writable state registers)
 * so we must reload block 1 every iteration. This is a hardware limitation.
 *
 * DPORT errata: every read of the SHA registers (DPORT bus) is preceded by a
 * dummy read of an APB register (UART0 date, 0x3ff40078), the workaround
 * ESP-IDF's DPORT_REG_READ uses, and interrupts are masked for the length of
 * one hash so nothing lands between the two reads. Without it, back-to-back
 * DPORT reads on this core corrupt APB reads on the other one: millis() jumps
 * by 2^31 us and the WiFi MAC stalls until the task watchdog resets the chip.
 * Masking also makes each hash take a fixed number of cycles, which is what
 * the fixed SHA_WAITn delays below rely on.
 */

#include <Arduino.h>
#include "sha256_asm.h"

#if defined(CONFIG_IDF_TARGET_ESP32)

#include <soc/dport_reg.h>
#include <soc/hwcrypto_reg.h>
#include <hal/sha_ll.h>

#define SHA_START_OFFSET    0x90
#define SHA_CONTINUE_OFFSET 0x94
#define SHA_LOAD_OFFSET     0x98
#define SHA_BUSY_OFFSET     0x9C

// APB register read before every DPORT read (DPORT errata workaround)
#define DPORT_APB_PREREAD   0x3ff40078

// Interrupt level masked during one hash (same as ESP-IDF's DPORT access)
#define SHA_INTLEVEL        "5"

// Fixed schedule, in CPU cycles at 240 MHz. There are no busy polls: each
// wait is idle time sized so the engine is done before the next command.
// Measured on hardware with an exact host-side check of every hash; the
// value below each limit computes wrong hashes (WAIT1 >= 29, WAIT2 >= 27,
// WAIT3 >= 4, WAIT4 >= 38, GUARD2 >= 8, GUARD4 >= 2).
#ifndef SHA_WAIT1
    #define SHA_WAIT1   30  // block 1, after preparing block 2
#endif
#ifndef SHA_WAIT2
    #define SHA_WAIT2   28  // block 2, after the bookkeeping
#endif
#ifndef SHA_WAIT3
    #define SHA_WAIT3   4   // LOAD of the intermediate digest
#endif
#ifndef SHA_WAIT4
    #define SHA_WAIT4   38  // double hash, after writing next block 1 words 8-15
#endif
// Idle cycles after starting an operation before overwriting input words
// 8-15: the engine reads input word t in round t, not all at once at start.
#ifndef SHA_GUARD2
    #define SHA_GUARD2  8
#endif
#ifndef SHA_GUARD4
    #define SHA_GUARD4  4
#endif

#define STR_(x) #x
#define STR(x)  STR_(x)
#define NOPS(n) ".rept " STR(n) "\n nop.n\n .endr\n"

/**
 * Pipelined mining v3 - equivalent to v2.
 * Register caching was attempted but not achievable due to Xtensa constraints.
 * The SHA hardware is the bottleneck (~715 KH/s on CYD), not CPU.
 */
bool IRAM_ATTR sha256_pipelined_mine_v3(
    volatile uint32_t *sha_base,
    const uint32_t *header_swapped,
    uint32_t *nonce_ptr,
    volatile uint64_t *hash_count_ptr,
    volatile bool *mining_flag
) {
    const uint32_t shaPad = 0x80000000u;
    volatile uint32_t *apb = (volatile uint32_t *)DPORT_APB_PREREAD;

    // Ensure SHA peripheral is enabled
    DPORT_REG_SET_BIT(DPORT_PERI_CLK_EN_REG, DPORT_PERI_EN_SHA);
    DPORT_REG_CLR_BIT(DPORT_PERI_RST_EN_REG, DPORT_PERI_EN_SHA | DPORT_PERI_EN_SECUREBOOT);

    __asm__ __volatile__(

        // Setup
        "l32i.n   a2,  %[nonce], 0 \n"
        "addi     a5,  %[sb], 0x90 \n"
        "movi.n   a8,  0           \n"

        // Block 1 words 8-15 (inside the loop they are written while the
        // double hash runs, since the final LOAD only overwrites words 0-7)
        "l32i.n    a3,  %[IN], 32  \n"
        "s32i.n    a3,  %[sb], 32  \n"
        "l32i.n    a3,  %[IN], 36  \n"
        "s32i.n    a3,  %[sb], 36  \n"
        "l32i.n    a3,  %[IN], 40  \n"
        "s32i.n    a3,  %[sb], 40  \n"
        "l32i.n    a3,  %[IN], 44  \n"
        "s32i.n    a3,  %[sb], 44  \n"
        "l32i.n    a3,  %[IN], 48  \n"
        "s32i.n    a3,  %[sb], 48  \n"
        "l32i.n    a3,  %[IN], 52  \n"
        "s32i.n    a3,  %[sb], 52  \n"
        "l32i.n    a3,  %[IN], 56  \n"
        "s32i.n    a3,  %[sb], 56  \n"
        "l32i.n    a3,  %[IN], 60  \n"
        "s32i.n    a3,  %[sb], 60  \n"

        "rsil     a7,  " SHA_INTLEVEL " \n"

    "v3_start: \n"
        // ===== BLOCK 1: words 0-7 =====
        "l32i.n    a3,  %[IN],  0  \n"
        "s32i.n    a3,  %[sb],  0  \n"
        "l32i.n    a3,  %[IN],  4  \n"
        "s32i.n    a3,  %[sb],  4  \n"
        "l32i.n    a3,  %[IN],  8  \n"
        "s32i.n    a3,  %[sb],  8  \n"
        "l32i.n    a3,  %[IN], 12  \n"
        "s32i.n    a3,  %[sb], 12  \n"
        "l32i.n    a3,  %[IN], 16  \n"
        "s32i.n    a3,  %[sb], 16  \n"
        "l32i.n    a3,  %[IN], 20  \n"
        "s32i.n    a3,  %[sb], 20  \n"
        "l32i.n    a3,  %[IN], 24  \n"
        "s32i.n    a3,  %[sb], 24  \n"
        "l32i.n    a3,  %[IN], 28  \n"
        "s32i.n    a3,  %[sb], 28  \n"

        // ===== START SHA on block 1 =====
        "movi.n  a3, 1             \n"
        "s32i.n  a3, a5, 0         \n"
        "memw                      \n"

        // ===== PIPELINE: Prepare block 2 while SHA processes block 1 =====
        // (the engine reads input word t in round t, so writing from word 0
        // up stays behind it)
        "l32i    a3,  %[IN], 64    \n"
        "s32i.n  a3,  %[sb],  0    \n"
        "l32i    a3,  %[IN], 68    \n"
        "s32i.n  a3,  %[sb],  4    \n"
        "l32i    a3,  %[IN], 72    \n"
        "s32i.n  a3,  %[sb],  8    \n"

        // Store nonce
        "s32i.n  a2,  %[sb], 12    \n"

        // Store padding
        "s32i.n  %[pad2], %[sb], 16 \n"

        // ===== UNROLLED ZEROS =====
        "s32i.n  a8,  %[sb], 20    \n"
        "s32i.n  a8,  %[sb], 24    \n"
        "s32i.n  a8,  %[sb], 28    \n"
        "s32i.n  a8,  %[sb], 32    \n"
        "s32i.n  a8,  %[sb], 36    \n"
        "s32i.n  a8,  %[sb], 40    \n"
        "s32i.n  a8,  %[sb], 44    \n"
        "s32i.n  a8,  %[sb], 48    \n"
        "s32i.n  a8,  %[sb], 52    \n"
        "s32i.n  a8,  %[sb], 56    \n"

        // Store length (640 bits)
        "movi    a3,  0x280        \n"
        "s32i.n  a3,  %[sb], 60    \n"

        // ===== WAIT for block 1 (fixed delay, no poll) =====
        NOPS(SHA_WAIT1)

        // ===== CONTINUE SHA with block 2 =====
        "movi.n  a3, 1            \n"
        "s32i.n  a3, a5, 4        \n"
        "memw                     \n"

        // Pipeline: increment nonce
        "addi.n  a2, a2, 1        \n"

        // ===== UPDATE hash counter =====
        "l32i.n  a3, %[ih], 0     \n"
        "addi.n  a3, a3, 1        \n"
        "s32i.n  a3, %[ih], 0     \n"
        "bnez.n  a3, v3_no_carry  \n"
        "l32i.n  a4, %[ih], 4     \n"
        "addi.n  a4, a4, 1        \n"
        "s32i.n  a4, %[ih], 4     \n"
    "v3_no_carry: \n"

        // ===== DOUBLE-HASH padding (256 bits), once the engine read words 8+ =====
        // The LOAD below only overwrites words 0-7; words 9-14 are still zero
        NOPS(SHA_GUARD2)
        "s32i.n  %[pad2], %[sb], 32 \n"
        "movi    a4,  0x100       \n"
        "s32i.n  a4,  %[sb], 60   \n"

        // ===== CHECK mining flag, take pending interrupts =====
        "l8ui    a3, %[flag], 0   \n"
        "beqz.n  a3, v3_exit      \n"
        "wsr     a7, ps           \n"
        "rsync                    \n"
        "rsil    a7, " SHA_INTLEVEL " \n"

        // ===== WAIT for block 2 =====
        NOPS(SHA_WAIT2)

        // ===== LOAD intermediate result =====
        "movi.n  a4, 1            \n"
        "s32i.n  a4, a5, 8        \n"
        "memw                     \n"
        NOPS(SHA_WAIT3)

        // ===== START double hash =====
        "s32i.n  a4, a5, 0        \n"
        "memw                     \n"

        // ===== Next block 1, words 8-15, while the double hash runs =====
        NOPS(SHA_GUARD4)
        "l32i.n    a3,  %[IN], 32  \n"
        "s32i.n    a3,  %[sb], 32  \n"
        "l32i.n    a3,  %[IN], 36  \n"
        "s32i.n    a3,  %[sb], 36  \n"
        "l32i.n    a3,  %[IN], 40  \n"
        "s32i.n    a3,  %[sb], 40  \n"
        "l32i.n    a3,  %[IN], 44  \n"
        "s32i.n    a3,  %[sb], 44  \n"
        "l32i.n    a3,  %[IN], 48  \n"
        "s32i.n    a3,  %[sb], 48  \n"
        "l32i.n    a3,  %[IN], 52  \n"
        "s32i.n    a3,  %[sb], 52  \n"
        "l32i.n    a3,  %[IN], 56  \n"
        "s32i.n    a3,  %[sb], 56  \n"
        "l32i.n    a3,  %[IN], 60  \n"
        "s32i.n    a3,  %[sb], 60  \n"

        // ===== WAIT for double hash =====
        NOPS(SHA_WAIT4)

        // ===== LOAD final result =====
        "movi.n  a3, 1            \n"
        "s32i.n  a3, a5, 8        \n"
        "memw                     \n"

        // ===== EARLY REJECT (the APB pre-read also covers the LOAD) =====
        "l32i.n  a3, %[apb], 0    \n"
        "l16ui   a3, %[sb], 28    \n"
        "bnez.n  a3, v3_start     \n"
        "j       v3_end           \n"

        // Stopped mid-hash: let the engine finish before handing it back
    "v3_exit: \n"
        "l32i.n  a4, %[apb], 0    \n"
        "l32i.n  a4, a5, 12       \n"
        "bnez.n  a4, v3_exit      \n"

    "v3_end: \n"
        "wsr     a7, ps           \n"
        "rsync                    \n"
        "s32i.n  a2, %[nonce], 0  \n"

        :
        : [sb] "r"(sha_base),
          [IN] "r"(header_swapped),
          [ih] "r"(hash_count_ptr),
          [nonce] "r"(nonce_ptr),
          [flag] "r"(mining_flag),
          [pad2] "r"(shaPad),
          [apb] "r"(apb)
        : "a2", "a3", "a4", "a5", "a7", "a8", "memory"
    );

    return *mining_flag;
}

#endif // CONFIG_IDF_TARGET_ESP32
