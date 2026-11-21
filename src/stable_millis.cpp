/*
 * SparkMiner - Glitch-free millis() for classic ESP32
 *
 * While one core hammers the SHA accelerator (DPORT bus), the other core's
 * APB reads can come back corrupted (ESP32 errata, DPORT/APB concurrent
 * access). millis() then returns the time plus 2^31 us for a single read,
 * which tripped the stratum inactivity check and skewed the hashrate.
 *
 * Linked in with -Wl,--wrap=millis: every millis() call in SparkMiner goes
 * through here and is accepted only when two consecutive reads agree.
 *
 * GPL v3 License
 */

#include <Arduino.h>

#if CONFIG_IDF_TARGET_ESP32

extern "C" unsigned long __real_millis();

extern "C" unsigned long IRAM_ATTR __wrap_millis() {
    unsigned long a = __real_millis();
    unsigned long b = __real_millis();
    while (b - a > 2) {
        a = b;
        b = __real_millis();
    }
    return b;
}

#endif
