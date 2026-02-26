#ifndef KAFEICHE_DRIVERS_PIGPIO_UTILS_HPP
#define KAFEICHE_DRIVERS_PIGPIO_UTILS_HPP

#include <stdexcept>
#include <pigpiod_if2.h>

/**
 * @brief Helper to open a pigpiod connection once and cache the handle.
 *
 * Inline so that it can be included in multiple translation units without
 * violating the one-definition-rule.  All callers should include this header
 * instead of defining their own copy of the function to avoid redefinition
 * errors when two headers are pulled into the same translation unit.
 *
 * @throws std::runtime_error if the daemon cannot be contacted.
 */
inline int getPiHandle()
{
    static int pi = -1;
    if (pi < 0) {
        pi = pigpio_start(NULL, NULL);
        if (pi < 0) {
            throw std::runtime_error(
                "pigpio daemon connection failed (is pigpiod running?).\n"
                "Start it with 'sudo systemctl start pigpiod' or enable at boot."
            );
        }
    }
    return pi;
}

#endif // KAFEICHE_DRIVERS_PIGPIO_UTILS_HPP
