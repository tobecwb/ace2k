/* What: the one way this board code registers an interrupt handler with Klipper's
 * armcm_enable_irq().
 * How: ACE2K_ARMCM_ENABLE_IRQ(func, num, prio) is armcm_enable_irq(func, num, prio) inside a
 * diagnostic push / pop that silences -Wunterminated-string-initialization for that one
 * statement: Klipper's DECL_CTR_INT, behind armcm_enable_irq, sizes its request string without
 * the terminating NUL on purpose (the values follow the text), and GCC 15 warns about that under
 * -Wextra -Werror.  -Wpragmas is silenced first, for compilers that do not know the option.
 * Depends on: nothing here — the including file brings board/armcm_boot.h itself (this header
 * is checked standalone, with the standard headers only). */
#ifndef ACE2K_BOARD_ARMCM_IRQ_H
#define ACE2K_BOARD_ARMCM_IRQ_H

/* one pragma per line, as the push / ignore / call / pop it is */
/* clang-format off */
#define ACE2K_ARMCM_ENABLE_IRQ(func, num, prio)                                  \
    _Pragma("GCC diagnostic push")                                               \
    _Pragma("GCC diagnostic ignored \"-Wpragmas\"")                              \
    _Pragma("GCC diagnostic ignored \"-Wunterminated-string-initialization\"")   \
    armcm_enable_irq(func, num, prio);                                           \
    _Pragma("GCC diagnostic pop")
/* clang-format on */

#endif
