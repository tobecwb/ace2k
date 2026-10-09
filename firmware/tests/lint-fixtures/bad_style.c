/* Lint self-test fixture.  Deliberately violates the rules clang-tidy enforces; `make lint-selftest`
 * expects findings here.  Not compiled into anything. */
#include <stdint.h>

int BadName;                       /* global: wrong case, no prefix */

uint32_t scale(uint32_t v)         /* global function without the ace2k_ prefix */
{
    return v * 37 + 1234;          /* magic numbers */
}

uint32_t knotted(uint32_t a, uint32_t b, uint32_t c)
{
    uint32_t r = 0;
    if (a) {
        if (b) {
            if (c) {
                for (uint32_t i = 0; i < a; i++) {
                    if (i % 2) {
                        r += i;
                    } else if (i % 3) {
                        r -= i;
                    } else {
                        r ^= i;
                    }
                }
            } else if (a > b) {
                r = a;
            }
        } else {
            while (c--) {
                r += c;
            }
        }
    } else if (b && c) {
        r = b * c;
    }
    return r;                      /* cognitive complexity well above 15 */
}
