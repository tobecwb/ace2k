/* Minimal host test harness for the core: named tests, ASSERT_* macros that report file:line and
 * keep going, a `-k substring` filter (a filter matching no test is itself a failure), an exit
 * code for make.
 *   TEST(name) { ASSERT_EQ(a, b); }   …
 *   int main(int argc, char **argv) { return TEST_MAIN(argc, argv); }
 * Depends on <stdio.h> and <string.h> only.  Tests register themselves through a constructor, so a
 * test file is just TESTs plus main. */
#ifndef ACE2K_TEST_H
#define ACE2K_TEST_H
// NOLINTBEGIN(readability-identifier-naming)
#include <stdio.h>
#include <string.h>

struct test_case {
    const char *name;
    void (*fn)(void);
    struct test_case *next;
};

static struct test_case *test_list_head;
static struct test_case **test_list_tail = &test_list_head;
static int test_asserts, test_failures, test_cases_run;

#define TEST(name)                                                                                 \
    static void test_##name(void);                                                                 \
    static struct test_case test_case_##name = { #name, test_##name, 0 };                          \
    __attribute__((constructor)) static void test_register_##name(void)                            \
    {                                                                                              \
        *test_list_tail = &test_case_##name;                                                       \
        test_list_tail = &test_case_##name.next;                                                   \
    }                                                                                              \
    static void test_##name(void)

#define ASSERT_TRUE(cond)                                                                          \
    do {                                                                                           \
        test_asserts++;                                                                            \
        if (!(cond)) {                                                                             \
            test_failures++;                                                                       \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                               \
        }                                                                                          \
    } while (0)

#define ASSERT_EQ(a, b)                                                                            \
    do {                                                                                           \
        long long test_a_ = (long long)(a);                                                        \
        long long test_b_ = (long long)(b);                                                        \
        test_asserts++;                                                                            \
        if (test_a_ != test_b_) {                                                                  \
            test_failures++;                                                                       \
            printf("  FAIL %s:%d: %s == %s  (%lld != %lld)\n", __FILE__, __LINE__, #a, #b,         \
                   test_a_, test_b_);                                                              \
        }                                                                                          \
    } while (0)

#define ASSERT_STR_EQ(a, b)                                                                        \
    do {                                                                                           \
        const char *test_a_ = (a);                                                                 \
        const char *test_b_ = (b);                                                                 \
        test_asserts++;                                                                            \
        if (strcmp(test_a_, test_b_) != 0) {                                                       \
            test_failures++;                                                                       \
            printf("  FAIL %s:%d: %s == %s  (\"%s\" != \"%s\")\n", __FILE__, __LINE__, #a, #b,     \
                   test_a_, test_b_);                                                              \
        }                                                                                          \
    } while (0)

static int test_main(int argc, char **argv)
{
    const char *filter = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-k") == 0 && i + 1 < argc) {
            filter = argv[++i];
        }
    }
    for (struct test_case *t = test_list_head; t; t = t->next) {
        if (filter && !strstr(t->name, filter)) {
            continue;
        }
        int before = test_failures;
        t->fn();
        test_cases_run++;
        if (test_failures != before) {
            printf("  in test %s\n", t->name);
        }
    }
    printf("%s: %d tests, %d assertions, %d failures\n", argv[0], test_cases_run, test_asserts,
           test_failures);
    if (filter && test_cases_run == 0) {
        printf("  no test matches -k %s\n", filter);
        return 1;
    }
    return test_failures ? 1 : 0;
}
#define TEST_MAIN(argc, argv) test_main(argc, argv)

// NOLINTEND(readability-identifier-naming)
#endif
