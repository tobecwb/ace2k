#include "test.h"
#include "core/report.h"

TEST(a_zero_period_is_never_due)
{
    struct ace2k_report r;
    ace2k_report_set(&r, 0, 100, 0);
    ASSERT_TRUE(!ace2k_report_due(&r, 100));
    ASSERT_TRUE(!ace2k_report_due(&r, 100000));
}

TEST(due_once_at_the_deadline_then_one_period_later)
{
    struct ace2k_report r;
    ace2k_report_set(&r, 1000, 100, 0);
    ASSERT_TRUE(!ace2k_report_due(&r, 1090));
    ASSERT_TRUE(ace2k_report_due(&r, 1100));
    ASSERT_TRUE(!ace2k_report_due(&r, 1100)); /* once */
    ASSERT_TRUE(!ace2k_report_due(&r, 2090));
    ASSERT_TRUE(ace2k_report_due(&r, 2100));
}

TEST(after_a_stall_the_next_deadline_stays_on_the_grid)
{
    struct ace2k_report r;
    ace2k_report_set(&r, 1000, 0, 0);
    ASSERT_TRUE(ace2k_report_due(&r, 3500)); /* 2.5 periods late: one report, not three */
    ASSERT_EQ(r.next_ms, 4000);              /* the grid, not now + rest */
    ASSERT_TRUE(!ace2k_report_due(&r, 3510));
    ASSERT_TRUE(!ace2k_report_due(&r, 3990));
    ASSERT_TRUE(ace2k_report_due(&r, 4000));
    ASSERT_EQ(r.next_ms, 5000);
    /* a stall that ends exactly on a grid point: one report, the next a full period on */
    ASSERT_TRUE(ace2k_report_due(&r, 7000));
    ASSERT_EQ(r.next_ms, 8000);
    ASSERT_TRUE(!ace2k_report_due(&r, 7990));
    ASSERT_TRUE(ace2k_report_due(&r, 8000));
}

TEST(two_phased_reports_keep_their_phase_across_a_stall)
{
    struct ace2k_report a;
    struct ace2k_report b;
    ace2k_report_set(&a, 1000, 0, 0);
    ace2k_report_set(&b, 1000, 0, 50);
    /* a tick at 2500 catches both up: a's 1000 and 2000, b's 1050 and 2050 were swallowed */
    ASSERT_TRUE(ace2k_report_due(&a, 2500));
    ASSERT_TRUE(ace2k_report_due(&b, 2500));
    ASSERT_EQ(a.next_ms, 3000);
    ASSERT_EQ(b.next_ms, 3050);
    ASSERT_TRUE(ace2k_report_due(&a, 3000));
    ASSERT_TRUE(!ace2k_report_due(&b, 3000)); /* not on the same tick */
    ASSERT_TRUE(ace2k_report_due(&b, 3050));
}

TEST(the_phase_delays_the_first_report_only)
{
    struct ace2k_report r;
    ace2k_report_set(&r, 1000, 100, 70);
    ASSERT_TRUE(!ace2k_report_due(&r, 1100));
    ASSERT_TRUE(!ace2k_report_due(&r, 1160));
    ASSERT_TRUE(ace2k_report_due(&r, 1170));
    ASSERT_EQ(r.next_ms, 2170); /* the period, from the phased instant */
}

TEST(due_survives_the_millisecond_wrap)
{
    struct ace2k_report r;
    /* 0xFFFFFF00 is 40 past its grid cell's start: the deadline is the cell + 1000, which wraps
     * to 0x2C0 (the counter's range is not a multiple of 100, so the grid shifts at the wrap) */
    ace2k_report_set(&r, 1000, 0xFFFFFF00U, 0);
    ASSERT_EQ(r.next_ms, 0x2C0U);
    ASSERT_TRUE(!ace2k_report_due(&r, 0xFFFFFFF0U));
    ASSERT_TRUE(!ace2k_report_due(&r, 0x2BFU));
    ASSERT_TRUE(ace2k_report_due(&r, 0x2C0U));
    ASSERT_EQ(r.next_ms, 0x2C0U + 1000U);
}

TEST(queries_from_different_ticks_of_one_grid_cell_land_on_their_phases)
{
    /* the host's queries arrive in one block, over a few ticks: anchored to the tick each
     * arrived on, a phase-50 report set at 120 and a phase-0 one set at 170 would both be due
     * at 1170 — the phases exist to keep them apart, so the anchor is the grid cell */
    struct ace2k_report a;
    struct ace2k_report b;
    ace2k_report_set(&a, 1000, 120, 50);
    ace2k_report_set(&b, 1000, 170, 0);
    ASSERT_EQ(a.next_ms, 1150);
    ASSERT_EQ(b.next_ms, 1100);
    ASSERT_TRUE(!ace2k_report_due(&a, 1100));
    ASSERT_TRUE(ace2k_report_due(&b, 1100));
    ASSERT_TRUE(ace2k_report_due(&a, 1150));
    ASSERT_TRUE(!ace2k_report_due(&b, 1150));
    /* the grid, not the query's tick, for every later period too */
    ASSERT_EQ(a.next_ms, 2150);
    ASSERT_EQ(b.next_ms, 2100);
    /* a period shorter than a grid cell may set a deadline in the past; the first due call
     * advances it whole periods ahead, onto the grid */
    struct ace2k_report c;
    ace2k_report_set(&c, 50, 190, 0);
    ASSERT_EQ(c.next_ms, 150);
    ASSERT_TRUE(ace2k_report_due(&c, 190));
    ASSERT_EQ(c.next_ms, 200);
}

/* A fake send: refuses or accepts as told, counts its calls, and records the flag as it stood
 * during the send; when told to, raises the flag from inside the send — the tick landing during
 * the try. */
static volatile bool ace2k_fake_due;
static bool ace2k_fake_send_ok;
static bool ace2k_fake_send_raises;
static int ace2k_fake_send_calls;
static bool ace2k_fake_due_during_send;

static bool fake_send(void)
{
    ace2k_fake_send_calls++;
    ace2k_fake_due_during_send = ace2k_fake_due;
    if (ace2k_fake_send_raises) {
        ace2k_fake_due = true;
    }
    return ace2k_fake_send_ok;
}

TEST(a_try_clears_the_flag_before_the_send_and_holds_the_report_when_the_send_refuses)
{
    ace2k_fake_due = true;
    ace2k_fake_send_ok = false;
    ace2k_fake_send_raises = false;
    ace2k_fake_send_calls = 0;
    /* refused: one send, the flag was down during it, and it is up again — held for the next
     * tick */
    ASSERT_TRUE(!ace2k_report_try(&ace2k_fake_due, fake_send));
    ASSERT_EQ(ace2k_fake_send_calls, 1);
    ASSERT_TRUE(!ace2k_fake_due_during_send);
    ASSERT_TRUE(ace2k_fake_due);
    /* the next tick's pass: the send succeeds, the flag stays down */
    ace2k_fake_send_ok = true;
    ASSERT_TRUE(ace2k_report_try(&ace2k_fake_due, fake_send));
    ASSERT_EQ(ace2k_fake_send_calls, 2);
    ASSERT_TRUE(!ace2k_fake_due_during_send);
    ASSERT_TRUE(!ace2k_fake_due);
    /* the tick raises the flag during a successful send: that deadline survives the try (a clear
     * after the send would have wiped it) */
    ace2k_fake_due = true;
    ace2k_fake_send_raises = true;
    ASSERT_TRUE(ace2k_report_try(&ace2k_fake_due, fake_send));
    ASSERT_EQ(ace2k_fake_send_calls, 3);
    ASSERT_TRUE(ace2k_fake_due);
    /* and during a refused one: still up, one report owed, not two */
    ace2k_fake_send_ok = false;
    ASSERT_TRUE(!ace2k_report_try(&ace2k_fake_due, fake_send));
    ASSERT_TRUE(ace2k_fake_due);
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
