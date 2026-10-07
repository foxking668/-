#include "../tools/gray_dt_ranges.h"
#include <stdio.h>

static unsigned int checks;
static unsigned int failures;
static void check(int condition, const char *name)
{
    ++checks;
    if (!condition) { ++failures; fprintf(stderr, "FAIL %s\n", name); }
}

int main(void)
{
    uint64_t result = 0;
    /* child 0x1000 -> parent 0x20000000, length 0x1000 (1/1/1 cells). */
    const unsigned char mapping[] = {
        0, 0, 0x10, 0, 0x20, 0, 0, 0, 0, 0, 0x10, 0
    };
    /* Same nonempty identity window used by common SoC buses. */
    const unsigned char identity[] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0x20, 0, 0, 0
    };
    const unsigned char mixed_cells[] = {
        0, 0, 0x10, 0, 0, 0, 0, 1, 0x20, 0, 0, 0, 0, 0, 0x10, 0
    };
    const unsigned char overlapping[] = {
        0, 0, 0x10, 0, 0x20, 0, 0, 0, 0, 0, 0x10, 0,
        0, 0, 0x10, 0, 0x30, 0, 0, 0, 0, 0, 0x10, 0
    };
    const unsigned char partial_overlap[] = {
        0, 0, 0x10, 0, 0x20, 0, 0, 0, 0, 0, 0x10, 0,
        0, 0, 0x11, 0x80, 0x30, 0, 0, 0, 0, 0, 0, 0x40
    };
    const unsigned char overflowing_window[] = {
        0, 0, 0x10, 0, 0xff, 0xff, 0xff, 0xf0, 0, 0, 0, 0x20
    };
    const unsigned char empty_window[] = {
        0, 0, 0x10, 0, 0x20, 0, 0, 0, 0, 0, 0, 0
    };
    check(dt_translate_ranges(NULL, 0, 1, 2, 1, 0x16104000, 0x1000, &result) == 0 && result == 0x16104000,
          "empty identity preserves address");
    check(dt_translate_ranges(identity, sizeof(identity), 1, 1, 1, 0x16104000, 0x1000, &result) == 0 && result == 0x16104000,
          "nonempty identity accepted");
    check(dt_translate_ranges(mapping, sizeof(mapping), 1, 1, 1, 0x1100, 0x100, &result) == 0 && result == 0x20000100,
          "translated window uses address offset");
    check(dt_translate_ranges(mixed_cells, sizeof(mixed_cells), 1, 2, 1, 0x1100, 0x100, &result) == 0 && result == UINT64_C(0x120000100),
          "32-bit child to 64-bit parent preserves high cell");
    check(dt_translate_ranges(mapping, sizeof(mapping), 1, 1, 1, 0x1f00, 0x100, &result) == 0 && result == 0x20000f00,
          "block may end exactly at window boundary");
    check(dt_translate_ranges(mapping, sizeof(mapping), 1, 1, 1, 0x1f00, 0x101, &result) != 0,
          "reject block crossing window boundary");
    check(dt_translate_ranges(mapping, sizeof(mapping), 1, 1, 1, 0x3000, 0x10, &result) != 0,
          "reject unmatched address");
    check(dt_translate_ranges(mapping, sizeof(mapping) - 1, 1, 1, 1, 0x1100, 0x100, &result) != 0,
          "reject truncated tuple");
    check(dt_translate_ranges(overlapping, sizeof(overlapping), 1, 1, 1, 0x1100, 0x100, &result) != 0,
          "reject multiple matching mappings");
    check(dt_translate_ranges(partial_overlap, sizeof(partial_overlap), 1, 1, 1, 0x1100, 0x100, &result) != 0,
          "reject partially overlapping second mapping");
    check(dt_translate_ranges(overflowing_window, sizeof(overflowing_window), 1, 1, 1, 0x1000, 1, &result) != 0,
          "reject parent address overflow");
    check(dt_translate_ranges(empty_window, sizeof(empty_window), 1, 1, 1, 0x1000, 1, &result) != 0,
          "reject zero-sized window");
    check(dt_translate_ranges(NULL, 0, 2, 1, 1, UINT64_C(0x100000000), 1, &result) != 0,
          "reject narrowing address overflow");
    check(dt_translate_ranges(NULL, 0, 2, 2, 2, UINT64_MAX, 2, &result) != 0,
          "reject 64-bit span overflow");
    check(dt_translate_ranges(NULL, 0, 1, 1, 1, 0, 0, &result) != 0,
          "reject zero register size");
    check(dt_translate_ranges(NULL, 0, 3, 1, 1, 0x1000, 1, &result) != 0,
          "reject unsupported address encoding");
    check(dt_translate_ranges(NULL, sizeof(mapping), 1, 1, 1, 0x1100, 0x100, &result) != 0,
          "reject missing bytes for nonempty property");
    check(dt_translate_ranges(NULL, 0, 1, 1, 1, 0x1000, 1, NULL) != 0,
          "reject null result pointer");
    printf("gray_dt_ranges_tests checks=%u failures=%u\n", checks, failures);
    return failures ? 1 : 0;
}
