#ifndef GRAY_DT_RANGES_H
#define GRAY_DT_RANGES_H

#include <stddef.h>
#include <stdint.h>

static inline uint64_t dt_big_endian_cells(const unsigned char *bytes, unsigned int cells)
{
    uint64_t value = 0;
    for (unsigned int i = 0; i < cells * 4; ++i) value = (value << 8) | bytes[i];
    return value;
}

static inline int dt_span_fits(uint64_t address, uint64_t span, unsigned int cells)
{
    if ((cells != 1 && cells != 2) || span == 0) return 0;
    uint64_t limit = cells == 1 ? UINT32_MAX : UINT64_MAX;
    return address <= limit && span - 1 <= limit - address;
}

/* Translate one complete register block. Empty ranges is identity;
 * missing ranges must be rejected by the caller, not passed as empty. */
static inline int dt_translate_ranges(const unsigned char *bytes, size_t length,
                                     unsigned int child_cells, unsigned int parent_cells,
                                     unsigned int size_cells, uint64_t address,
                                     uint64_t span, uint64_t *translated)
{
    if (!translated || !dt_span_fits(address, span, child_cells) ||
        (parent_cells != 1 && parent_cells != 2) ||
        (size_cells != 1 && size_cells != 2)) return -1;
    if (length == 0) {
        if (!dt_span_fits(address, span, parent_cells)) return -1;
        *translated = address;
        return 0;
    }
    size_t stride = (child_cells + parent_cells + size_cells) * 4;
    if (!bytes || length % stride != 0) return -1;
    unsigned int matches = 0;
    uint64_t result = 0;
    for (size_t offset = 0; offset < length; offset += stride) {
        const unsigned char *tuple = bytes + offset;
        uint64_t child = dt_big_endian_cells(tuple, child_cells);
        uint64_t parent = dt_big_endian_cells(tuple + child_cells * 4, parent_cells);
        uint64_t window = dt_big_endian_cells(tuple + (child_cells + parent_cells) * 4, size_cells);
        if (!dt_span_fits(child, window, child_cells) ||
            !dt_span_fits(parent, window, parent_cells)) return -1;
        /* Even a partially overlapping second window is ambiguous. */
        if (address <= child + window - 1 && child <= address + span - 1) {
            if (address < child || span > window || address - child > window - span)
                return -1;
            result = parent + (address - child);
            ++matches;
        }
    }
    if (matches != 1 || !dt_span_fits(result, span, parent_cells)) return -1;
    *translated = result;
    return 0;
}

#endif
