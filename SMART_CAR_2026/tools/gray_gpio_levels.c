#define _XOPEN_SOURCE 700
/* LS2K0300 diagnostic only. /dev/mem is opened O_RDONLY and mapped PROT_READ.
 * Read GPIO_IN byte aliases and GPIO_OEN; never request GPIOs or map mux MMIO.
 * References and deployment procedure: deploy/GRAY_GPIO_LEVELS.md. */
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/gpio.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/utsname.h>
#include <time.h>
#include <unistd.h>
#include "gray_dt_ranges.h"

#define PIN_COUNT 106
#define GPIO_BASE UINT64_C(0x16104000)
#define MUX_BASE UINT64_C(0x16000490)
#define NODE "/sys/bus/platform/devices/16000490.pinctrl/of_node"
#define DT_ROOT "/sys/firmware/devicetree/base"
#define PINMUX_OWNERS "/sys/kernel/debug/pinctrl/16000490.pinctrl-loongson pinctrl/pinmux-pins"
#define VERSION "2026-10-05.3"

struct readonly_region {
    void *mapping;
    size_t length;
    volatile const uint8_t *bytes;
};
static volatile sig_atomic_t interrupted;
static void stop_capture(int signal_number) { (void)signal_number; interrupted = 1; }

static int read_property(const char *path, unsigned char *bytes, size_t capacity)
{
    FILE *file = fopen(path, "rb");
    if (!file) { fprintf(stderr, "Read %s: %s\n", path, strerror(errno)); return -1; }
    size_t size = fread(bytes, 1, capacity, file);
    int extra = size == capacity ? fgetc(file) : EOF;
    int failed = ferror(file) || extra != EOF;
    if (fclose(file) != 0) failed = 1;
    if (failed) { fprintf(stderr, "Invalid/oversized property: %s\n", path); return -1; }
    return (int)size;
}

static int node_property(const char *node, const char *name, unsigned char *bytes, size_t capacity)
{
    char path[PATH_MAX];
    int length = snprintf(path, sizeof(path), "%s/%s", node, name);
    if (length < 0 || (size_t)length >= sizeof(path)) return -1;
    return read_property(path, bytes, capacity);
}

static int node_cells(const char *node, const char *name, unsigned int *cells)
{
    unsigned char bytes[4];
    int length = node_property(node, name, bytes, sizeof(bytes));
    if (length != 4) {
        fprintf(stderr, "Invalid/missing %s/%s: bytes=%d.\n", node, name, length);
        return -1;
    }
    *cells = (unsigned int)dt_big_endian_cells(bytes, 1);
    if (*cells != 1 && *cells != 2) {
        fprintf(stderr, "Unsupported %s/%s=%u; requires 1 or 2 cells.\n", node, name, *cells);
        return -1;
    }
    return 0;
}

static int parent_node(char *path, const char *root)
{
    size_t root_length = strlen(root);
    if (strncmp(path, root, root_length) != 0 || path[root_length] != '/') return -1;
    char *slash = strrchr(path, '/');
    if (!slash || slash < path + root_length) return -1;
    *slash = '\0';
    return 0;
}

static int translate_to_cpu(const char *bus, const char *root, uint64_t address,
                            uint64_t span, uint64_t *physical)
{
    char current[PATH_MAX];
    int copied = snprintf(current, sizeof(current), "%s", bus);
    if (copied < 0 || (size_t)copied >= sizeof(current)) return -1;
    for (unsigned int depth = 0; depth < 16; ++depth) {
        if (!strcmp(current, root)) { *physical = address; return 0; }
        char parent[PATH_MAX];
        memcpy(parent, current, strlen(current) + 1);
        if (parent_node(parent, root) != 0) return -1;
        unsigned int child_cells, parent_cells, size_cells;
        if (node_cells(current, "#address-cells", &child_cells) != 0 ||
            node_cells(current, "#size-cells", &size_cells) != 0 ||
            node_cells(parent, "#address-cells", &parent_cells) != 0) return -1;
        unsigned char ranges[4096];
        int length = node_property(current, "ranges", ranges, sizeof(ranges));
        if (length < 0) return -1;
        printf("DT_RANGES bus=%s bytes=%d child_cells=%u parent_cells=%u size_cells=%u\n",
               current, length, child_cells, parent_cells, size_cells);
        uint64_t next;
        if (dt_translate_ranges(ranges, (size_t)length, child_cells, parent_cells,
                                size_cells, address, span, &next) != 0) {
            fprintf(stderr, "Invalid/unmatched/ambiguous ranges at %s: address=0x%" PRIx64
                    " span=0x%" PRIx64 "; no register access.\n", current, address, span);
            fprintf(stderr, "DT_RANGES_HEX=");
            for (int i = 0; i < length; ++i) fprintf(stderr, "%02x", ranges[i]);
            fputc('\n', stderr);
            return -1;
        }
        printf("DT_TRANSLATE 0x%" PRIx64 " -> 0x%" PRIx64 " span=0x%" PRIx64 "\n", address, next, span);
        address = next;
        memcpy(current, parent, strlen(parent) + 1);
    }
    fprintf(stderr, "Device-tree nesting exceeds diagnostic limit.\n");
    return -1;
}

static int validate_board(void)
{
    struct utsname host;
    if (uname(&host) != 0 || strcmp(host.machine, "loongarch64") != 0) {
        fprintf(stderr, "Refusing register access: requires the LoongArch target board.\n");
        return -1;
    }
    unsigned char bytes[256];
    int size = read_property(NODE "/compatible", bytes, sizeof(bytes));
    int matched = 0;
    for (int offset = 0; size > 0 && offset < size;) {
        size_t length = strnlen((const char *)bytes + offset, (size_t)(size - offset));
        if (length == (size_t)(size - offset)) break;
        if (!strcmp((const char *)bytes + offset, "loongson,ls2k300-pinctrl")) matched = 1;
        offset += (int)length + 1;
    }
    if (!matched) { fprintf(stderr, "Unsupported pinctrl compatible; no register access.\n"); return -1; }
    char root[PATH_MAX], bus[PATH_MAX];
    if (!realpath(DT_ROOT, root) || !realpath(NODE, bus)) {
        fprintf(stderr, "Resolve device-tree path: %s\n", strerror(errno)); return -1;
    }
    if (parent_node(bus, root) != 0) { fprintf(stderr, "Pinctrl node is outside the device tree.\n"); return -1; }
    unsigned int address_cells, size_cells;
    if (node_cells(bus, "#address-cells", &address_cells) != 0 ||
        node_cells(bus, "#size-cells", &size_cells) != 0) return -1;
    size = read_property(NODE "/reg", bytes, sizeof(bytes));
    unsigned int stride = (address_cells + size_cells) * 4;
    if (size < (int)(2 * stride) || size % (int)stride != 0) {
        fprintf(stderr, "Invalid pinctrl reg property: bytes=%d stride=%u.\n", size, stride); return -1;
    }
    uint64_t mux_address = dt_big_endian_cells(bytes, address_cells);
    uint64_t mux_size = dt_big_endian_cells(bytes + address_cells * 4, size_cells);
    uint64_t gpio_address = dt_big_endian_cells(bytes + stride, address_cells);
    uint64_t gpio_size = dt_big_endian_cells(bytes + stride + address_cells * 4, size_cells);
    if (!dt_span_fits(mux_address, mux_size, address_cells) ||
        !dt_span_fits(gpio_address, gpio_size, address_cells)) {
        fprintf(stderr, "Invalid/overflowing pinctrl register block.\n"); return -1;
    }
    if (translate_to_cpu(bus, root, mux_address, mux_size, &mux_address) != 0 ||
        translate_to_cpu(bus, root, gpio_address, gpio_size, &gpio_address) != 0) return -1;
    /* Mux is identification metadata only. Do not map or infer its layout. */
    if (mux_address != MUX_BASE || mux_size < 4 || gpio_address != GPIO_BASE || gpio_size < 0xA00 + PIN_COUNT) {
        fprintf(stderr, "Unexpected register resources: mux=0x%" PRIx64 " size=0x%" PRIx64
                " gpio=0x%" PRIx64 " size=0x%" PRIx64 "; no register access.\n",
                mux_address, mux_size, gpio_address, gpio_size);
        return -1;
    }
    for (unsigned int bank = 0; bank < 7; ++bank) {
        char path[64], expected[32];
        snprintf(path, sizeof(path), "/dev/gpiochip%u", bank);
        snprintf(expected, sizeof(expected), "GPA%u", bank * 16);
        int fd = open(path, O_RDONLY | O_CLOEXEC);
        struct gpiochip_info chip = {0};
        int good = fd >= 0 && ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &chip) == 0;
        if (fd >= 0 && close(fd) != 0) good = 0;
        if (!good || chip.lines != 16 || strncmp(chip.label, expected, sizeof(chip.label)) != 0) {
            fprintf(stderr, "Unexpected GPIO bank %u; no register access.\n", bank); return -1;
        }
    }
    printf("BOARD_CHECK_PASS LS2K0300 GPIO=0x%" PRIx64 " mux=0x%" PRIx64 " valid_pins=106\n", GPIO_BASE, MUX_BASE);
    printf("MUX_MMIO_DISABLED declared_bytes=0x%" PRIx64 "; ownership metadata uses kernel debugfs.\n", mux_size);
    return 0;
}

static int map_readonly(int fd, uint64_t address, size_t span, struct readonly_region *region)
{
    long page_size = sysconf(_SC_PAGESIZE);
    if (page_size <= 0) return -1;
    uint64_t aligned = address / (uint64_t)page_size * (uint64_t)page_size;
    size_t offset = (size_t)(address - aligned);
    region->length = ((offset + span + (size_t)page_size - 1) / (size_t)page_size) * (size_t)page_size;
    region->mapping = mmap(NULL, region->length, PROT_READ, MAP_SHARED, fd, (off_t)aligned);
    if (region->mapping == MAP_FAILED) {
        region->mapping = NULL;
        fprintf(stderr, "Read-only mmap 0x%" PRIx64 ": %s\n", address, strerror(errno)); return -1;
    }
    region->bytes = (volatile const uint8_t *)region->mapping + offset;
    return 0;
}

static int monotonic_ns(uint64_t *value)
{
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return -1;
    *value = (uint64_t)now.tv_sec * UINT64_C(1000000000) + (uint64_t)now.tv_nsec;
    return 0;
}

static int finish_file(FILE *file)
{
    int failed = fflush(file) != 0;
    if (fsync(fileno(file)) != 0) failed = 1;
    if (fclose(file) != 0) failed = 1;
    return failed ? -1 : 0;
}

static int verify_saved_rows(const char *path, unsigned int expected_rows)
{
    FILE *file = fopen(path, "rb");
    if (!file) return -1;
    unsigned int lines = 0;
    int character;
    while ((character = fgetc(file)) != EOF) if (character == '\n') ++lines;
    int failed = ferror(file) || lines != expected_rows + 1;
    if (fclose(file) != 0) failed = 1;
    return failed ? -1 : 0;
}

/* Ownership describes kernel users/functions, not a raw hardware mux value.
 * Missing debugfs is recorded explicitly and does not prevent GPIO sampling. */
static int save_pinmux_owners(const char *directory, const char *phase)
{
    char path[256];
    int length = snprintf(path, sizeof(path), "%s/pinmux_owners_%s.txt", directory, phase);
    if (length < 0 || (size_t)length >= sizeof(path)) return -1;
    int fd = open(path, O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC, 0644);
    if (fd < 0) return -1;
    FILE *output = fdopen(fd, "w");
    if (!output) { close(fd); return -1; }
    fprintf(output, "phase=%s\nsource=%s\nmeaning=kernel_ownership_not_raw_mux\n", phase, PINMUX_OWNERS);
    FILE *input = fopen(PINMUX_OWNERS, "rb");
    int failed = 0;
    if (!input) {
        int source_errno = errno;
        fprintf(output, "status=unavailable errno=%d\n", source_errno);
        printf("PINMUX_OWNERS unavailable: %s; GPIO sampling remains enabled.\n", strerror(source_errno));
    } else {
        unsigned char buffer[4096];
        size_t size, total = 0;
        while ((size = fread(buffer, 1, sizeof(buffer), input)) != 0) {
            total += size;
            if (total > 65536 || fwrite(buffer, 1, size, output) != size) { failed = 1; break; }
        }
        if (ferror(input)) failed = 1;
        if (fclose(input) != 0) failed = 1;
        fprintf(output, "\nstatus=%s bytes=%zu\n", failed ? "read_error" : total ? "available" : "empty_report", total);
        if (total == 0 && !failed) {
            printf("PINMUX_OWNERS empty report; do not infer pin function.\n");
        }
    }
    if (finish_file(output) != 0) failed = 1;
    return failed ? -1 : 0;
}

int main(int argc, char **argv)
{
    const char *usage = "gray_gpio_levels version=" VERSION "\n"
                        "gray_gpio_levels --check | --stages\n"
                        "LS2K0300 GPIO input read-only diagnostic. No output/direction/mux writes.\n";
    if (argc == 2 && !strcmp(argv[1], "--help")) { fputs(usage, stdout); return 0; }
    int check = argc == 2 && !strcmp(argv[1], "--check");
    if (!check && !(argc == 2 && !strcmp(argv[1], "--stages"))) { fputs(usage, stderr); return 1; }
    printf("gray_gpio_levels version=%s\n", VERSION);
    fflush(stdout);
    if (validate_board() != 0) return 1;
    int lock_fd = open("/tmp/smart_car_2026.lock", O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (lock_fd < 0 || flock(lock_fd, LOCK_EX | LOCK_NB) != 0) {
        fprintf(stderr, "Cannot acquire vehicle lock; stop other capture/control tools.\n");
        if (lock_fd >= 0) close(lock_fd);
        return 1;
    }
    struct sigaction action = {0};
    action.sa_handler = stop_capture;
    sigemptyset(&action.sa_mask);
    int failed = sigaction(SIGINT, &action, NULL) != 0 || sigaction(SIGTERM, &action, NULL) != 0;
    int memory_fd = -1;
    struct readonly_region gpio = {0};
    FILE *samples = NULL, *metadata = NULL;
    if (failed) goto cleanup;
    memory_fd = open("/dev/mem", O_RDONLY | O_SYNC | O_CLOEXEC);
    if (memory_fd < 0) { fprintf(stderr, "Read-only /dev/mem: %s\n", strerror(errno)); failed = 1; goto cleanup; }
    if (map_readonly(memory_fd, GPIO_BASE, 0xA00 + PIN_COUNT, &gpio) != 0) { failed = 1; goto cleanup; }
    uint8_t initial[PIN_COUNT];
    for (unsigned int pin = 0; pin < PIN_COUNT; ++pin) initial[pin] = gpio.bytes[0xA00 + pin];
    printf("REGISTER_READ_PASS input_byte_alias=0x16104a00..0x16104a69\nGPIO0..105 low_bits=");
    for (unsigned int pin = 0; pin < PIN_COUNT; ++pin) printf("%u", initial[pin] & 1U);
    putchar('\n');
    if (check) goto cleanup;

    uint64_t session_time;
    if (monotonic_ns(&session_time) != 0) { failed = 1; goto cleanup; }
    time_t wall_time = time(NULL);
    struct tm utc;
    char stamp[32], directory[160], path[192];
    if (!gmtime_r(&wall_time, &utc) || !strftime(stamp, sizeof(stamp), "%Y%m%dT%H%M%SZ", &utc)) { failed = 1; goto cleanup; }
    snprintf(directory, sizeof(directory), "captures/gray_gpio_%s_%ld_%" PRIu64, stamp, (long)getpid(), session_time);
    if ((mkdir("captures", 0755) != 0 && errno != EEXIST) || mkdir(directory, 0755) != 0) { failed = 1; goto cleanup; }
    snprintf(path, sizeof(path), "%s/gpio_samples.csv", directory);
    int output_fd = open(path, O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC, 0644);
    if (output_fd < 0) { failed = 1; goto cleanup; }
    samples = fdopen(output_fd, "w");
    if (!samples) { close(output_fd); failed = 1; goto cleanup; }
    snprintf(path, sizeof(path), "%s/pin_config.csv", directory);
    output_fd = open(path, O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC, 0644);
    if (output_fd < 0) { failed = 1; goto cleanup; }
    metadata = fdopen(output_fd, "w");
    if (!metadata) { close(output_fd); failed = 1; goto cleanup; }
    fputs("phase,sample,read_start_ns,read_end_ns,input_raw_hex\n", samples);
    fputs("phase,gpio,gpio_oen_raw\n", metadata);
    printf("SESSION %s\n", directory);
    const char *phases[] = {"white_begin", "left1", "left2", "left3", "left4", "white_end"};
    const char *patterns[] = {"WWWW", "BWWW", "WBWW", "WWBW", "WWWB", "WWWW"};
    unsigned int total_rows = 0;
    unsigned int white_high_counts[PIN_COUNT] = {0};
    for (unsigned int phase = 0; phase < 6 && !interrupted && !failed; ++phase) {
        printf("\nPREPARE %s pattern=%s (vehicle left to right; B=black W=white).\n"
               "Keep the car still and normal sensor height. Press Enter when ready, or Q then Enter to stop: ", phases[phase], patterns[phase]);
        fflush(stdout);
        char answer[64];
        if (!fgets(answer, sizeof(answer), stdin) || answer[0] == 'q' || answer[0] == 'Q') { interrupted = 1; break; }
        if (strcmp(answer, "\n") != 0 && strcmp(answer, "\r\n") != 0) {
            fprintf(stderr, "Use Enter only after preparing the surface.\n"); failed = 1; break;
        }
        if (save_pinmux_owners(directory, phases[phase]) != 0) {
            fprintf(stderr, "Save kernel ownership metadata: failed.\n"); failed = 1; break;
        }
        for (unsigned int pin = 0; pin < PIN_COUNT; ++pin) {
            uint8_t oen_raw = gpio.bytes[0x800 + pin];
            fprintf(metadata, "%s,%u,%u\n", phases[phase], pin, oen_raw);
        }
        uint64_t next_time;
        if (monotonic_ns(&next_time) != 0) { failed = 1; break; }
        unsigned int changed[PIN_COUNT] = {0};
        unsigned int high_counts[PIN_COUNT] = {0};
        uint8_t previous[PIN_COUNT] = {0};
        printf("CAPTURING %s for about 3 seconds. Hold the pattern.\n", phases[phase]);
        for (unsigned int sample = 0; sample < 60 && !interrupted && !failed; ++sample) {
            uint8_t values[PIN_COUNT];
            uint64_t start, end;
            if (monotonic_ns(&start) != 0) { failed = 1; break; }
            for (unsigned int pin = 0; pin < PIN_COUNT; ++pin) values[pin] = gpio.bytes[0xA00 + pin];
            if (monotonic_ns(&end) != 0) { failed = 1; break; }
            fprintf(samples, "%s,%u,%" PRIu64 ",%" PRIu64 ",", phases[phase], sample, start, end);
            for (unsigned int pin = 0; pin < PIN_COUNT; ++pin) {
                fprintf(samples, "%02x", values[pin]);
                high_counts[pin] += values[pin] & 1U;
                if (sample && ((values[pin] ^ previous[pin]) & 1U)) ++changed[pin];
                previous[pin] = values[pin];
            }
            fputc('\n', samples);
            ++total_rows;
            if (ferror(samples) || ferror(metadata)) { failed = 1; break; }
            next_time += UINT64_C(50000000);
            struct timespec deadline = {(time_t)(next_time / UINT64_C(1000000000)), (long)(next_time % UINT64_C(1000000000))};
            int wait_status;
            do { wait_status = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL); }
            while (wait_status == EINTR && !interrupted);
            if (wait_status != 0 && !interrupted) failed = 1;
        }
        printf("DONE %s. GPIOs whose low bit changed during this phase:", phases[phase]);
        for (unsigned int pin = 0; pin < PIN_COUNT; ++pin) if (changed[pin]) printf(" %u(%u)", pin, changed[pin]);
        putchar('\n');
        if (!failed && !interrupted) {
            if (phase == 0) memcpy(white_high_counts, high_counts, sizeof(high_counts));
            else {
                printf("MAJORITY_DIFF_FROM_WHITE_BEGIN (candidate pins only):");
                for (unsigned int pin = 0; pin < PIN_COUNT; ++pin) {
                    if (white_high_counts[pin] != 30 && high_counts[pin] != 30 &&
                        (white_high_counts[pin] > 30) != (high_counts[pin] > 30))
                        printf(" %u(high %u/60 -> %u/60)", pin, white_high_counts[pin], high_counts[pin]);
                }
                putchar('\n');
            }
        }
        printf("Changes may include network/ultrasonic activity; they are not automatic gray mappings.\n");
    }
    if (finish_file(samples) != 0) failed = 1;
    samples = NULL;
    if (finish_file(metadata) != 0) failed = 1;
    metadata = NULL;
    if (!failed && !interrupted && total_rows == 360) {
        snprintf(path, sizeof(path), "%s/gpio_samples.csv", directory);
        if (verify_saved_rows(path, 360) != 0) failed = 1;
        snprintf(path, sizeof(path), "%s/pin_config.csv", directory);
        if (verify_saved_rows(path, 6 * PIN_COUNT) != 0) failed = 1;
    }
    if (!failed && !interrupted && total_rows == 360) printf("SAVED %s rows=%u phases=6\n", directory, total_rows);
    else { fprintf(stderr, "PARTIAL %s rows=%u; mapping not established.\n", directory, total_rows); failed = 1; }

cleanup:
    if (samples && finish_file(samples) != 0) failed = 1;
    if (metadata && finish_file(metadata) != 0) failed = 1;
    if (gpio.mapping && munmap(gpio.mapping, gpio.length) != 0) failed = 1;
    if (memory_fd >= 0 && close(memory_fd) != 0) failed = 1;
    if (close(lock_fd) != 0) failed = 1;
    if (failed) fprintf(stderr, "GPIO read-only diagnostic did not complete.\n");
    return failed || interrupted ? 1 : 0;
}
