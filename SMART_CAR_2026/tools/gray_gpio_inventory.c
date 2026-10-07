#define _POSIX_C_SOURCE 200809L
/* Linux GPIO v1 metadata only. Never request a line or change its direction. */
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <linux/gpio.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

int main(void)
{
    glob_t paths = {0};
    int result = glob("/dev/gpiochip*", 0, NULL, &paths);
    if (result != 0) {
        fprintf(stderr, "GPIO device discovery: %s\n",
                result == GLOB_NOMATCH ? "no gpiochip devices" : "glob failed");
        globfree(&paths);
        return 2;
    }
    int failed = 0;
    for (size_t index = 0; index < paths.gl_pathc; ++index) {
        const char *path = paths.gl_pathv[index];
        int fd = open(path, O_RDONLY | O_CLOEXEC);
        if (fd < 0) {
            fprintf(stderr, "%s: open: %s\n", path, strerror(errno));
            failed = 1;
            continue;
        }
        struct gpiochip_info chip = {0};
        if (ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &chip) < 0) {
            fprintf(stderr, "%s: chip info: %s\n", path, strerror(errno));
            failed = 1;
        } else {
            printf("CHIP path=%s name=%.32s label=%.32s lines=%u\n",
                   path, chip.name, chip.label, chip.lines);
            for (unsigned int offset = 0; offset < chip.lines; ++offset) {
                struct gpioline_info line = {0};
                line.line_offset = offset;
                if (ioctl(fd, GPIO_GET_LINEINFO_IOCTL, &line) < 0) {
                    fprintf(stderr, "%s offset=%u: line info: %s\n",
                            path, offset, strerror(errno));
                    failed = 1;
                    continue;
                }
                printf("LINE chip=%s offset=%u flags=0x%08x "
                       "kernel_used=%u reported_direction=%s active_low=%u "
                       "name=%.32s consumer=%.32s\n",
                       path, offset, line.flags,
                       !!(line.flags & GPIOLINE_FLAG_KERNEL),
                       (line.flags & GPIOLINE_FLAG_IS_OUT) ? "out" : "in",
                       !!(line.flags & GPIOLINE_FLAG_ACTIVE_LOW),
                       line.name, line.consumer);
            }
        }
        if (close(fd) < 0) {
            fprintf(stderr, "%s: close: %s\n", path, strerror(errno));
            failed = 1;
        }
    }
    globfree(&paths);
    /* Reported direction/name does not establish a physical U5 connection. */
    if (fflush(stdout) != 0 || ferror(stdout)) failed = 1;
    return failed ? 1 : 0;
}
