#ifndef MC_EVENT_FORWARD_H
#define MC_EVENT_FORWARD_H
/* Portable select/eventfd policy. No protocol parsing, sending or motor state. */
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/select.h>
#include <unistd.h>

static inline size_t mc_forward_drain_count(size_t buffered, size_t capacity)
{
    return capacity < 2u ? 0u : buffered < capacity-1u ? buffered : capacity-1u;
}

static inline bool mc_forward_notify(int fd)
{
    const uint64_t one = 1u;
    return write(fd, &one, sizeof(one)) == (ssize_t)sizeof(one);
}

/* -1: error; 0: timeout or UART-only wake (caller drains queue at loop top);
 * 1: command socket readable. The original 20ms watchdog-check ceiling stays.
 * Only the UDP task reads the event counter; session validation remains above.
 */
static inline int mc_forward_wait(int socket_fd, int wake_fd)
{
    if (socket_fd < 0 || wake_fd < 0 || socket_fd == wake_fd
        || socket_fd >= FD_SETSIZE || wake_fd >= FD_SETSIZE)
        return -1;
    fd_set readable;
    FD_ZERO(&readable);
    FD_SET(socket_fd, &readable);
    FD_SET(wake_fd, &readable);
    struct timeval timeout = {.tv_sec=0, .tv_usec=20000};
    const int maximum = socket_fd > wake_fd ? socket_fd : wake_fd;
    const int result = select(maximum+1, &readable, NULL, NULL, &timeout);
    if (result < 0) return errno == EINTR ? 0 : -1;
    if (!result) return 0;
    if (FD_ISSET(wake_fd, &readable)) {
        uint64_t value;
        if (read(wake_fd, &value, sizeof(value)) != (ssize_t)sizeof(value)) return -1;
    }
    return FD_ISSET(socket_fd, &readable) ? 1 : 0;
}
#endif
