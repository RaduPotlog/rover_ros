#include "baudrate_helper.h"
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>


bool set_custom_baudrate(const char* device, unsigned int baudrate) {
    int fd = ::open(device, O_RDWR | O_NOCTTY);
    if (fd < 0) return false;

    struct termios2 tio;
    if (::ioctl(fd, TCGETS2, &tio) < 0) { ::close(fd); return false; }

    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= CBAUDEX;
    tio.c_ispeed = baudrate;
    tio.c_ospeed = baudrate;

    bool ok = (::ioctl(fd, TCSETS2, &tio) == 0);
    ::close(fd);
    return ok;
}