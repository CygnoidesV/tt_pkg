#include "tt_pkg/uart.hpp"

int uart_open(const char *device)
{
    int fd;
    while(1)
    {
        fd = open(device, O_RDWR | O_NOCTTY);

        if (fd == -1)
        {
            printf("Waiting for device.\n");
            usleep(1000000);
            return fd;
        }

        struct termios settings;
        tcgetattr(fd, &settings);
        cfsetospeed(&settings, B115200);
        cfsetispeed(&settings, B115200);
        settings.c_cflag &= ~PARENB;
        settings.c_cflag &= ~CSTOPB;
        settings.c_cflag &= ~CSIZE;
        settings.c_cflag |= CS8;
        settings.c_cflag &= ~CRTSCTS;
        settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        tcsetattr(fd, TCSANOW, &settings);

        // printf("uart is open.\n");
        break;
    }
    return fd;
}

int uart_close(int fd)
{
    return close(fd);
}

int uart_write(int fd, const char* data, int length)
{
    return write(fd, data, length);
}

int uart_read(int fd, char* buffer, int length)
{
    return read(fd, buffer, length);
}