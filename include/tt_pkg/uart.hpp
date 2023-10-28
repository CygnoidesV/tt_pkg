#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

#define DEVICE "/dev/ttyUSB1"

int uart_open(const char *device);
int uart_close(int fd);
int uart_write(int fd, const char *data, int length);
int uart_read(int fd, char *buffer, int length); 
