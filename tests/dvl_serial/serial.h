#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>

#ifdef __cplusplus
extern "C"
{
#endif

int open_serial_port(const char * device, uint32_t baud_rate);
int write_port(int fd, uint8_t * buffer, size_t size);
void write_command(int fd, uint8_t * cmd);


#ifdef __cplusplus
}
#endif

#endif  // _SERIAL_H_
