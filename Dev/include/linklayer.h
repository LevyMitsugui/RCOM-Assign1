#ifndef LINKLAYER_H
#define LINKLAYER_H

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

#define TRUE 1
#define FALSE 0

#define TRANSMITTER 0
#define RECEIVER 1
#define BAUDRATE B38400
#define RECEIVE_TIMEOUT 30 // in seconds
#define TRANSMIT_TIMEOUT 2
#define MAX_TRANSMISSION_ATTEMPTS 3

#define BUF_SIZE 32 //256

#define FLAG            0x7e
#define ADDRESS_RECV     0x03
#define CONTROL_SET     0x03 

#define CONTROL_FRAME_0 0x00
#define CONTROL_FRAME_1 0x40

#define ADDRESS_EMIT  	0x01
#define CONTROL_UA      0x07
#define CONTROL_RR0     0x05
#define CONTROL_RR1     0x85
#define CONTROL_REJ0    0x01
#define CONTROL_REJ1    0x81
#define CONTROL_DISC    0x0B

typedef struct{
    char port[20]; /*Device /dev/ttySx, x = 0, 1*/
    int baudRate; /*Speed of the transmission*/
    unsigned int sequenceNumber; /*Frame sequence number: 0, 1*/
    unsigned int timeout; /*Timer value: 1 s*/
    unsigned int numTransmissions; /*Number of retries in case of
    failure*/
    char frame[BUF_SIZE]; /*Frame*/
} linkLayer;

extern linkLayer ll;

linkLayer create_link_layer(const char *port, int baudRate, uid_t timeout, uid_t numTransmissions);

int llopen(const char *port, int role);

int llread(int fd, u_int8_t* buf, int length);

int llwrite(int fd, const u_int8_t* buf, int length);

int llclose(int fd);

void setFrame_SET(u_int8_t* buf);

void setFrame_UA(u_int8_t* buf);

void setFrame_DISC(u_int8_t* buf);

int confirm_frame_control(u_int8_t* receiver_buf, u_int8_t control);


#endif