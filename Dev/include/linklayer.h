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
#define RECEIVE_TIMEOUT 60 // in seconds
#define TRANSMIT_TIMEOUT 3
#define MAX_TRANSMISSION_ATTEMPTS 5

#define SUPERV_FRAME_SIZE 5
#define BUF_SIZE 64 //256
#define HEADER_SIZE 4
#define FOOTER_SIZE 2
//#define PACKET_SIZE BUF_SIZE-HEADER_SIZE-FOOTER_SIZE - 1 // -1 in case the bcc2 is 2 bytes

#define FLAG            0x7E
#define ADDRESS_RECV    0x03
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
    int status; /*TRANSMITTER | RECEIVER*/
    unsigned int sequenceNumber; /*Frame sequence number: 0, 1*/
    unsigned int timeout; /*Timer value: 1 s*/
    unsigned int numTransmissions; /*Number of retries in case of
    failure*/
    char frame[BUF_SIZE]; /*Frame*/
} linkLayer;

typedef enum { START, FLAG_RCV, A_RCV, C_RCV, C_FRAME_0, C_FRAME_1, BCC_OK, BCC2 , DATA, DATA_DESTUFF , SEND ,STP} READ_STATE;

extern linkLayer ll;

linkLayer create_link_layer(const char *port, int baudRate, uid_t timeout, uid_t numTransmissions);

int llopen(const char *port, int role);

int llread(int fd, u_int8_t* buf, int length);

int llwrite(int fd,u_int8_t* buf, int length);

int llwrite_test(int fd, u_int8_t* buf, int length);

int llclose(int fd);

int send_frame(u_int8_t*sender_buf, u_int8_t* receiver_buf, uid_t attempts, uid_t timeout, int fd);

int destuff_bytes(u_int8_t* orig, u_int8_t* target, uid_t init_index, uid_t final_index);

int stuff_bytes(u_int8_t* data_packet, u_int8_t* buf, uid_t packet_size, uid_t offset);

u_int8_t array_xor(u_int8_t* array, int arr_size, uid_t init_index, uid_t final_index);

void setFrame_SET(u_int8_t* buf);

void setFrame_UA(u_int8_t* buf);

void setFrame_SUP(u_int8_t* buf, u_int8_t control);

void setFrame_DISC(u_int8_t* buf);

void setFrame_MOCK1(u_int8_t* buf);

int setFrame_DATA(u_int8_t* buf, u_int8_t* data_packet, uid_t packet_size, u_int8_t control);

int confirm_header(u_int8_t* receiver_buf);

int confirm_frame_control(READ_STATE* state_machine, int byte, u_int8_t control);

//int confirm_frame(u_int8_t* receiver_buf, u_int8_t* control);
int confirm_frame(READ_STATE* state_machine, u_int8_t byte, u_int8_t control);
  
#endif