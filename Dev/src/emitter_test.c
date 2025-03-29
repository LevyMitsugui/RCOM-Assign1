#include "linklayer.h"

#define PACK_SIZE 10

struct applicationLayer {
    int fileDescriptor; /*Serial port descriptor*/
    int status; /*TRANSMITTER | RECEIVER*/
} al;

linkLayer ll;

int main(int argc, char *argv[]){
    if (argc < 2)
    {
        exit(1);
    }
    int written_bytes = 0;

    ll = create_link_layer(argv[1], BAUDRATE, TRANSMIT_TIMEOUT, MAX_TRANSMISSION_ATTEMPTS);

    u_int8_t buf[PACK_SIZE] = {0};
    int incoming_byte = 0;

    for(int i = 0; i < PACK_SIZE; i++){
        buf[i] = i;
    }

    buf[0] = 0x7e;
    buf[1] = 0x03;
    buf[2] = 0x00;
    buf[3] = 0x03;
    buf[4] = 0x01;
    buf[5] = 0x02;
    buf[6] = 0x03;
    buf[7] = 0x04;
    buf[8] = 0x04;
    buf[9] = 0x7e;


    // buf[5] = 0x7d;
    // buf[6] = 0x7d;
    // buf[7] = 0x7e;

    

    al.fileDescriptor = llopen(argv[1], TRANSMITTER);
    if(al.fileDescriptor < 0){
        printf("Error opening serial port\n");
        return -1;
    }
    sleep(3);

    written_bytes = write(al.fileDescriptor, buf, PACK_SIZE);

    // written_bytes = llwrite(al.fileDescriptor, buf, PACK_SIZE);
    // printf("Application Layer sent: \n");
    // for(int i = 0; i < PACK_SIZE; i++){
    //     printf("%02x\n", buf[i]);
    // }
    printf("Application Layer: %d bytes written\n", written_bytes);

    for(int j=0; j<8; j++){
    sleep(2);
    buf[0] = 0x7e;
    buf[1] = 0x03;
    buf[2] = 0x40;
    buf[3] = 0x01;//0x43;
    buf[4] = 0x01;
    buf[5] = 0x02;
    buf[6] = 0x03;
    buf[7] = 0x04;
    buf[8] = 0x04;
    buf[9] = 0x7e;
    written_bytes = write(al.fileDescriptor, buf, PACK_SIZE);
    // written_bytes = llwrite(al.fileDescriptor, buf, PACK_SIZE);
    // printf("Application Layer sent: \n");
    // for(int i = 0; i < PACK_SIZE; i++){
    //     printf("%02x\n", buf[i]);
    // }

    printf("Application Layer: %d bytes written\n", written_bytes);
    }

    for(int i = 0; i<100; i++){
        read(al.fileDescriptor, &incoming_byte, 1);
        printf("readin[%d]: %02x\n", i, incoming_byte);
    }

    llclose(al.fileDescriptor);

    return 0;
}

