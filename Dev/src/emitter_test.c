#include "linklayer.h"

#define PACK_SIZE 16

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

    for(int i = 0; i < PACK_SIZE; i++){
        buf[i] = i;
    }

    buf[3] = 0x7e;
    // buf[8] = 0x7d;
    // buf[11] = 0x7e;

    printf("Application Layer sent: \n");
    for(int i = 0; i < PACK_SIZE; i++){
        printf("%02x\n", buf[i]);
    }

    al.fileDescriptor = llopen(argv[1], TRANSMITTER);
    written_bytes = llwrite(al.fileDescriptor, buf, PACK_SIZE);
    printf("Application Layer: %d bytes written\n", written_bytes);

    llclose(al.fileDescriptor);

    return 0;
}

