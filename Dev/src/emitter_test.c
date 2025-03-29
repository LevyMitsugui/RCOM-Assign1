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
    u_int8_t bcc2 = 0;
    int incoming_byte = 0; 

    al.fileDescriptor = llopen(argv[1], TRANSMITTER);
    if(al.fileDescriptor < 0){
        printf("Error opening serial port\n");
        return -1;
    }
    sleep(0.5);

    for(int j=0; j<3; j++){
        sleep(3);
        // buf[0] = 0x7e;
        // buf[1] = 0x03;
        // buf[2] = (j%2 == 0) ? 0x00 : 0x40;
        // buf[3] = buf[1] ^ buf[2]; 
        // bcc2 = 0;
        // for(int i = 4; i<PACK_SIZE-2; i++){
        //     buf[i] = (i-3) + (16*j);
        // }
        // buf[6]= 0x7d; 
        // buf[7] = 0x5e;
        // for(int i = 4; i<PACK_SIZE-2; i++){
        //     bcc2 ^= buf[i];
        // } 
        // if (bcc2 == 0x7e || bcc2 == 0x7d){
        //     printf("TEST WITH OTHER VALUES\n");
        //     return 0;
        // }
        // buf[PACK_SIZE-2] = bcc2;
        // buf[PACK_SIZE-1] = 0x7e;
        // written_bytes = write(al.fileDescriptor, buf, PACK_SIZE);


        for(int i = 0; i < PACK_SIZE; i++){
            buf[i] = i + (16*j);
        }

        written_bytes = llwrite(al.fileDescriptor, buf, PACK_SIZE);
        printf("Application Layer: %d bytes written\n", written_bytes);
        printf("Application Layer sent: \n");
        for(int i = 0; i < PACK_SIZE; i++){
            printf("%02x\n", buf[i]);
        }

    }

    for(int i = 0; i<100; i++){
        read(al.fileDescriptor, &incoming_byte, 1);
        printf("reading[%d]: %02x\n", i, incoming_byte);
    }

    llclose(al.fileDescriptor);

    return 0;
}

