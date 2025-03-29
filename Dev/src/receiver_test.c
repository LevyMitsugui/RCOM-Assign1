#include "linklayer.h"


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

    ll = create_link_layer(argv[1], BAUDRATE, TRANSMIT_TIMEOUT, MAX_TRANSMISSION_ATTEMPTS);

    al.fileDescriptor = llopen(argv[1], RECEIVER);
    if(al.fileDescriptor < 0){
        printf("Error opening serial port\n");
        return -1;
    }
    
    u_int8_t incoming_bytes[BUF_SIZE] = {0};
    int bytes_read = 0;
    int counter = 0;

    while(bytes_read >= 0){
        bytes_read = llread(al.fileDescriptor, incoming_bytes, BUF_SIZE);
        if (bytes_read <= 0)
        {
            printf("Error in llread\n");
            return -1;
        }
        printf("Cycle %d of Application Layer, Received %d bytes\n", counter, bytes_read);
        printf("Application Layer, Received: \n");
        for(int i = 0; i < bytes_read; i++){
            printf("%02x\n", incoming_bytes[i]);
        }
        counter++;
    }

    llclose(al.fileDescriptor);

    return 0;
}