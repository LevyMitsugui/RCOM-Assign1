#include "linklayer.h"

struct applicationLayer {
    int fileDescriptor; /*Serial port descriptor*/
    int status; /*TRANSMITTER | RECEIVER*/
} al;

int main(int argc, char *argv[]){
    if (argc < 2)
    {
        exit(1);
    }


    al.fileDescriptor = llopen(argv[1], RECEIVER);
    printf("file Descriptor: %d\n", al.fileDescriptor);
    
    u_int8_t incoming_bytes[BUF_SIZE] = {0};
    llread(al.fileDescriptor, incoming_bytes, BUF_SIZE);

    for(int i = 0; i < BUF_SIZE; i++){
        if(incoming_bytes[i] == '\0') break;
        printf("%02x\n", incoming_bytes[i]);
    }

    llclose(al.fileDescriptor);

    return 0;
}