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
    int written_bytes = 0;

    al.fileDescriptor = llopen(argv[1], TRANSMITTER);
    printf("file Descriptor: %d\n", al.fileDescriptor);
    written_bytes = llwrite(al.fileDescriptor, "abcdefghij", 11);
    printf("%d bytes written\n", written_bytes);
    llclose(al.fileDescriptor);

    return 0;
}