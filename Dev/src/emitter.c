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
    int written_bytes = 0;

    ll = create_link_layer(argv[1], BAUDRATE, TRANSMIT_TIMEOUT, MAX_TRANSMISSION_ATTEMPTS);

    al.fileDescriptor = llopen(argv[1], TRANSMITTER);
    written_bytes = llwrite(al.fileDescriptor, "abcdefghij", 11);
    printf("%d bytes written\n", written_bytes);

    llclose(al.fileDescriptor);

    return 0;
}