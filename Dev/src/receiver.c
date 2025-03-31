#include "linklayer.h"
#include <stdio.h>

#define PACK_SIZE 16
#define MAXIMUM_FILE_SIZE 0x80000//16384
#define FILE_NAME "penguin.gif"
#define PORT "/dev/pts/5"

long get_file_size(FILE* file_pointer);
int retrieve_packet(FILE* file_pointer, u_int8_t* packet_array, uid_t packet_size, long file_size);

struct applicationLayer {
    int fileDescriptor; /*Serial port descriptor*/
    int status; /*TRANSMITTER | RECEIVER*/
} al;

linkLayer ll;

long get_file_size(FILE* file_pointer){
    fseek(file_pointer, 0, SEEK_END);
    long ret = ftell(file_pointer);
    rewind(file_pointer);
    return ret;
}


int main(int argc, char *argv[]) {
    ll = create_link_layer(PORT, BAUDRATE, TRANSMIT_TIMEOUT, MAX_TRANSMISSION_ATTEMPTS);
    al.status = RECEIVER;
    al.fileDescriptor = llopen(PORT, al.status);

    long cycles = 0;
    long bytes_until_now = 0;

    long bytes_read = 0;
    long index_file = 0;
    long file_size = 0;
    u_int8_t buf_file[MAXIMUM_FILE_SIZE] = {0};
    u_int8_t buf_packet[PACK_SIZE] = {0};

    while(1){
        bytes_read = llread(al.fileDescriptor, &buf_file[index_file], PACK_SIZE);
        if (bytes_read <= 0)
        {
            printf("Error in llread\n");
            break; //return -1;
        }
        index_file += bytes_read;
        cycles+=1;
        bytes_until_now += bytes_read;
        printf("Cycle %ld of Application Layer, Received %ld bytes\n", cycles, bytes_read);
    }
    //llclose(al.fileDescriptor);

    FILE *file = fopen("output.gif", "wb");
    size_t bytes_written = fwrite(buf_file, 1, index_file, file);
    if(bytes_written != index_file){
        printf("Error writing file\n");
        return -1;
    }
    fclose(file);

    

    return 0;
}
