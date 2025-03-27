#include "linklayer.h"
#include <stdio.h>

#define PACK_SIZE 16
#define MAXIMUM_FILE_SIZE 16384
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

    long index_file = 0;
    long file_size = 0;
    u_int8_t buf_file[MAXIMUM_FILE_SIZE] = {0};
    u_int8_t buf_packet[PACK_SIZE] = {0};

    while(llread(al.fileDescriptor, buf_packet, MAXIMUM_FILE_SIZE) >= PACK_SIZE){
        printf("cycles: %ld  ", cycles++);
        for(int i = 0; i < PACK_SIZE; i++){
            buf_file[index_file] = buf_packet[i];
            index_file+=1;
        }
        bytes_until_now += PACK_SIZE;
        printf("bytes_until_now: %ld  ", bytes_until_now);
        if(bytes_until_now == file_size){
            printf("file_size: %ld\n", file_size);
            break;
        }
    };
    llclose(al.fileDescriptor);

    FILE *file = fopen("output.gif", "wb");
    size_t bytes_written = fwrite(buf_file, 1, index_file, file);
    if(bytes_written != index_file){
        printf("Error writing file\n");
        return -1;
    }
    fclose(file);

    

    return 0;
}




int retrieve_packet(FILE* file_pointer, u_int8_t* packet_array, uid_t packet_size, long file_size){

    if(ftell(file_pointer) >= file_size) return -1;

    float remainder_bytes = (file_size%packet_size);
    if((file_size - remainder_bytes) <= ftell(file_pointer)){

        fread(packet_array, 1, remainder_bytes, file_pointer);
        for(int i=remainder_bytes; i<packet_size;i++){
            packet_array[i] = 0;
        }
        return 0;
    } else {
        fread(packet_array, 1, packet_size, file_pointer);
    }

    // for(int i=0; i<packet_size; i++){
    //     printf("packet[%d]: %02x\n", i, packet_array[i]);
    // }
    return 1;
}