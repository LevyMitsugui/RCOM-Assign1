#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>


#define BUF_SIZE 32 //256
#define PACK_SIZE BUF_SIZE - 6

long get_file_size(FILE* file_pointer);
int retreive_packet(FILE* file_pointer, u_int8_t* packet_array, uid_t packet_size, long file_size);

int main()
{
    // file pointer variable to store the value returned by
    // fopen
    FILE* file_pointer;

    // opening the file in read mode
    file_pointer = fopen("penguin.gif", "rb");

    // checking if the file is opened successfully
    if (file_pointer == NULL) {
        printf("The file is not opened. The program will "
               "now exit.");
        exit(0);
    }else {
        printf("The file is created Successfully.\n");
    }
  
    fseek(file_pointer, 0, SEEK_END);
    long file_size = get_file_size(file_pointer);
    printf("file_size: %ld\n", file_size);

    int n_blocks = (int) file_size/((long)PACK_SIZE);
    n_blocks += 1;
    printf("Number of Packets: %d\n", n_blocks);

    u_int8_t buffer2[PACK_SIZE] = {0};
    int count = 0;

    while(ftell(file_pointer) < file_size){
        ++count;
        retreive_packet(file_pointer, buffer2, PACK_SIZE, file_size);


        if(count <2){
            for(int i=0; i<PACK_SIZE; i++){
                printf("Packet %d, idx %d, Value: %02x\n", count, i, buffer2[i]);
            }
        }
    }

    for(int i=0; i<PACK_SIZE; i++){
        printf("Packet %d, idx %d, Value: %02x\n", count, i, buffer2[i]);
    }

    printf("file_pointer: %ld\n", ftell(file_pointer));
    return 0;
}

long get_file_size(FILE* file_pointer){
    fseek(file_pointer, 0, SEEK_END);
    long ret = ftell(file_pointer);
    rewind(file_pointer);
    return ret;
}

int retreive_packet(FILE* file_pointer, u_int8_t* packet_array, uid_t packet_size, long file_size){

    if(ftell(file_pointer) >= file_size) return -1;

    float remainder_bytes = (file_size%packet_size);
    if((file_size - remainder_bytes) <= ftell(file_pointer)){

        fread(packet_array, 1, remainder_bytes, file_pointer);
        for(int i=remainder_bytes; i<packet_size;i++){
            packet_array[i] = 0;
        }
    } else {
        fread(packet_array, 1, PACK_SIZE, file_pointer);
    }
    return 1;
} 

