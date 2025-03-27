#include "linklayer.h"

#define PACK_SIZE 32
#define FILE_NAME "/mnt/c/Users/Levy/Documents/GitHub/RCOM-Assign1/Dev/src/penguin.gif"
//#define FILE_NAME "C:/Users/Levy/Documents/GitHub/RCOM-Assign1/Dev/src/penguin.gif"
#define PORT "/dev/pts/4"

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
    printf("Application Layer\n");	
    ll = create_link_layer(PORT, BAUDRATE, TRANSMIT_TIMEOUT, MAX_TRANSMISSION_ATTEMPTS);
    al.status = TRANSMITTER;
    al.fileDescriptor = llopen(PORT, al.status);
    if(al.fileDescriptor < 0){
        printf("Error opening serial port\n");
        return -1;
    }

    long cycles = 0;

    FILE* file_pointer = fopen(FILE_NAME, "rb");
    if (file_pointer == NULL) {
        printf("The file is not opened. The program will now exit.\n");
        exit(0);
    }
    long file_size = get_file_size(file_pointer);

    u_int8_t packet_array[PACK_SIZE] = {0};

    printf("file_size: %ld\n", file_size);	
    while(ftell(file_pointer) < file_size){
        printf("cycles: %ld  ", cycles++);
        retrieve_packet(file_pointer, packet_array, PACK_SIZE, file_size);
        llwrite(al.fileDescriptor, packet_array, PACK_SIZE);
    }

    fclose(file_pointer);
    llclose(al.fileDescriptor);

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