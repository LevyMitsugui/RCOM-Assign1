#include "linklayer.h"

#define PACK_SIZE 250
#define FILE_NAME "/mnt/c/Users/Levy/Documents/GitHub/RCOM-Assign1/Dev/src/penguin.gif"
//#define FILE_NAME "/mnt/c/Users/Levy/Documents/GitHub/RCOM-Assign1/Dev/src/smol.jpg"
//#define FILE_NAME "C:/Users/Levy/Documents/GitHub/RCOM-Assign1/Dev/src/penguin.gif"
#define PORT "/dev/pts/4"

long get_file_size(FILE* file_pointer);
int retrieve_packet(FILE* file_pointer, u_int8_t* packet_array, size_t packet_size, long file_size);

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

    int bytes_written = 0;

    int packet_len;
    while ((packet_len = retrieve_packet(file_pointer, packet_array, PACK_SIZE, file_size)) > 0) {
        bytes_written = llwrite(al.fileDescriptor, packet_array, packet_len);
    }

    fclose(file_pointer);

    
    sleep(4);
    llclose(al.fileDescriptor);

    return 0;
}

int retrieve_packet(FILE* file_pointer, u_int8_t* packet_array, size_t packet_size, long file_size) {
    long current_pos = ftell(file_pointer);
    if (current_pos >= file_size) return 0;  // EOF, nothing left to read

    size_t bytes_left = file_size - current_pos;
    size_t bytes_to_read = (bytes_left < packet_size) ? bytes_left : packet_size;

    size_t bytes_read = fread(packet_array, 1, bytes_to_read, file_pointer);

    return bytes_read; // Actual number of bytes retrieved
}