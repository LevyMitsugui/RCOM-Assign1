// Write to serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define TIMEOUT 2
#define MAX_ATTEMPTS 3

#define BUF_SIZE 32 //256
#define PACKET_SIZE BUF_SIZE-6

#define FLAG            0x7E
#define ADDRESS_SET     0x03
#define CONTROL_SET     0x03 

#define CONTROL_FRAME_0 0x00
#define CONTROL_FRAME_1 0x40

#define ADDRESS_UA  	0x01
#define CONTROL_UA      0x07
#define CONTROL_RR0     0x05
#define CONTROL_RR1     0x85
#define CONTROL_REJ0    0x01
#define CONTROL_REJ1    0x81
#define CONTROL_DISC    0x0B

#define FILE_NAME "penguin.gif"

//#define DEBUG
//#define BYPASS

volatile int STOP = FALSE;

enum OP_STATE {SET, CONFIRM_UA, GET_0, GET_1, DISC,CONFIRM, SEND, OP_STP};
enum READ_STATE { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK , STP};

int alarmEnabled = FALSE;
int alarmCount = 0;

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
}

struct FILE_INFO{
    FILE *fp;
    long file_size;
    int packet_size;
    int n_packets;
    int current_packet;
};

/**
 * @brief Does a simple xor operation between two unsigned chars.
 * 
 * @param byte1 First operand.
 * @param byte2 Second Operand.
 * @return u_int8_t - The resulting xor value.
 */
u_int8_t byte_xor(u_int8_t byte1, u_int8_t byte2);

/**
 * @brief Applies XOR between all bytes stored in @p array between the @p init_index and the @p final_index.
 * 
 * @param array Array that the bytes are stored in.
 * @param arr_size Size of the array.
 * @param init_index Index of the first element for the xor operation
 * @param final_index Index of the last element for the xor operation
 * @return u_int8_t - The resulting xor value between the @p init_index and @p final_index elements in @p array.
 */
u_int8_t array_xor(u_int8_t *array, int arr_size, uid_t init_index, uid_t final_index);

/**
 * @brief Send frame stored at @p sender_buf. If a response is successfully registered, it will store the response at @p receiver_buf 
 * then it will return the number of bytes read. If not, it will return -1. 
 * 
 * @param sender_buf Pointer to the buffer containing the frame to be sent.
 * @param receiver_buf Pointer to the buffer where the received response will be stored.
 * @param attempts The maximum number of retransmission attempts before giving up.
 * @param timeout The timeout duration (in seconds) before retransmitting the frame.
 * @param fd File descriptor of the communication channel.
 * 
 * @return int - Number of bytes read if a response is received successfully, -1 otherwise.
 */
int send_frame(u_int8_t*sender_buf, u_int8_t* receiver_buf, uid_t attempts, uid_t timeout, int fd);


/**
 * @brief Do Byte Stuffing for all bytes stored in @p data_packet. Result is stored directly in @p buf starting at @p offset index.
 * 
 * @param data_packet Array where all bytes of data is stored (and only data bytes, it is not the frame array)
 * @param buf Array that the frame will be constructed into.
 * @param packet_size Size of the data packet array.
 * @param offset Index to start recording the byte stuffing.
 * @return int - The number of bytes added by stuffing.
 */
int stuff_bytes(u_int8_t* data_packet, u_int8_t* buf, uid_t packet_size, uid_t offset);


/**
 * @brief Constructs the SET frame inside @p buf array, starting at 0 index.
 * 
 * @param buf Array to construct SET frame into
 */
void setFrame_SET(u_int8_t* buf);


/**
 * @brief Constructs the Information frame (or data frame) inside @p buf array, starting at 0 index.
 * 
 * @param buf Array to construct DATA frame into.
 * @param data_packet Array containing all bytes to be sent (not stuffed).
 * @param packet_size Size of the data packet array.
 * @param control Control byte.
 */
void setFrame_DATA(u_int8_t* buf, u_int8_t* data_packet, uid_t packet_size, u_int8_t control);

void setFrame_DISC(u_int8_t* buf);

/**
 * @brief State Machine that Processes responses from receiver.
 * 
 * @param receiver_buf Array containing the received bytes of data.
 * @param control_variable Variable to store the control byte.
 * @return int - 1 if expected message, -1 otherwise.
 */
int confirm_frame(u_int8_t* receiver_buf, u_int8_t* control_variable);

int confirm_frame_UA(u_int8_t* receiver_buf);

long get_file_size(FILE* file_pointer);
int retrieve_packet(FILE* file_pointer, u_int8_t* packet_array, uid_t packet_size, long file_size);

int llopen(const char *port, int role);
int llread(int fd, u_int8_t* buf, int length);
int llwrite(int fd, const u_int8_t* buf, int length);
int llclose(int fd);


int main(int argc, char *argv[]){
    (void)signal(SIGALRM, alarmHandler);
    // Program usage: Uses either COM1 or COM2
    const char *serialPortName = argv[1];

    if (argc < 2)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS1\n",
               argv[0],
               argv[0]);
        exit(1);
    }

    // Open serial port device for reading and writing, and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    int fd = open(serialPortName, O_RDWR | O_NOCTTY);

    if (fd < 0)
    {
        perror(serialPortName);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VMIN] = 0;  // Blocking read until 5 chars received
    newtio.c_cc[VTIME] = 5; // Inter-character timer unused

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    // --- --- --- --- --- --- --- --- ---
    enum OP_STATE macro_state;
    macro_state = SET;

    unsigned char buf[BUF_SIZE] = {0};
    unsigned char buf_receiver[BUF_SIZE + 1] = {0};

    int retriever_status=0;
    
    struct FILE_INFO file_info;
    file_info.fp = fopen(FILE_NAME, "rb");
    if (file_info.fp == NULL) {
        printf("The file is not opened. The program will "
               "now exit.");
        exit(0);
    } else {
        #ifdef DEBUG
        printf("The file is created Successfully.\n");
        #endif
    }
    file_info.file_size = get_file_size(file_info.fp);
    file_info.packet_size = PACKET_SIZE;
    file_info.n_packets = (int) file_info.file_size/file_info.packet_size;
  

    // u_int8_t mock_data_packet[] = {0x1e, 0x23, 0x7e, 0x5e, 0x7e, 0xa2, 0x12, 0x24, 0x7e, 0x59, 0x42};
    // u_int8_t mock_data_packet[] = {0x47, 0x49, 0x46, 0x38, 0x39, 0x61, 0x01, 0x01, 0x2f, 0x01, 0xf5};
    // uid_t mock_packet_size = 11;
    u_int8_t data_packet[PACKET_SIZE] = {0};
    uid_t packet_size = PACKET_SIZE;
    // retrieve_packet(file_info.fp, data_packet, packet_size, file_info.file_size);
    // for(int i=0; i<packet_size; i++){
    //     printf("from retrieve Packet[%d]: %02x\n", i, data_packet[i]);
    // }

    int bytes_read = 0;
    u_int8_t control_received = 0;

    while(STOP == FALSE){
        switch (macro_state){
            case SET:
                setFrame_SET(buf);
                macro_state = SEND;
                break;

            case GET_0:
                retriever_status = retrieve_packet(file_info.fp, data_packet, packet_size, file_info.file_size);
                setFrame_DATA(buf, data_packet, packet_size, CONTROL_FRAME_0);
                macro_state = SEND;
                break;
            
            case GET_1:
                retriever_status = retrieve_packet(file_info.fp, data_packet, packet_size, file_info.file_size);
                setFrame_DATA(buf, data_packet, packet_size, CONTROL_FRAME_1);
                macro_state = SEND;
                break;

            case DISC:
                setFrame_DISC(buf);
                macro_state = SEND;
                break;

            case SEND:
                #ifdef DEBUG
                printf("DEBUG: macro_state SEND\n");
                printf(":%s:%d\n", buf, bytes_read);
                for(int i=0;i< BUF_SIZE;i++){
                    printf("%02x\n",buf[i]);
                    if(buf[i] == 0x7E && i != 0) break;
                }
                #endif
            
                bytes_read = send_frame(buf, buf_receiver, MAX_ATTEMPTS, TIMEOUT, fd);

                #ifdef DEBUG
                printf("DEBUG: bytes_read: %d\n", bytes_read);	
                #endif
                
                if (bytes_read <= 0){
                    printf("Did not receive message after %d attempts. Terminating code\n", MAX_ATTEMPTS);
                    macro_state = OP_STP;
                    #ifdef BYPASS
                    macro_state = CONFIRM_UA;
                    #endif
                } else if(buf[2]==CONTROL_SET) {
                    macro_state = CONFIRM_UA;
                } else {
                    macro_state = CONFIRM;
                }
                break;

            case CONFIRM_UA:
                macro_state = (confirm_frame_UA(buf_receiver) == -1) ? SET : GET_0;
                break;

            case CONFIRM:
                if (confirm_frame(buf_receiver, &control_received) == -1){
                    macro_state = SEND; //ERROR IN THE FRAME RECEIVED, TRYING AGAIN
                } else if((control_received == CONTROL_RR0 || control_received == CONTROL_RR1) && retriever_status == 0){
                    macro_state = DISC;
                } else if (control_received == CONTROL_RR0){
                    macro_state = GET_0;
                } else if (control_received == CONTROL_RR1){
                    macro_state = GET_1;
                } else if (control_received == CONTROL_REJ0 || control_received == CONTROL_REJ1){
                    macro_state = SEND;
                } else if (control_received == CONTROL_DISC){
                    macro_state = OP_STP;
                }

                #ifdef BYPASS
                macro_state = GET_0;
                #endif

                break;

            case OP_STP:
                #ifdef DEBUG
                printf("DEBUG: macro_state OP_STP\n");
                #endif
                STOP = TRUE;
                break;

            default:
                #ifdef DEBUG
                printf("DEBUG: macro_state default\n");
                printf("Unexpected value for macro_state: %d\n", macro_state);
                #endif
                break;
            
        }
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }
    close(fd);

    return 0;
}

u_int8_t byte_xor(u_int8_t byte1, u_int8_t byte2){
    return byte1^byte2;
}

u_int8_t array_xor(u_int8_t* array, int arr_size, uid_t init_index, uid_t final_index){
    #ifdef DEBUG
    printf("DEBUG: array_xor\n");
    #endif

    if(final_index >= arr_size) return 0;
    unsigned char ret_xor = 0;
    for (int i = init_index; i<=final_index; i++){
        ret_xor = array[i] ^ ret_xor;
        
        #ifdef DEBUG
        printf("ret_xor[%d]: %02x\n", i, ret_xor);
        #endif
    }
    return ret_xor;
}

// int send(int fd, u_int8_t* frame, int frame_size){
    
// }

int send_frame(u_int8_t*sender_buf, u_int8_t* receiver_buf, uid_t attempts, uid_t timeout, int fd){

    int bytes = 0;
    int bytes_read = 0;

    while (alarmCount < attempts){
        if (alarmEnabled == FALSE){
            bytes = write(fd, sender_buf, BUF_SIZE);
            alarm(timeout);
            alarmEnabled = TRUE;
        }
        bytes_read = read(fd, receiver_buf, BUF_SIZE);
        receiver_buf[bytes_read] = '\0'; // Set end of string to '\0', so we can printf

        if(bytes_read != 0)
            break;
    }
    alarm(0);
    alarmEnabled = FALSE;
    alarmCount = 0;
    return bytes_read;
}

int stuff_bytes(u_int8_t* data_packet, u_int8_t* buf, uid_t packet_size, uid_t offset){
    int n_stuffed_bytes = 0;
    for(int i = 0; i<packet_size; i ++){
        //printf("%02x\n",data_packet[i]);

        if(data_packet[i] == 0x7e){
            buf[offset + i + n_stuffed_bytes]= 0x7d;
            n_stuffed_bytes++;
            buf[offset + i + n_stuffed_bytes]= 0x5e;
        } else if(data_packet[i] == 0x7d){
            buf[offset + i + n_stuffed_bytes]= 0x7d;
            n_stuffed_bytes++;
            buf[offset + i + n_stuffed_bytes]= 0x5d;
        } else {
            buf[offset+ i + n_stuffed_bytes] = data_packet[i];
        }
    }

    for(int i=0;i< BUF_SIZE;i++){
        
        if(buf[i] == 0x7E && i != 0) break;
    }
    return n_stuffed_bytes;
} 

void setFrame_SET(u_int8_t* buf){
    buf[0] = FLAG;
    buf[1] = ADDRESS_SET;
    buf[2] = CONTROL_SET;
    buf[3] = byte_xor(buf[1], buf[2]); 
    buf[4] = FLAG;
} 

void setFrame_DISC(u_int8_t* buf){
    buf[0] = FLAG;
    buf[1] = ADDRESS_SET;
    buf[2] = CONTROL_DISC;
    buf[3] = byte_xor(buf[1], buf[2]); 
    buf[4] = FLAG;
}

void setFrame_DATA(u_int8_t* buf, u_int8_t* data_packet, uid_t packet_size, u_int8_t control){
    u_int8_t bcc2;
    
    buf[0] = FLAG;
    buf[1] = ADDRESS_SET;
    buf[2] = control;
    buf[3] = buf[1] ^ buf[2]; 
    //Assemble Data Packet W/ byte stuffing
    int added_bytes = stuff_bytes(data_packet, buf, packet_size, 4); //number of bytes added by the stuffing function
    
    bcc2 = array_xor(data_packet, packet_size, 0, packet_size-1); //BCC2
    
    if(bcc2 == 0x7e){
        buf[4+packet_size+added_bytes] = 0x7d;
        added_bytes+=1;
        buf[4+packet_size+added_bytes] = 0x5e;
    } else if(bcc2 == 0x7d){
        buf[4+packet_size+added_bytes] = 0x7d;
        added_bytes+=1;
        buf[4+packet_size+added_bytes] = 0x5d;
    }
    buf[4+packet_size+added_bytes] = bcc2; //BCC2
    buf[4+packet_size+added_bytes+1] = FLAG;                                              //FLAG
    #ifdef DEBUG
    printf("DEBUG: setFrame_DATA\n");
    for(int i=0; i< 4+packet_size+added_bytes+3; i++){
        printf("buf[%d] = %02x\n", i, buf[i]);
    }
    #endif
}

long get_file_size(FILE* file_pointer){
    fseek(file_pointer, 0, SEEK_END);
    long ret = ftell(file_pointer);
    rewind(file_pointer);
    return ret;
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

struct applicationLayer {
    int fileDescriptor; /*Serial port descriptor*/
    int status; /*TRANSMITTER | RECEIVER*/
};
struct linkLayer {
    char port[20]; /*Device /dev/ttySx, x = 0, 1*/
    int baudRate; /*Speed of the transmission*/
    unsigned int sequenceNumber; /*Frame sequence number: 0, 1*/
    unsigned int timeout; /*Timer value: 1 s*/
    unsigned int numTransmissions; /*Number of retries in case of
    failure*/
    char frame[BUF_SIZE]; /*Frame*/
};

int llopen(const char *port, int role){
    struct applicationLayer* al = malloc(sizeof(struct applicationLayer));
    if (al == NULL) return -1;
    struct linkLayer* ll = malloc(sizeof(struct linkLayer));
    if (ll == NULL) return -1;

    strncpy(ll->port, port, 20);
    al->fileDescriptor = open(port, O_RDWR | O_NOCTTY);
    al->status = role;

    if (al->fileDescriptor < 0)
    {
        perror(ll->port);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    if (tcgetattr(al->fileDescriptor, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VMIN] = 0;  // Blocking read until 5 chars received
    newtio.c_cc[VTIME] = 5; // Inter-character timer unused

    tcflush(al->fileDescriptor, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(al->fileDescriptor, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    setFrame_SET(ll->frame);
}

int confirm_frame_UA(u_int8_t* receiver_buf){
    enum READ_STATE read_state;
    read_state = START;
    int ret = -1;
    for(int j = 0; j < BUF_SIZE; j++){
        switch(read_state){
            case START:
                if(receiver_buf[j]== FLAG){
                    read_state= FLAG_RCV;
                }
                else read_state=START;
                break;
            case FLAG_RCV:
                if(receiver_buf[j] == 0x01){
                    read_state=A_RCV;
                }
                else if(receiver_buf[j]== FLAG){
                    read_state=FLAG_RCV;                
                }
                else read_state= START;
                break;
            case A_RCV:
                if(receiver_buf[j] == 0x07){
                    read_state = C_RCV;
                }
                else if(receiver_buf[j]== FLAG){
                    read_state=FLAG_RCV;                
                }
                else {read_state= START;}
                break;
            
            case C_RCV:
                if(receiver_buf[j] == (0x01 ^ 0x07)){
                    read_state = BCC_OK;
                }
                else if(receiver_buf[j]== FLAG){
                    read_state=FLAG_RCV;                
                }
                else read_state= START;
                break;
            case BCC_OK:
                if(receiver_buf[j] == FLAG){
                    read_state=STP;
                }
                else read_state= START;
                break;
            case STP:
                ret = 1;
        }
    }
    return ret;
}

int confirm_frame(u_int8_t* receiver_buf, u_int8_t* control){
    enum READ_STATE read_state;
    read_state = START;
    int ret = -1;
    for(int j = 0; j < BUF_SIZE; j++){
        switch(read_state){
            case START:
                if(receiver_buf[j]== FLAG){
                    read_state= FLAG_RCV;
                }
                else read_state=START;
                break;
            case FLAG_RCV:
                if(receiver_buf[j] == 0x01){
                    read_state=A_RCV;
                }
                else if(receiver_buf[j]== FLAG){
                    read_state=FLAG_RCV;                
                }
                else read_state= START;
                break;
            case A_RCV:
                if(receiver_buf[j] == *control){
                    read_state = C_RCV;
                }
                else if(receiver_buf[j]== FLAG){
                    read_state=FLAG_RCV;                
                }
                else read_state= START;
                break;
            case C_RCV:
                if(receiver_buf[j] == (0x01 ^ 0x07)){
                    read_state = BCC_OK;
                }
                else if(receiver_buf[j]== FLAG){
                    read_state=FLAG_RCV;                
                }
                else read_state= START;
                break;
            case BCC_OK:
                if(receiver_buf[j] == FLAG){
                    read_state=STP;
                }
                else read_state= START;
                break;
            case STP:
                ret = 1;
        }
    }
    return ret;
}