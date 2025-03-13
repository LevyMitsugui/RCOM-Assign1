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

#define BUF_SIZE 16 //256

#define FLAG      0x7E
#define ADDRESS   0x03
#define CONTROL   0x03
#define TEST      0x01

volatile int STOP = FALSE;

enum OP_STATE {SET, CONFIRM, DATA, OP_STP};
enum READ_STATE { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK , STP};

int alarmEnabled = FALSE;
int alarmCount = 0;

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
}

u_int8_t byte_xor(u_int8_t byte1, u_int8_t byte2);

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
     * @return int Number of bytes read if a response is received successfully, -1 otherwise.
     */
int send_frame(u_int8_t*sender_buf, u_int8_t* receiver_buf, uid_t attempts, uid_t timeout, int fd);

int stuff_bytes(u_int8_t* data_packet, u_int8_t* buf, uid_t packet_size);

void setFrame_SET(u_int8_t* buf);

void setFrame_DATA(u_int8_t* buf, u_int8_t* data_packet, uid_t packet_size, u_int8_t control);

int confirm_frame(u_int8_t* receiver_buf);

int main(int argc, char *argv[])
{
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
    unsigned char buf_ua[BUF_SIZE + 1] = {0};
    
    u_int8_t mock_data_packet[] = {0x1e, 0x23, 0x7e, 0x5e};
    uid_t mock_packet_size = 4;

    int bytes_read = 0;
    for(int i = 0; i<3; i++){
    switch (macro_state){
    case SET:
        setFrame_SET(buf);
        bytes_read = send_frame(buf, buf_ua, 3, 3, fd);
        if (bytes_read <= 0){
            printf("Did not receive message or message received was not as expected. Terminating code\n");
            //macro_state = OP_STP;
            macro_state = CONFIRM;
        } else {
            macro_state = CONFIRM;
        }
        break;

    case CONFIRM:
        //if (confirm_frame(buf_ua) == 1) macro_state = DATA; //UA frame received
        macro_state = DATA;
        break;

    case DATA:
        printf("we arrived at DATA state\n");
        setFrame_DATA(buf, mock_data_packet, mock_packet_size, 0x00);
        bytes_read = send_frame(buf, buf_ua, 3, 3, fd);
        if (bytes_read <= 0){
            printf("Did not receive message or message received was not as expected. Terminating code\n");
            macro_state = OP_STP;
        } else {
            macro_state = CONFIRM;
        }
        break;

    case OP_STP:
        break;

    default:
        break;
    }}
    

    // DEBUG PRINT
    // printf(":%s:%d\n", buf_ua, bytes_read);
    // for(int i=0;i< bytes_read;i++){
    //     printf("%02x\n",buf_ua[i]);
    //     if(buf_ua[i] == 0x7E && i != 0) break;
    // }

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

u_int8_t array_xor(u_int8_t *array, int arr_size, uid_t init_index, uid_t final_index){
    if(final_index >= arr_size) return 0;
    unsigned char ret_xor = 0;
    for (int i = init_index; i< final_index; i++){
        ret_xor = array[i] = ret_xor;
    }
    return ret_xor;
}

int send_frame(u_int8_t*sender_buf, u_int8_t* receiver_buf, uid_t attempts, uid_t timeout, int fd){

    int bytes = 0;
    int bytes_read = 0;

    while (alarmCount < attempts){
        if (alarmEnabled == FALSE)
        {
            bytes = write(fd, sender_buf, BUF_SIZE);
            alarm(3); // Set alarm to be triggered in 3s
            alarmEnabled = TRUE;
        }
        bytes_read = read(fd, receiver_buf, BUF_SIZE);
        receiver_buf[bytes_read] = '\0'; // Set end of string to '\0', so we can printf

        if(bytes_read != 0){
            alarm(0);
            if (receiver_buf[2] == 0x07)
                if ((receiver_buf[1] ^ receiver_buf[2]) == receiver_buf[3]) {
                    printf("UA received and is Correct\n");
                } else { 
                    printf("Something went wrong");
                    return -1;
                }
            break;
        }
    }
    alarm(0);
    return bytes_read;
}

int stuff_bytes(u_int8_t* data_packet, u_int8_t* buf, uid_t packet_size){
    int n_stuffed_bytes = 0;
    for(int i = 0; i<packet_size; i ++){
        if(data_packet[i] == 0x7e){
            buf[4 + i + n_stuffed_bytes]= 0x7d;
            buf[4 + i + n_stuffed_bytes + 1]= 0x5e;
            n_stuffed_bytes++;
        } else {
            buf[4+ i + n_stuffed_bytes] = data_packet[i];
        }
    }
    return n_stuffed_bytes;
} 

void setFrame_SET(u_int8_t* buf){
    buf[0] = FLAG;
    buf[1] = ADDRESS;
    buf[2] = CONTROL;
    buf[3] = byte_xor(buf[1], buf[2]); 
    buf[4] = FLAG;
} 


void setFrame_DATA(u_int8_t* buf, u_int8_t* data_packet, uid_t packet_size, u_int8_t control){
    buf[0] = FLAG;
    buf[1] = ADDRESS;
    buf[2] = control;
    buf[3] = byte_xor(buf[1], buf[2]); 
    //Assemble Data Packet W/ byte stuffing
    int added_bytes = stuff_bytes(data_packet, buf, packet_size); //number of bytes added by the stuffing function

    
    buf[4+packet_size+added_bytes] = array_xor(data_packet, packet_size, 0, packet_size); //BCC2
    buf[4+packet_size+added_bytes+1] = FLAG;                                              //FLAG
}


int confirm_frame(u_int8_t* receiver_buf){
    enum READ_STATE read_state;
    read_state = START;
    int ret = 0;
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
                if(receiver_buf[j]==0x07){
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