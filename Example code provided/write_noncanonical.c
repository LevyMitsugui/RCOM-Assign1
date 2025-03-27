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

#define TRUE 1
#define FALSE 0

#define TRANSMITTER 0
#define RECEIVER 1
#define BAUDRATE B38400
#define RECEIVE_TIMEOUT 60 // in seconds
#define TRANSMIT_TIMEOUT 10
#define MAX_TRANSMISSION_ATTEMPTS 2

#define BUF_SIZE 32
#define HEADER_SIZE 4

#define FLAG            0x7E
#define ADDRESS_RECV    0x03
#define CONTROL_SET     0x03 

#define CONTROL_FRAME_0 0x00
#define CONTROL_FRAME_1 0x40

#define ADDRESS_EMIT  	0x01
#define CONTROL_UA      0x07
#define CONTROL_RR0     0x05
#define CONTROL_RR1     0x85
#define CONTROL_REJ0    0x01
#define CONTROL_REJ1    0x81
#define CONTROL_DISC    0x0B

enum READ_STATE { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, BCC2 , DATA , SEND ,STP};


typedef struct{
    char port[20]; /*Device /dev/ttySx, x = 0, 1*/
    int baudRate; /*Speed of the transmission*/
    int status; /*TRANSMITTER | RECEIVER*/
    unsigned int sequenceNumber; /*Frame sequence number: 0, 1*/
    unsigned int timeout; /*Timer value: 1 s*/
    unsigned int numTransmissions; /*Number of retries in case of
    failure*/
    char frame[BUF_SIZE]; /*Frame*/
} linkLayer;

linkLayer ll;

volatile int STOP = FALSE;

int alarmEnabled = FALSE;
int alarmCount = 0;

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
}

void reset_alarm(void){
    alarm(0);
    alarmEnabled = FALSE;
    alarmCount = 0; 
}

int send_frame(u_int8_t*sender_buf, u_int8_t* receiver_buf, uid_t attempts, uid_t timeout, int fd){
    int bytes_read = 0;

    for(int i = 0; i < BUF_SIZE; i++){
        printf("%02x\n", sender_buf[i]);
    }

    while (alarmCount < attempts){
        if (alarmEnabled == FALSE){
            printf("Sending frame\n");	
            write(fd, sender_buf, BUF_SIZE);
            alarm(timeout);
            alarmEnabled = TRUE;
        }
        bytes_read = read(fd, receiver_buf, BUF_SIZE);

        if(bytes_read != 0)
            break;
    }
    reset_alarm();
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

void setFrame_SET(u_int8_t* buf){
    buf[0] = FLAG;
    buf[1] = ADDRESS_RECV;
    buf[2] = CONTROL_SET;
    buf[3] = buf[1]^buf[2]; 
    buf[4] = FLAG;
} 

void setFrame_UA(u_int8_t* buf){
    buf[0] = FLAG;
    buf[1] = ADDRESS_EMIT;
    buf[2] = CONTROL_UA;
    buf[3] = buf[1]^buf[2]; 
    buf[4] = FLAG;
}

void setFrame_DISC(u_int8_t* buf){
    buf[0] = FLAG;
    buf[1] = ADDRESS_RECV;
    buf[2] = CONTROL_DISC;
    buf[3] =(buf[1] ^ buf[2]); 
    buf[4] = FLAG;
}

void setFrame_MOCK1(u_int8_t* buf){
    buf[0] = FLAG;
    buf[1] = ADDRESS_RECV;
    buf[2] = 0x00;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = 0x11;
    buf[5] = 0x22;
    buf[6] = 0x33;
    buf[7] = 0x44;
    buf[8] = 0x55;
    buf[9] = 0x66;
    buf[10] = array_xor(buf, 11, 4, 9);
    if (buf[10] == 0x7e){
        buf[10] = 0x7d;
        buf[11] = 0x5e;
        buf[12] = FLAG;
    } else if (buf[10] == 0x7d){
        buf[10] = 0x7d;
        buf[11] = 0x5d;
        buf[12] = FLAG;
    } else {
        buf[11] = FLAG;
    }
}

int setFrame_DATA(u_int8_t* buf, u_int8_t* data_packet, int packet_size, u_int8_t control){
    u_int8_t bcc2;
    
    buf[0] = FLAG;
    buf[1] = ADDRESS_RECV;
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

    return 4+packet_size+added_bytes+2;
}

int confirm_frame_control(u_int8_t* receiver_buf, u_int8_t control){

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
                if((receiver_buf[j] == ADDRESS_RECV && control == CONTROL_DISC) || (receiver_buf[j] == ADDRESS_RECV && control == CONTROL_SET) || (receiver_buf[j] == ADDRESS_EMIT && control == CONTROL_UA)){
                    read_state=A_RCV;
                }
                else if(receiver_buf[j]== FLAG){
                    read_state=FLAG_RCV;                
                }
                else read_state= START;
                break;
            case A_RCV:
                if(receiver_buf[j] == control){
                    read_state = C_RCV;
                }
                else if(receiver_buf[j]== FLAG){
                    read_state=FLAG_RCV;                
                }
                else {read_state= START;}
                break;
            
            case C_RCV:
                if(receiver_buf[j] == (receiver_buf[1] ^ receiver_buf[2])){
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

int llwrite(int fd,u_int8_t* buf, int length){
    (void)signal(SIGALRM, alarmHandler);

    u_int8_t buf_send[BUF_SIZE] = {0};
    u_int8_t buf_retrieve[BUF_SIZE] = {0};

    u_int8_t ctrl = (ll.sequenceNumber == 0) ? CONTROL_FRAME_0 : CONTROL_FRAME_1;

    int bytes = setFrame_DATA(buf_send, buf, length, ctrl);
    int bytes_read = 0;
    int attempts = 0;
    uid_t control = 0;

    // do{
    //     bytes_read = send_frame(buf_send, buf_retrieve, ll.numTransmissions, ll.timeout, fd);
    //     attempts++;
    // } while (attempts < ll.numTransmissions && (confirm_frame_control(buf_retrieve, &control) == -1 || bytes_read <= 0));

    while(1){
        bytes_read = send_frame(buf_send, buf_retrieve, ll.numTransmissions, ll.timeout, fd);
        attempts++;
        
        if(bytes_read > 0 && confirm_frame_control(buf_retrieve, &control) == 1){
            if(control == CONTROL_REJ1 && ll.sequenceNumber == 1 || control == CONTROL_REJ0 && ll.sequenceNumber == 0){
                continue;  
            } else if(control == CONTROL_RR0 && ll.sequenceNumber == 1 || control == CONTROL_RR1 && ll.sequenceNumber == 0){
                break;
            } else {
                printf("llwirte, confirm state. Unexpected Control signal. Control: %02x\n", control);
                return -1;
            }
        }
        if(attempts == ll.numTransmissions) return -1;
    }

    ll.sequenceNumber = (ll.sequenceNumber == 0) ? 1 : 0;
    return bytes;
}

int main(int argc, char *argv[])
{
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
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 5;  // Blocking read until 5 chars received

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

    

    // Wait until all bytes have been written to the serial port
    sleep(1);

    u_int8_t buf_a[BUF_SIZE] = {0};
    for(int i = 0; i < BUF_SIZE; i++){
        buf_a[i] = i;
    }

    llwrite(fd, buf_a, BUF_SIZE);

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
