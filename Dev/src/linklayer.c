#include "linklayer.h"

#define DEBUG 


enum READ_STATE { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK , STP};

//typedef struct linkLayer linkLayer;
linkLayer ll;
struct termios oldtio;
struct termios newtio;

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

int llopen(const char *port, int role){

    strncpy(ll.port, port, 20);
    int fd = open(port, O_RDWR | O_NOCTTY);
    int status = role;

    if (fd < 0)
    {
        perror(ll.port);
        exit(-1);
    }

    //struct termios oldtio;
    //struct termios newtio;

    if (tcgetattr(fd, &oldtio) == -1)
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

    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }
    //--- --- --- --- --- --- ---

    alarmCount = 0;
    int bytes_read = 0;
    u_int8_t buf[BUF_SIZE] = {0};

    if (status == TRANSMITTER){ //send SET and wait for UA
        setFrame_SET(buf);
        int bytes = 0;
        
        while(alarmCount < MAX_TRANSMISSION_ATTEMPTS){
            if (alarmEnabled == FALSE){
                bytes = write(fd, buf, BUF_SIZE);
                alarmEnabled = TRUE;
                alarm(TRANSMIT_TIMEOUT);
            }
            bytes = read(fd, buf, BUF_SIZE);
            if (bytes > 0){
                break;
            }
        }
        reset_alarm();

        if (bytes <= 0) return -1;

        if(confirm_frame_control(buf, CONTROL_UA) != 1) return -1;
            

    } else if (status == RECEIVER){ //wait for SET and send UA

        while(alarmCount == 0){
            if (alarmEnabled == FALSE){
                alarmEnabled = TRUE;
                alarm(RECEIVE_TIMEOUT);
            }
            bytes_read = llread(fd, buf, BUF_SIZE);
            if (bytes_read > 0){
                break;
            }
        }
        reset_alarm();

        if (bytes_read <= 0) return -1;

        if(confirm_frame_control(buf, CONTROL_SET) != 1){
            return -1;
        } else {
            setFrame_UA(buf);
            llwrite(fd, buf, BUF_SIZE);
        }

    } else {
        printf("Incorrect program usage\n");
        exit(1);
    }
    printf("Connected to %s\n", ll.port);
    return fd;
}

int llread(int fd, u_int8_t* buf, int length){
    int bytes_read = read(fd, buf, BUF_SIZE);
    #ifdef DEBUG
    printf("%s\n", buf);
    #endif
    return bytes_read;
}

 int llwrite(int fd, const u_int8_t* buf, int length){
    int bytes = write(fd, buf, BUF_SIZE);
    printf("%d bytes written\n", bytes);
    if (bytes < 0)
    {
        perror("write");
        exit(-1);
    }
    
    return bytes;
}

int llclose(int fd){
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
    return 0;
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
                if((receiver_buf[j] == ADDRESS_RECV && control == CONTROL_SET) || (receiver_buf[j] == ADDRESS_EMIT && control == CONTROL_UA)){
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