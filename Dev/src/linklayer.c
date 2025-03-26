#include "linklayer.h"

#define DEBUG 


enum READ_STATE { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK , STP};

//typedef struct linkLayer linkLayer;
//linkLayer ll;

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

linkLayer create_link_layer(const char *port, int baudRate, uid_t timeout, uid_t numTransmissions){
    linkLayer ll;
    strncpy(ll.port, port, 20);
    ll.status = -1;
    ll.baudRate = BAUDRATE;
    ll.timeout = timeout;
    ll.numTransmissions = numTransmissions;

    ll.sequenceNumber = 0;
    return ll;
}

int llopen(const char *port, int role){
    (void)signal(SIGALRM, alarmHandler);

    strncpy(ll.port, port, 20);
    int fd = open(port, O_RDWR | O_NOCTTY);
    ll.status = role;

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

    newtio.c_cflag = ll.baudRate | CS8 | CLOCAL | CREAD;
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
    u_int8_t buf_receive[BUF_SIZE] = {0};

    if (ll.status == TRANSMITTER){ //send SET and wait for UA
        setFrame_SET(buf);
        int bytes = 0;
        
        bytes = send_frame(buf, buf_receive, ll.numTransmissions, ll.timeout, fd);
        printf("%d bytes read\n", bytes);
        
        if (bytes <= 0) return -1;

        if(confirm_frame_control(buf_receive, CONTROL_UA) != 1) return -1;
            

    } else if (ll.status == RECEIVER){ //wait for SET and send UA

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
    (void)signal(SIGALRM, alarmHandler);
    u_int8_t buf_receive[BUF_SIZE] = {0};
    u_int8_t buf_send[BUF_SIZE] = {0};

    int bytes_read = read(fd, buf_receive, BUF_SIZE);
    if(bytes_read < 0) return -1;
    #ifdef DEBUG
    printf("%s\n", buf);
    #endif

    u_int8_t ctrl_recv = confirm_header(buf_receive);
    if(ctrl_recv == -1) return -1;
    
    


    return bytes_read;
}

 int llwrite(int fd, const u_int8_t* buf, int length){
    (void)signal(SIGALRM, alarmHandler);

    int bytes = write(fd, buf, BUF_SIZE);
    if (bytes < 0)
    {
        perror("write");
        exit(-1);
    }
    
    return bytes;
}

int llclose(int fd){
    (void)signal(SIGALRM, alarmHandler);
    u_int8_t buf[BUF_SIZE] = {0};
    u_int8_t buf_retrieve[BUF_SIZE] = {0};
    int bytes_read = 0;

    if(ll.status == TRANSMITTER){
        setFrame_DISC(buf);

        send_frame(buf, buf_retrieve, ll.numTransmissions, ll.timeout, fd);
        
        if(confirm_frame_control(buf_retrieve, CONTROL_DISC) == -1){
            printf("DISC not received\n");
            return -1;
        }

        setFrame_UA(buf);
        send_frame(buf, buf_retrieve, ll.numTransmissions, ll.timeout, fd);
    } else if (ll.status == RECEIVER){
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
        if (bytes_read <= 0){
            printf("DISC never received\n");
            return -1;
        }
    }

    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
    printf("Disconnected from %s\n", ll.port);
    return 0;
}


int send_frame(u_int8_t*sender_buf, u_int8_t* receiver_buf, uid_t attempts, uid_t timeout, int fd){
    int bytes_read = 0;

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
    printf("alarmEnalbled: %d\n", alarmEnabled);
    printf("alarmCount: %d\n", alarmCount);
    return bytes_read;
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

int confirm_header(u_int8_t* receiver_buf){
    enum READ_STATE read_state = START;

    for(int j = 0; j < HEADER_SIZE; j++){
        switch(read_state){
            case START:
                if(receiver_buf[j]== FLAG){
                    read_state= FLAG_RCV;
                }
                else read_state=START;
                break;
            case FLAG_RCV:
                if(receiver_buf[j] == ADDRESS_RECV){
                    read_state=BCC_OK;
                }
                else if(receiver_buf[j]== FLAG){
                    read_state=FLAG_RCV;                
                }
                else read_state= START;
                break;
            case BCC_OK:
                j+=1;
                if(receiver_buf[j] == (receiver_buf[1] ^ receiver_buf[2])){
                    return receiver_buf[2];
                }
            
        }
    }
    return -1;
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