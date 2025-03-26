#include "linklayer.h"

#define DEBUG 


enum READ_STATE { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, BCC2 , DATA , SEND ,STP};

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
    u_int8_t buf_receive[BUF_SIZE] = {0}; // tudo 
    u_int8_t buf_send[BUF_SIZE] = {0};  // montar um frame 
                                        // buf serve para a data 
    u_int8_t buf_aux[BUF_SIZE] = {0};      // buf auxiliar          
    int count=0;
    int siz=0;
    int bytes_read = read(fd, buf_receive, BUF_SIZE);
    if(bytes_read < 0) return -1;
    #ifdef DEBUG
    printf("%s\n", buf);
    #endif

    u_int8_t ctrl_recv = confirm_header(buf_receive);
    if(ctrl_recv == -1) return -1;
    
    for(int j=0; j< BUF_SIZE;j++){
        switch(READ_STATE){
            case START:
                if(buf_receive[j]== FLAG){
                    state= FLAG_RCV;
                    //buf_receive[count]= FLAG;
                    count ++;
                }
                else{
                    state=START;
                    count=0; 
                }
                printf("start\n");
                break;
            case FLAG_RCV:
                if(buf_receive[j]== ADDRESS_SET){
                    state=A_RCV;
                    //buf_receive[count]= ADDRESS_SET ;
                    count++;
                }
                else if(buf_receive[j]== FLAG){
                    state=FLAG_RCV;
                    count=1; //                
                }
                else {
                    state= START;
                    count=0; 
                }
                printf("flag\n");
                break;
            case A_RCV:
                if(buf_receive[j] == 0x00){
                    state = C_RCV;
                   // buf_receive[count]= 0x00 ;
                    count++;
                    control=0;
                  /* if(ll.sequenceNumber==0){
                        ctrl=CONTROL_RR1;
                        state=send;
                   }
                   else if(ll.sequenceNumber==1){
                        ctrl=CONTROL_RR0;
                   }*/
                }
                else if(buf_receive[j] == 0x40){
                    state = C_RCV;
                    //buf_receive[count]= 0x40 ;
                    count++;
                    control=4;
                }
               /* else if(buf_receive[j]== 0x0B){
                    state = C_RCV;
                    //buf_receive[count]= 0x0B ;
                    count++;
                    ctrl= CONTROL_DISC;
                    control=0x0B;
                }*/

                else if(buf_recieve[j]== FLAG){
                    state=FLAG_RCV; 
                    count=0;               
                }
                
                else{
                    state= START;
                    count=0;
                }
                printf("adress\n");
                break;
            case C_RCV: 
                if(buf_receive[j] == (ADDRESS_SET ^ control)){  /// BCC1 
                    //buf_receive[count]= (ADDRESS_SET ^ control) ;
                    count++;                   
                    state = BCC_OK;
                   
                }
                else if(buf_receive[j]== FLAG){
                    state=FLAG_RCV; 
                    count=0;       
                }
                else{              
                    state= START;
                    count=0;
                }
                printf("control\n");
                break;
            case BCC_OK:
                /*
                if it is a duplicate frame (Ns is not the expected) 
                » the data field is discarded 
                » a response RR(expected Ns) is sent 
                
                */
                
                if( control == 0 || control == 4){ // espera de N
                    state= DATA; 

                }
                //else if (control == disc)
                else state= START;
                printf("BCC\n");
                break;
            case DATA:
              
                if (buf_receive[j]== 0x7d && buf_receive[j+1]== 0x5e){
                  //  buf_receive[count]=0x7e;   mudar daqui para baixo 
                    j++;
                    count++;
                   // dat+=1;
                    buf_aux[siz]=0x7e;
                    siz+=1;
                }
                else if (buf[j]== 0x7d && buf[j+1]== 0x5d){
                    buf_receive[count]=0x7d;
                    j++;
                    count++;
                    //dat+=1;
                    buf_aux[siz]=0x7d;
                    siz+=1;
                }
                else {
                    buf_receive[count]=buf[j];
                    count++;
                    //dat+=1;
                    buf_aux[siz]=buf[j];
                    siz+=1;
                }
                if(buf[j]==0x7E){
                    buf_receive[count]=buf[j]; // está a guardar tudo 
                    buf_aux[siz]=buf[j]; // guarda só a data 
                    siz+=1;
                    state= BCC2;
                }
                bcc2=buf_aux[0];
                break;
            case BCC2:  
                for (int h=1;h<siz-2;h++ ){ // pega no segundo de data até ao buf -flag -bcc2 e faz xor e guarda em bcc2
                   bcc2 = (bcc2 ^ buf_aux[h]);    
                   
                }
                if(bcc2==buf_receive[siz-1]){ //caso bcc2 esteja igual ao penultimo elemento do bufferstore avança para stop pois já deu store da flag 
                   
                    if( control== 0){
                        ctrl=CONTROL_RR1;
                        //the data field is passed to the Application layer 
                    }
                    else if( control == 4){
                        ctrl=CONTROL_RR0;
                       //the data field is passed to the Application layer 
                    }
                    state=SEND;
                }
                else {
                   if(control==0){
                        ctrl=CONTROL_REJ0;
                        buf_aux[siz]={0};  // foi rejeitado dá reset a frame e a data 
                        buf_receive[count]={0};
                        siz=0;
                        count=0;
                    }
                    if else(control==4){
                        ctrl=CONTROL_REJ1;
                        buf_aux[siz]={0}; 
                        buf_receive[count]={0};
                        siz=0;
                        count=0;
                    }
                }
                
                break;
            case STP:
                
                printf("stop\n");
                //STOP = TRUE ;
                break;
            case SEND:
                
                //printf("CORRECT\n");
                buf_ua[0] = FLAG;              //
                buf_ua[1] = ADDRESS_UA;           //
                buf_ua[2] = ctrl;           //
                buf_ua[3] = ADDRESS_UA ^ buf_ua[2]; //
                //buf[3] = ADDRESS + TEST;
                buf_ua[4] = FLAG;

                int bytes = write(fd, buf_ua, BUF_SIZE);
                printf("%d bytes written\n", bytes);
                sleep(1);
                state = START;
                break;
        }	
        
       if(STOP==TRUE) break;

    }


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