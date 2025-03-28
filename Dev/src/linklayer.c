#include "linklayer.h"

// #define DEBUG 
// #define DEBUG_llopen
// #define DEBUG_llclose
// #define DEBUG_send_frame
//#define DEBUG_setFrame_DATA
//#define DEBUG_array_xor
//#define DEBUG_stuff_bytes
//#define DEBUG_destuff_bytes
//#define DEBUG_llread
// #define DEBUG_llwrite


enum READ_STATE { START, FLAG_RCV, A_RCV, C_RCV, C_FRAME_0, C_FRAME_1, BCC_OK, BCC2 , DATA , SEND ,STP};
enum llread_state {READ, CONF_HEADER, DESTUFF, CONF_DATA, RESPOND, FINISH_READ};
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
    newtio.c_cc[VMIN] =  0;  
    newtio.c_cc[VTIME] = 1;
    
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
        
        #ifdef DEBUG_llopen
        printf("Response from receiver:\n");        
        for (int i = 0; i < bytes; i++){
            printf("%02x\n", buf_receive[i]);
            if(i != 0 && buf_receive[i] == 0x7e) break;
        }
        printf("End of response from receiver\n");        
        #endif

        if (bytes <= 0){ 
            #ifdef DEBUG_llopen 
            printf("No response from receiver\n"); 
            #endif
            return -1;
        }

        if(confirm_frame_control(buf_receive, CONTROL_UA) != 1){
            #ifdef DEBUG_llopen 
            printf("Invalid frame\n"); 
            #endif
            return -1;
        }

        #ifdef DEBUG_llopen
        printf("Valid frame\n");
        #endif

    } else if (ll.status == RECEIVER){ //wait for SET and send UA
        
        #ifdef DEBUG_llopen
        printf("Waiting for SET\n");
        #endif
        
        ll.sequenceNumber = 1; // setups the sequence number, so the first iteration is expecting a sequence 0
        reset_alarm();
        while(alarmCount == 0){
            if (alarmEnabled == FALSE){
                alarmEnabled = TRUE;
                alarm(RECEIVE_TIMEOUT);
            }
            bytes_read = read(fd, buf_receive, BUF_SIZE);
            if (bytes_read > 0){
                #ifdef DEBUG_llopen
                printf("Incoming from transmitter\n");
                for (int i = 0; i < bytes_read; i++){
                    printf("%02x\n", buf_receive[i]);
                    if(i != 0 && buf_receive[i] == 0x7e) break;
                }
                printf("End of incoming from transmitter\n");
                #endif
                break;
            }
        }
        reset_alarm();

        if (bytes_read <= 0){
            #ifdef DEBUG_llopen 
            printf("No response from transmitter\n"); 
            #endif
            return -1;
        }

        if(confirm_frame_control(buf_receive, CONTROL_SET) != 1){
            #ifdef DEBUG_llopen 
            printf("Invalid frame\n"); 
            #endif
            return -1;
        } else {
            setFrame_UA(buf);

            #ifdef DEBUG_llopen 
            printf("Valid frame, response to transmitter:\n"); 
            for (int i = 0; i < BUF_SIZE; i++){
                printf("%02x\n", buf[i]);
                if(i != 0 && buf[i] == 0x7e) break;
            }
            printf("End of response to transmitter\n");
            #endif

            write(fd, buf, BUF_SIZE);
            
            #ifdef DEBUG_llopen 
            printf("Frame sent\n"); 
            #endif
        }

    } else {
        printf("Incorrect program usage\n");
        exit(1);
    }
    #ifdef DEBUG_llopen
    printf("Connected to %s\n", ll.port);
    #endif
    return fd;
}

//desmontar frame
//byte destuffing
int llread(int fd, u_int8_t* buf, int length){
    (void)signal(SIGALRM, alarmHandler);
    u_int8_t buf_receive = 0; // tudo 
    u_int8_t buf_send[BUF_SIZE] = {0};  // montar um frame 
    u_int8_t buf_copy[BUF_SIZE] = {0};                                    // buf serve para a data 
    u_int8_t buf_data[BUF_SIZE] = {0};      // buf auxiliar 
    
    int control=0;  
    int ctrl = 0;       
    int count=0;
    int siz=0;
    int bytes_read = 0;
    enum READ_STATE state = START;

    int STOP = FALSE;

    u_int8_t ctrl_recv = confirm_header(buf_receive);
    if(ctrl_recv == -1) return -1;
    
    while(!STOP){
        bytes_read = read(fd, buf_receive, BUF_SIZE);
        switch(state){
            case START:
                if(buf_receive == FLAG){
                    state= FLAG_RCV;
                    buf_copy[count]= FLAG;
                    count ++;
                }
                else{
                    state=START;
                    count=0; 
                }
                printf("start\n");
                break;
            case FLAG_RCV:
                if(buf_receive == ADDRESS_RECV){
                    state=A_RCV;
                    buf_copy[count]= ADDRESS_RECV ;
                    count++;
                }
                else if(buf_receive == FLAG){
                    state=FLAG_RCV;
                    count=0;                
                }
                else {
                    state= START;
                    count=0; 
                }
                printf("flag\n");
                break;
            case A_RCV:
                if(buf_receive == 0x00){
                    state = C_RCV;
                    buf_copy[count]= 0x00 ;
                    count++;
                    control=0;
                }
                else if(buf_receive == 0x40){
                    state = C_RCV;
                    buf_copy[count]= 0x40 ;
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

                else if(buf_receive== FLAG){
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
                if(buf_receive == (ADDRESS_RECV ^ control)){  /// BCC1 
                    buf_copy[count]= (ADDRESS_RECV ^ control) ;
                    count++;                   
                    state = BCC_OK;
                   
                }
                else if(buf_receive== FLAG){
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
                
                if(ll.sequenceNumber==control){
                    if(control==0){
                        ctrl=CONTROL_RR1;
                        buf_copy[count]={0};
                        count=0;
                        state = SEND;
                    }
                    else if(control==4){
                        ctrl=CONTROL_RR0;
                        buf_copy[count]={0};
                        count=0;
                        state = SEND;
                    }
                }
        
                
                else { // espera de N
                    if(control==0 || control==4){
                        state= DATA; 
                    }
                    else state= START;
                    
                }
                
                
                printf("BCC\n");
                break;
            case DATA:
              
                if (buf_receive == 0x7d && buf_[j+1]== 0x5e){
                    buf_copy[count]=0x7e;   mudar daqui para baixo 
                    j++;
                    count++;
                   // dat+=1;
                    buf_data[siz]=0x7e; //buf_data 
                    siz+=1;
                }
                else if (buf_receive[j]== 0x7d && buf_receive[j+1]== 0x5d){
                    buf_copy[count]=0x7d;
                    j++;
                    count++;
                    //dat+=1;
                    buf_data[siz]=0x7d;
                    siz+=1;
                }
                else {
                    buf_copy[count]=buf_receive[j];
                    count++;
                    //dat+=1;
                    buf_data[siz]=buf_receive[j];
                    siz+=1;
                }
                if(buf[j]==0x7E){
                    buf_copy[count]=buf_receive[j]; // está a guardar tudo 
                    buf_data[siz]=buf_receive[j]; // guarda só a data 
                    siz+=1;
                    state= BCC2;
                }
                bcc2=buf_data[0];
                break;
            case BCC2:  
                for (int h=1;h<siz-2;h++ ){ // pega no segundo de data até ao buf -flag -bcc2 e faz xor e guarda em bcc2
                   bcc2 = (bcc2 ^ buf_data[h]);    
                   
                }
                if(bcc2==buf_copy[siz-1]){ //caso bcc2 esteja igual ao penultimo elemento do bufferstore avança para stop pois já deu store da flag 
                   
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
                        buf_data[siz]={0};  // foi rejeitado dá reset a frame e a data 
                        buf_copy[count]={0};
                        siz=0;
                        count=0;
                    }
                    if else(control==4){
                        ctrl=CONTROL_REJ1;
                        buf_data[siz]={0}; 
                        buf_copy[count]={0};
                        siz=0;
                        count=0;
                    }
                }
                
                break;
            case STP:
                
                printf("stop\n");
                STOP = TRUE ;
                break;
            case SEND:
                
                //printf("CORRECT\n");
                buf_send[0] = FLAG;              //
                buf_send[1] = ADDRESS_RECV;           //
                buf_send[2] = ctrl;           //
                buf_send[3] = ADDRESS_EMIT ^ buf_send[2]; //
                //buf[3] = ADDRESS + TEST;
                buf_send[4] = FLAG;

                int bytes = write(fd, buf_send, BUF_SIZE);
                printf("%d bytes written\n", bytes);
                sleep(1);
                state = START;
                break;
        }	
        
       if(STOP==TRUE) break;

    }


    return bytes_read;
}


int llwrite(int fd, u_int8_t* buf, int length){
    (void)signal(SIGALRM, alarmHandler);

    u_int8_t buf_send[BUF_SIZE] = {0};
    u_int8_t buf_retrieve[BUF_SIZE] = {0};

    int bytes = setFrame_DATA(buf_send, buf, length, ll.sequenceNumber);
    
    int bytes_read = 0;
    int attempts = 0;
    u_int8_t control = 0;

    // do{
    //     bytes_read = send_frame(buf_send, buf_retrieve, ll.numTransmissions, ll.timeout, fd);
    //     attempts++;
    // } while (attempts < ll.numTransmissions && (confirm_frame_control(buf_retrieve, &control) == -1 || bytes_read <= 0));

    //while(1){
        bytes_read = send_frame(buf_send, buf_retrieve, ll.numTransmissions, ll.timeout, fd);
        attempts++;
        
        #ifdef DEBUG_llwrite
        printf("llwrite, bytes_read: %d\n", bytes_read);
        for(int i = 0; i < 5; i++){
            printf("buf_retrieve[%d]: %02x\n", i, buf_retrieve[i]);
        }
        #endif

        if(bytes_read > 0 && confirm_frame(buf_retrieve, &control) == 1){

            #ifdef DEBUG_llwrite
            printf("llwrite, confirm state. Control received from receiver: %02x\n", control);
            #endif

            if(control == CONTROL_REJ1 && ll.sequenceNumber == 1 || control == CONTROL_REJ0 && ll.sequenceNumber == 0){
                //continue;  
            } else if(control == CONTROL_RR0 && ll.sequenceNumber == 1 || control == CONTROL_RR1 && ll.sequenceNumber == 0){
                //break;
            } else {
                printf("llwirte, confirm state. Unexpected Control signal. Control: %02x\n", control);
                return -1;
            }
        }
        if(attempts == ll.numTransmissions) return -1;
    //}

    ll.sequenceNumber = (ll.sequenceNumber == 0) ? 1 : 0;
    return bytes;
}

 int llwrite_test(int fd, u_int8_t* buf, int length){
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

        #ifdef DEBUG_llclose
        printf("Run llclose as TRANSMITTER\n");
        #endif
        
        setFrame_DISC(buf);
        send_frame(buf, buf_retrieve, ll.numTransmissions, ll.timeout, fd); // Send DISC frame and wait for DISC
        
        if(confirm_frame_control(buf_retrieve, CONTROL_DISC) == -1){ // Check if DISC was received correctly
            printf("DISC not received\n");
            return -1; // If not, return -1
        }

        setFrame_UA(buf); // As the DISC frame was received correctly, send the UA frame
        write(fd, buf, BUF_SIZE);
        // Does not check if the UA frame was received correctly

    } else if (ll.status == RECEIVER){
        reset_alarm();
        while(alarmCount < ll.numTransmissions){ // Wait for DISC
            
            #ifdef DEBUG
            if (alarmCount == 0){
                printf("Waiting for DISC\n");
            }
            #endif
            
            
            if (alarmEnabled == FALSE){
                alarmEnabled = TRUE;
                alarm(RECEIVE_TIMEOUT);
            }
            bytes_read = read(fd, buf, BUF_SIZE);
            if (bytes_read > 0 && confirm_frame_control(buf, CONTROL_DISC) == 1){ // Check if DISC was received correctly
                break;
            }
        }
        reset_alarm();

        if (bytes_read <= 0){
            printf("DISC never received or received incorrectly\n");
            return -1;
        }

        setFrame_DISC(buf);
        bytes_read = send_frame(buf, buf_retrieve, ll.numTransmissions, ll.timeout, fd);
    
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

    #ifdef DEBUG_llclose
    printf("Disconnected from %s\n", ll.port);
    #endif

    return 0;
}


int send_frame(u_int8_t*sender_buf, u_int8_t* receiver_buf, uid_t attempts, uid_t timeout, int fd){
    int bytes_read = 0;

    #ifdef DEBUG_send_frame
    printf("DEBUG: send_frame beginning\n");
    for(int i = 0; i < BUF_SIZE; i++){
        printf("%02x\n", sender_buf[i]);
        if(i != 0 && sender_buf[i] == 0x7e) break;
    }
    #endif

    while (alarmCount < attempts){

        #ifdef DEBUG_send_frame
        printf("alarmCount: %d", alarmCount);
        printf("  attempts: %d\n", attempts);
        #endif

        if (alarmEnabled == FALSE){
            #ifdef DEBUG_send_frame
            if(alarmCount == 0){
                printf("Sending frame\n");
            }
            #endif
            
            write(fd, sender_buf, BUF_SIZE);
            alarm(timeout);
            alarmEnabled = TRUE;
        }
        
        #ifdef DEBUG_send_frame
        printf("reading\n");
        #endif

        bytes_read = read(fd, receiver_buf, BUF_SIZE);

        #ifdef DEBUG_send_frame
        printf("bytes_read: %d\n", bytes_read);
        #endif

        if(bytes_read != 0)
            break;
    }
    reset_alarm();
    return bytes_read;
}

int destuff_bytes(u_int8_t* orig, u_int8_t* target, uid_t init_index, uid_t final_index){
    if (final_index >= BUF_SIZE){
        printf("destuff_bytes final_index >= BUF_SIZE\n");
        return -1;
    }
    if(init_index > final_index){
        printf("destuff_bytes init_index > final_index\n");
        return-1;
    }
    
    int new_length = final_index - init_index-1;
    int target_index = 0;

    for(int i = init_index; i <= final_index; i ++){
        if (orig[i] == 0x7d && orig[i+1] == 0x5e){
            target[target_index] = 0x7e;
            #ifdef DEBUG_destuff_bytes
            printf("orig[%d]: %02x, orig[%d]: %02x", i, orig[i], i+1, orig[i+1]);
            printf("  target[%d]: %02x\n", target_index, target[target_index]);
            #endif
            i+=1;
            new_length-=1;
        } else if (orig[i] == 0x7d && orig[i+1] == 0x5d){
            target[target_index] = 0x7d;
            #ifdef DEBUG_destuff_bytes
            printf("orig[%d]: %02x, orig[%d]: %02x", i, orig[i], i+1, orig[i+1]);
            printf("  target[%d]: %02x\n", target_index, target[target_index]);
            #endif
            i+=1;
            new_length-=1;
        } else {
            target[target_index] = orig[i];
            #ifdef DEBUG_destuff_bytes
            printf("orig[%d]: %02x", i, orig[i]);
            printf("  target[%d]: %02x\n", target_index, target[target_index]);
            #endif
        }

        target_index+=1;
    }

    #ifdef DEBUG_destuff_bytes
    printf("DEBUG destuff_bytes, new_length: %d\n", ++new_length);

    printf("DEBUG destuff_bytes output:\n");
    for(int i=0;i< BUF_SIZE;i++){
        printf("%02x\n",target[i]);
    }
    printf("end of destuff\n");
    #endif

    return new_length;
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

    #ifdef DEBUG_stuff_bytes
    printf("DEBUG stuff_bytes output:\n");
    for(int i=0;i< BUF_SIZE;i++){
        printf("%02x\n",buf[i]);
        if(buf[i] == 0x7E && i != 0) break;
    }
    #endif
    return n_stuffed_bytes;
}

u_int8_t array_xor(u_int8_t* array, int arr_size, uid_t init_index, uid_t final_index){
    #ifdef DEBUG_array_xor
    printf("DEBUG: array_xor\n");
    #endif

    //printf("final_index: %d, BUF_SIZE: %d\n", final_index, BUF_SIZE); // TODO remove this later
    if(final_index >= BUF_SIZE){
        printf("array_xor: Index out of bounds\n"); return 0;
    }

    unsigned char ret_xor = 0;
    for (int i = init_index; i<=final_index; i++){
        ret_xor = array[i] ^ ret_xor;
        
        #ifdef DEBUG_array_xor
        printf("ret_xor[%d]: %02x\n", i, ret_xor);
        #endif
    }

    #ifdef DEBUG_array_xor
    printf("ret_xor final value: %02x\n", ret_xor);
    #endif
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

void setFrame_SUP(u_int8_t* buf, u_int8_t control){
    buf[0] = FLAG;
    buf[1] = ADDRESS_EMIT;
    buf[2] = control;
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

int setFrame_DATA(u_int8_t* buf, u_int8_t* data_packet, uid_t packet_size, u_int8_t control){
    //printf("KASHGDFAJHSD: %d", packet_size);
    
    u_int8_t bcc2;
    
    buf[0] = FLAG;
    buf[1] = ADDRESS_RECV;
    buf[2] = control;
    buf[3] = buf[1] ^ buf[2]; 
    //Assemble Data Packet W/ byte stuffing
    int added_bytes = stuff_bytes(data_packet, buf, packet_size, 4); //number of bytes added by the stuffing function
    
    #ifdef DEBUG_setFrame_DATA
    printf("setFrame_DATA, added_bytes: %d\n", added_bytes);
    #endif

    bcc2 = array_xor(data_packet, packet_size, 0, packet_size-1); //BCC2
    
    if(bcc2 == 0x7e){
        buf[4+packet_size+added_bytes] = 0x7d;
        added_bytes+=1;
        buf[4+packet_size+added_bytes] = 0x5e;
    } else if(bcc2 == 0x7d){
        buf[4+packet_size+added_bytes] = 0x7d;
        added_bytes+=1;
        buf[4+packet_size+added_bytes] = 0x5d;
    } else {
        buf[4+packet_size+added_bytes] = bcc2; //BCC2
    }
    buf[4+packet_size+added_bytes+1] = FLAG;                                              //FLAG
    #ifdef DEBUG_setFrame_DATA
    printf("DEBUG setFrame_DATA, assembled frame:\n");
    for(int i=0; i< 4+packet_size+added_bytes+2; i++){
        printf("buf[%d] = %02x\n", i, buf[i]);
    }
    #endif

    return 4+packet_size+added_bytes+2;
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