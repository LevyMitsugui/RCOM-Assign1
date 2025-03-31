#include "linklayer.h"

// #define DEBUG 
// #define DEBUG_llopen
#define DEBUG_llclose
// #define DEBUG_llwrite
// #define DEBUG_llread
// #define DEBUG_llread2
// #define DEBUG_llwrite
// #define DEBUG_send_frame
// #define DEBUG_setFrame_DATA
// #define DEBUG_array_xor
// #define DEBUG_stuff_bytes
// #define DEBUG_destuff_bytes
// #define DEBUG_llread
#define DEBUG_confirm_frame_control

//TODO code does not discard duplicated frames, it seems to only be able to notice half the duplicate bytes


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
    u_int8_t buf[SUPERV_FRAME_SIZE] = {0};
    u_int8_t incoming_byte = 0;

    READ_STATE SM_llopen = READ;

    int n_bytes = 0;
    int st = 0;
    int STOP = FALSE;

    if (ll.status == TRANSMITTER){ //send SET and wait for UA
        tcflush(fd, TCIOFLUSH);
        setFrame_SET(buf);
        
        //bytes = write(fd, buf, SUPERV_FRAME_SIZE);
        //bytes = send_frame(buf, &incoming_byte, ll.numTransmissions, ll.timeout, fd);
        
        while(alarmCount <= ll.numTransmissions && STOP != TRUE){
            
            if(alarmEnabled == FALSE){
                n_bytes = write(fd, buf, SUPERV_FRAME_SIZE);
                if (n_bytes < 0){
                    printf("Error sending frame\n");
                    return -1;
                }
                alarmEnabled = TRUE;
                alarm(ll.timeout);
            }

            n_bytes = read(fd, &incoming_byte, 1);
            
            #ifdef DEBUG_llopen
            printf("llopen as transmitter, incoming_byte: %02x\n", incoming_byte);
            #endif

            st = confirm_frame_control(&SM_llopen, incoming_byte, CONTROL_UA);
            #ifdef DEBUG_llopen
            printf("llopen as transmitter, st: %d\n", st);
            #endif
            
            if (st == 15){
                #ifdef DEBUG_llopen
                printf("Received UA\n");
                #endif
                STOP = TRUE;
            }
        }
        reset_alarm();

        if(st != 15){
            #ifdef DEBUG_llopen
            printf("Error: llopen as transmitter. No UA received or identified after %d attempts\n", ll.numTransmissions);
            #endif
            return -1;
        }

    } else if (ll.status == RECEIVER){ //wait for SET and send UA
        
        #ifdef DEBUG_llopen
        printf("Waiting for SET\n");
        #endif
        READ_STATE SM_llopen = START;
        //ll.sequenceNumber = 1; // setups the sequence number, so the first iteration is expecting a sequence 0
        
        reset_alarm();
        while(alarmCount == 0){
            if (alarmEnabled == FALSE){ // st because it will reset
                alarmEnabled = TRUE;
                alarm(RECEIVE_TIMEOUT);
            }
            
            n_bytes = read(fd, &incoming_byte, 1);

        
            st = confirm_frame_control(&SM_llopen, incoming_byte, CONTROL_SET);
            #ifdef DEBUG_llopen
            printf("llopen as transmitter, st: %d\n", st);
            #endif
            
            
            if (st == 15){
                #ifdef DEBUG_llopen
                printf("Received UA\n");
                #endif
                break;
            }
        }
        reset_alarm();

        if(st != 15){
            #ifdef DEBUG_llopen
            printf("Error: llopen as receiver failed. Reached timeout at %d seconds. SET frame never identified\n", RECEIVE_TIMEOUT);
            #endif
            return -1;
        }


        setFrame_UA(buf);
        write(fd, buf, SUPERV_FRAME_SIZE);
        

    } else {
        printf("Incorrect program usage\n");
        exit(1);
    }
    #ifdef DEBUG_llopen
    printf("Connected to %s\n", ll.port);
    #endif
    printf("Connected to %s\n", ll.port);//TODO remove
    return fd;
}

//desmontar frame
//byte destuffing
int llread(int fd, u_int8_t* buf, int length){
    (void)signal(SIGALRM, alarmHandler);
    
    int buf_index = 0;
    u_int8_t buf_send[SUPERV_FRAME_SIZE] = {0}; // used to build a response frame 

    u_int8_t incoming_byte = 0;     // byte read from the channel shall be stored here
    int bytes_read = 0;             // bytes read by the read syscall

    int STOP = FALSE;               // flag to stop the while loop
    int should_read = TRUE;         // flag to read the next byte
    READ_STATE state = START;  // state machine for the reading operation

    int frame_length = 0;           // length of the whole frame that was received. This will be incemented through the State Machine (SM)
    int data_length = 0;            // length of the data that was received. This will be incemented through the SM
    
    u_int8_t ctrl_send = 0;         // control byte to be sent in the response frame
    u_int8_t bcc2 = 0;              // BCC2 to be sent in the response frame

    u_int8_t address_received = 0;
    u_int8_t control_received = 0;
    u_int8_t bcc1_received = 0;
    u_int8_t bcc2_received = 0;

    // data bytes will be directly stored in the buffer buf (parameter of the function)
    
    //tcflush(fd, TCIOFLUSH); // flushes the underlying buffer so discard any previous, unwanted data

    alarm(RECEIVE_TIMEOUT);
    alarmEnabled = TRUE;

    while(STOP == FALSE && alarmEnabled == TRUE){
        if(should_read){
            bytes_read = read(fd, &incoming_byte, 1);
            frame_length++;
        }
        if(bytes_read <= 0){continue;
        }
        #ifdef DEBUG_llread2
        printf("\nSeqNum: %d, Current STATE: %d\n", ll.sequenceNumber, state);
        if(should_read){
            printf("incoming byte: %02x\n", incoming_byte);
        }
        
        #endif

        switch(state){
            case START:
                #ifdef DEBUG_llread
                printf("START state\n");
                #endif

                frame_length = 0;
                buf_index = 0;

                if(incoming_byte == FLAG){
                    frame_length = 1;
                    buf_index = 0;
                    state = FLAG_RCV;
                    //alarm(RECEIVE_TIMEOUT);
                } else {
                    state = START;
                }
            break;

            case FLAG_RCV:
                #ifdef DEBUG_llread
                printf("FLAG_RCV state\n");
                #endif

                if(incoming_byte == ADDRESS_RECV){
                    address_received = incoming_byte;
                    state = A_RCV;
                    alarm(RECEIVE_TIMEOUT);
                } else if(incoming_byte == FLAG){
                    state = FLAG_RCV;
                } else {
                    state = START;
                }

            break;

            case A_RCV:
                #ifdef DEBUG_llread
                printf("A_RCV state, incoming_byte: %02x\n", incoming_byte);
                #endif
                
                if(incoming_byte == CONTROL_FRAME_0 && ll.sequenceNumber == 1 ||
                   incoming_byte == CONTROL_FRAME_1 && ll.sequenceNumber == 0){ // Duplicated Byte
                    
                    ctrl_send = (ll.sequenceNumber == 0) ? CONTROL_RR1 : CONTROL_RR0;
                    setFrame_control(buf_send, ctrl_send);
                    write(fd, buf_send, SUPERV_FRAME_SIZE);

                    #ifdef DEBUG_llread2
                    printf("Duplicated Frame\n");
                    #endif

                    state = START;
                    break;
                }

                if(incoming_byte == CONTROL_FRAME_0 || incoming_byte == CONTROL_FRAME_1){
                    state = C_RCV;
                    control_received = incoming_byte;
                    alarm(RECEIVE_TIMEOUT);
                } else if(incoming_byte == CONTROL_SET){ // Retransmission of SET (that means the transmitter did not receive the UA frame)
                    state = SET;
                    control_received == incoming_byte;
                    alarm(RECEIVE_TIMEOUT);
                } else if(incoming_byte == FLAG){
                    state = FLAG_RCV;
                } else {
                    state = START;
                }


            break;

            case C_RCV:
                #ifdef DEBUG_llread
                printf("C_RCV state\n");
                #endif
                #ifdef DEBUG_llread2
                printf("bcc1_received: %02x\n", incoming_byte);
                printf("bcc1 calculated: %02x ^ %02x = %02x\n", address_received, control_received, address_received ^ control_received);
                #endif
                if(incoming_byte == (address_received ^ control_received)){
                    #ifdef DEBUG_llread2
                    printf("bcc1_received: %02x\n", incoming_byte);
                    printf("bcc1 calculated: %02x ^ %02x = %02x == %02x : %d\n", address_received, control_received, address_received ^ control_received, incoming_byte, (incoming_byte == (address_received ^ control_received)));
                    #endif
                    #ifdef DEBUG_llread
                    printf("BCC1 is correct\n");
                    #endif
                    buf_index = 0;
                    state = DATA; // Goes directly to data destuffing and storage becaue BCC1 is correct
                    bcc1_received = incoming_byte; //TODO pode-se apagar isso.
                    alarm(RECEIVE_TIMEOUT);
                
                } else if(incoming_byte == FLAG){
                    state = FLAG_RCV;
                } else {
                    ctrl_send = (control_received == 0) ? CONTROL_RR1 : CONTROL_RR0;
                    setFrame_control(buf_send, ctrl_send);
                    write(fd, buf_send, SUPERV_FRAME_SIZE);

                    state = START;
                }

            break;

            case DATA:
                #ifdef DEBUG_llread
                printf("BCC_OK state\n");
                #endif

                if(incoming_byte == 0x7d){
                    state = DATA_DESTUFF;
                    alarm(RECEIVE_TIMEOUT);
                } else if(incoming_byte == 0x7e){
                    state = BCC2;
                    should_read = FALSE;
                    alarm(RECEIVE_TIMEOUT);
                } else {
                    buf[buf_index] = incoming_byte;
                    buf_index+=1;
                    alarm(RECEIVE_TIMEOUT);

                    #ifdef DEBUG_llread2
                    printf("buf: ");
                    for(int i=0 ; i<buf_index ; i++){
                        printf(" %02x ", buf[i]);
                    }
                    printf("\n");
                    #endif
                }
            break;

            case DATA_DESTUFF:
                #ifdef DEBUG_llread
                printf("DATA_DESTUFF state\n");
                #endif

                if(incoming_byte == 0x5e){
                    buf[buf_index] = 0x7e;
                    buf_index+=1;
                    state = DATA;
                    alarm(RECEIVE_TIMEOUT);

                    #ifdef DEBUG_llread2
                    printf("buf: ");
                    for(int i=0 ; i<buf_index ; i++){
                        printf(" %02x ", buf[i]);
                    }
                    printf("\n");
                    #endif
                } else if(incoming_byte == 0x5d){
                    buf[buf_index] = 0x7d;
                    buf_index+=1;
                    state = DATA;
                    alarm(RECEIVE_TIMEOUT);

                    #ifdef DEBUG_llread2
                    printf("buf: ");
                    for(int i=0 ; i<buf_index ; i++){
                        printf(" %02x ", buf[i]);
                    }
                    printf("\n");
                    #endif
                } else { // SOMETHING WENT WRONG. DROP IT AND REJ

                    ctrl_send = (control_received == CONTROL_FRAME_0) ? CONTROL_REJ0 : CONTROL_REJ1;
                    setFrame_control(buf_send, ctrl_send);
                    write(fd, buf_send, SUPERV_FRAME_SIZE);

                    state = START;
                }
            break;

            case BCC2:
                #ifdef DEBUG_llread
                printf("BCC2 state\n");
                #endif  
                bcc2_received = buf[buf_index-1];
                bcc2 = array_xor(buf, buf_index-1, 0, buf_index-2);

                #ifdef DEBUG_llread2
                printf("bcc2_received: %02x\n", bcc2_received);
                printf("bcc2 calculated: %02x\n", bcc2);
                #endif
                
                if(bcc2 == bcc2_received){
                    state = STP;
                    should_read = FALSE;
                } else {
                    state = START;
                    should_read = TRUE;

                    ctrl_send = (control_received == CONTROL_FRAME_0) ? CONTROL_REJ0 : CONTROL_REJ1;
                    setFrame_control(buf_send, ctrl_send);
                    write(fd, buf_send, SUPERV_FRAME_SIZE);
                }
            break;  

            case STP:
                #ifdef DEBUG_llread
                printf("STOP state\n");
                #endif
                data_length = buf_index-1;
                #ifdef DEBUG_llread2
                printf("buf_index: %d, data_length: %d\n", buf_index, data_length);
                #endif
                buf_index = 0;

                ctrl_send = (control_received == 0x00) ? CONTROL_RR1 : CONTROL_RR0;
                setFrame_control(buf_send, ctrl_send);
                write(fd, buf_send, SUPERV_FRAME_SIZE);
                ll.sequenceNumber = !ll.sequenceNumber;
                state = START;
                STOP = TRUE;
            break;


            case SET:
                if(incoming_byte == address_received ^ control_received){
                    state = ACK;
                    should_read = FALSE;
                    alarm(RECEIVE_TIMEOUT);
                } else if (incoming_byte == FLAG){
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
            break;

            case ACK:
                ctrl_send = CONTROL_UA;
                setFrame_control(buf_send, ctrl_send);
                write(fd, buf_send, SUPERV_FRAME_SIZE);

                state = START;
                should_read = TRUE;
            break;

        }        

        #ifdef DEBUG_llread2
        printf("Next STATE: %d\n", state);
        #endif
    }
    reset_alarm();
    #ifdef DEBUG_llread2
    printf("Exiting llread\n");
    #endif
    return data_length;
}


int llwrite(int fd, u_int8_t* buf, int length){
    (void)signal(SIGALRM, alarmHandler);

    int buf_ret_index = 0;
    u_int8_t buf_retrieve[BUF_SIZE] = {0};// TODO remove later
    u_int8_t buf_send[BUF_SIZE] = {0};// TODO remove later

    u_int8_t ctrl_send = ((ll.sequenceNumber == 0) ? CONTROL_FRAME_0 : CONTROL_FRAME_1);
    printf("ctrl_send: %02x\n", ctrl_send); // TODO remove
    int frame_size = setFrame_DATA(buf_send, buf, length, ctrl_send);
    int bytes_written = 0;
    int bytes_read = 0;
    // int n_bytes = write(fd, buf_send, bytes);
    // if (n_bytes < 0)
    // {
    //     perror("write - linklayer error");
    //     exit(-1);
    // }
    int STOP = FALSE;
    READ_STATE SM_llwrite = START;
    u_int8_t incoming_byte = 0;
    int should_read = TRUE;
    int send_again = FALSE;
    int control_received = 0;
    int address_received = 0;

    while(!STOP && (alarmCount < ll.numTransmissions)){
        if(alarmEnabled == FALSE || send_again == TRUE){
            bytes_written = write(fd, buf_send, frame_size);
            if (bytes_written < 0){ perror("write - linklayer error"); exit(-1);}
            
            #ifdef DEBUG_llwrite
            printf("llwrite, n_bytes %d written\n", bytes_written);
            printf("buf_send: ");
            for(int i = 0; i < bytes_written; i++){
                printf(" %02x ", buf_send[i]);
            } printf("\n");
            #endif

            alarmEnabled = TRUE;
            send_again = FALSE;
            alarm(ll.timeout);
        }
        if (SM_llwrite == START) buf_ret_index = 0; // TODO remove later
        
        #ifdef DEBUG_llwrite
        printf("sequence number: %d, Current STATE: %d\n", ll.sequenceNumber, SM_llwrite);
        #endif
        if(should_read){
            bytes_read = read(fd, &incoming_byte, 1);
            
            buf_retrieve[buf_ret_index] = incoming_byte; // TODO remove later
            buf_ret_index++; // TODO remove later


            #ifdef DEBUG_llwrite
            printf("Incoming byte: %02x\n", incoming_byte);
            printf("Incoming array: ");// TODO remove later

            for(int i = 0; i < buf_ret_index; i++) // TODO remove later
                printf(" %02x ", buf_retrieve[i]);// TODO remove later
            printf("\n");// TODO remove later
            #endif
        }
        if (bytes_read < 0){continue;}


        switch(SM_llwrite){
            case START:
                if(incoming_byte == FLAG){
                    SM_llwrite = FLAG_RCV;
                } else {
                    SM_llwrite = START;
                }
            break;

            case FLAG_RCV:
                if(incoming_byte == ADDRESS_EMIT){
                    address_received = incoming_byte;
                    SM_llwrite = A_RCV;
                } else if(incoming_byte == FLAG){
                    SM_llwrite = FLAG_RCV;
                } else {
                    SM_llwrite = START;
                }
            break;

            case A_RCV:
                if (incoming_byte == CONTROL_RR0 || 
                    incoming_byte == CONTROL_RR1) {

                    control_received = incoming_byte;
                    SM_llwrite = C_RCV;

                } else if (incoming_byte == CONTROL_REJ0 || // receiver rejected frame
                           incoming_byte == CONTROL_REJ1){

                    send_again = TRUE;
                    SM_llwrite = START;

                }else if(incoming_byte == FLAG){
                    SM_llwrite = FLAG_RCV;
                } else {
                    SM_llwrite = START;
                }
            break;

            case C_RCV:
                if(incoming_byte == address_received ^ control_received){
                    SM_llwrite = BCC_OK;
                } else if(incoming_byte == FLAG){
                    SM_llwrite = FLAG_RCV;
                } else {
                    SM_llwrite = START;
                }
            break;

            case BCC_OK:
                if(incoming_byte == FLAG){
                    if(control_received == CONTROL_RR1 && ll.sequenceNumber == 0 ||
                       control_received == CONTROL_RR0 && ll.sequenceNumber == 1){
                        SM_llwrite = STP;
                        should_read = FALSE;
                    } else if (control_received == CONTROL_RR0 && ll.sequenceNumber == 0 ||
                               control_received == CONTROL_RR1 && ll.sequenceNumber == 1){
                        send_again = TRUE;
                        SM_llwrite = START;
                    }
                } else {
                    SM_llwrite = START;
                }
            break;

            case STP:
                ll.sequenceNumber = !ll.sequenceNumber;
                // SM_llwrite = START;
                // should_read = TRUE;
                STOP = TRUE;
            break;
        }
        #ifdef DEBUG_llwrite
        printf("Next STATE: %d\n", SM_llwrite);
        #endif
    }
    reset_alarm();

    // #ifdef DEBUG_llwrite
    // printf("llwrite, n_bytes %d written\n", bytes_written);
    // printf("buf_send: ");
    // for(int i = 0; i < bytes_written; i++){
    //     printf(" %02x ", buf_send[i]);
    // } printf("\n");
    // #endif

    return bytes_written;
}

int llclose(int fd){
    (void)signal(SIGALRM, alarmHandler);
    u_int8_t buf[BUF_SIZE] = {0};
    u_int8_t buf_retrieve[BUF_SIZE] = {0};

    READ_STATE SM_llclose = START;

    int incoming_byte = 0;
    int bytes_read = 0;

    if(ll.status == TRANSMITTER){

        #ifdef DEBUG_llclose
        printf("Run llclose as TRANSMITTER\n");
        #endif
        
        setFrame_DISC(buf);
        while(alarmCount < ll.numTransmissions){
            if(alarmEnabled == FALSE){

                #ifdef DEBUG_llclose
                printf("Sending DISC : ");
                for(int i = 0; i < SUPERV_FRAME_SIZE; i++){
                    printf("%02x ", buf[i]);
                } printf("\n");
                #endif

                write(fd, buf, SUPERV_FRAME_SIZE);
                alarmEnabled = TRUE;
                alarm(ll.timeout);
            }
            bytes_read = read(fd, &incoming_byte, 1);
            if (bytes_read > 0 && confirm_frame_control(&SM_llclose, incoming_byte, CONTROL_DISC) == 1){ // Check if DISC was received correctly
                break;
            }
        }
        reset_alarm();
        if (bytes_read <= 0){
            printf("DISC never received or received incorrectly\n");
            return -1;
        }

        if(confirm_frame_control(&SM_llclose, incoming_byte, CONTROL_DISC) == -1){ // Check if DISC was received correctly
            printf("DISC not received\n");
            return -1; // If not, return -1
        }

        setFrame_UA(buf); // As the DISC frame was received correctly, send the UA frame

        #ifdef DEBUG_llclose
        printf("Sending UA : ");
        for(int i = 0; i < SUPERV_FRAME_SIZE; i++){
            printf("%02x ", buf[i]);
        } printf("\n");
        #endif

        write(fd, buf, BUF_SIZE);
        // Does not check if the UA frame was received correctly

    } else if (ll.status == RECEIVER){

        #ifdef DEBUG_llclose
        printf("Run llclose as RECEIVER\n");
        #endif

        alarmEnabled = TRUE;
        alarm(RECEIVE_TIMEOUT);

        #ifdef DEBUG_llclose
        printf("Waiting for DISC\n");
        #endif

        while(alarmEnabled){ // Wait for DISC  
            bytes_read = read(fd, &incoming_byte, 1);
            if (bytes_read > 0 && confirm_frame_control(&SM_llclose, incoming_byte, CONTROL_DISC) == 1){ // Check if DISC was received correctly
                break;
            }
        }
        reset_alarm();

        if (bytes_read <= 0){
            printf("DISC never received or received incorrectly\n");
            return -1;
        }
        setFrame_control(buf, CONTROL_DISC); // As the DISC frame was received correctly, send the UA frame
        
        while(alarmCount < ll.numTransmissions){
            if(alarmEnabled == FALSE){
                write(fd, buf, SUPERV_FRAME_SIZE);

                #ifdef DEBUG_llclose
                printf("Sending DISC : ");
                for(int i = 0; i < SUPERV_FRAME_SIZE; i++){
                    printf("%02x ", buf[i]);
                } printf("\n");
                #endif

                alarmEnabled = TRUE;
                alarm(ll.timeout);
            }
            bytes_read = read(fd, &incoming_byte, 1);
            if (bytes_read > 0 && confirm_frame_control(&SM_llclose, incoming_byte, CONTROL_UA) == 1){ // Check if DISC was received correctly
                break;
            }
        }
        reset_alarm();
        if (bytes_read <= 0){
            printf("Did not receive UA, quitting eitherway\n");
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

void setFrame_control(u_int8_t* buf, u_int8_t control){
    buf[0] = FLAG;
    buf[1] = (ll.status == TRANSMITTER) ? ADDRESS_RECV : ADDRESS_EMIT;
    buf[2] = control;
    buf[3] = buf[1]^control; 
    buf[4] = FLAG;
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
    buf[1] = (ll.status == TRANSMITTER) ? ADDRESS_RECV : ADDRESS_EMIT;
    buf[2] = CONTROL_UA;
    buf[3] = buf[1]^buf[2]; 
    buf[4] = FLAG;
}

void setFrame_DISC(u_int8_t* buf){
    buf[0] = FLAG;
    buf[1] = (ll.status == TRANSMITTER) ? ADDRESS_RECV : ADDRESS_EMIT;
    buf[2] = CONTROL_DISC;
    buf[3] =(buf[1] ^ buf[2]); 
    buf[4] = FLAG;
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

int confirm_frame_control(READ_STATE* state_machine, int byte, u_int8_t control){
    int prev_state = *state_machine;
    u_int8_t address = (ll.status == TRANSMITTER) ? ADDRESS_EMIT : ADDRESS_RECV;
    #ifdef DEBUG_confirm_frame_control
    printf("DEBUG CONF_FM_CTRL -> SM: %d, byte: %02x\n", *state_machine, byte);
    #endif
    switch(*state_machine){
        case START:
            if(byte == FLAG){
                *state_machine = FLAG_RCV;
            }
            else *state_machine = START;
            break;
        case FLAG_RCV:
            if(byte == address){
                *state_machine=A_RCV;
            }
            else if(byte == FLAG){
                *state_machine=FLAG_RCV;                
            }
            else *state_machine = START;
            break;
        case A_RCV:
            if(byte == control){
                *state_machine = C_RCV;
            }
            else if(byte == FLAG){
                *state_machine=FLAG_RCV;                
            }
            else *state_machine = START;
            break;
        case C_RCV:
            if(byte == (address ^ control)){
                *state_machine = BCC_OK;
            }
            else if(byte == FLAG){
                *state_machine=FLAG_RCV;                
            }
            else *state_machine= START;
            break;
        case BCC_OK:
            if(byte == FLAG){
                *state_machine=STP;
                return 15;
            }
            else *state_machine= START;
            break;
        case STP:
            return 15;
    }

    if(prev_state > *state_machine){
        return prev_state - *state_machine;
    }
    return *state_machine;
}
