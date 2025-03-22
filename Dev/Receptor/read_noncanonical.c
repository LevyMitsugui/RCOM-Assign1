
// Read from serial port in non-canonical mode
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
#include <time.h>

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BUF_SIZE    256
#define FLAG        0x7E
#define ADDRESS_UA 	0x01
#define CONTROL_UA     0x07
#define CONTROL_RR0  0x05
#define CONTROL_RR1  0x85
#define CONTROL_REJ0  0x01
#define CONTROL_REJ1  0x81
#define CONTROL_DISC  0x0B
#define ADDRESS_SET  0x03
#define CONTROL_SET 0x03 

volatile int STOP = FALSE;
enum STATE { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, DATA, BCC2, STP, SEND};

int main(int argc, char *argv[])
{
	enum STATE state;
	state= START;


    struct tm* ptr;
    time_t t;

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

    // Open serial port device for reading and writing and not as controlling tty
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

    // Loop for input
    unsigned char buf[BUF_SIZE + 1] = {0}; // +1: Save space for the final '\0' char
    unsigned char buf_ua[BUF_SIZE] = {0};
    unsigned char buf_store[BUF_SIZE]={0};
    int count =0;
    int cntrol=3;
    int dat=0;
    unsigned char bcc2;
//	int j=0;
    while (STOP == FALSE)
    {
        // Returns after 5 chars have been input
        int bytes = read(fd, buf, BUF_SIZE);
        buf[bytes] = '\0'; // Set end of string to '\0', so we can printf

        t = time(NULL);
        ptr = localtime(&t);
        printf("Message received at: %s", asctime(ptr));

        printf(":%s:%d\n", buf, bytes);
        for(int i=0;i< bytes;i++){
            //if(buf[i] == 0x00) break;
		    printf("%02x\n",buf[i]);
            if(buf[i] == 0x7E && i != 0) break;
	    }
        for(int j=0; j<bytes;j++){
            switch(state){
                case START:
                    if(buf[j]== FLAG){
                        state= FLAG_RCV;
                       // buf_store[count]= FLAG;
                        count ++;
                    }
                    else{
                        state=START;
                        count=0; 
                    }
                    printf("start\n");
                    break;
                case FLAG_RCV:
                    if(buf[j]== ADDRESS_SET){
                        state=A_RCV;
                        //buf_store[count]= ADDRESS_SET ;
                        count++;
                    }
                    else if(buf[j]== FLAG){
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
                    if(buf[j]== CONTROL_SET){
                        state = C_RCV;
                        //buf_store[count]= CONTROL_SET ;
                        count++;
                        control=3;
                    }
                    else if(buf[j] == 0x00){
                        state = C_RCV;
                       // buf_store[count]= 0x00 ;
                        count++;
                        control=0;
                        ctrl=CONTROL_UA;
                    }
                    else if(buf[j] == 0x40){
                        state = C_RCV;
                        //buf_store[count]= 0x40 ;
                        count++;
                        control=4;
                    }
                    else if(buf[j]== 0x0B){
                        state = C_RCV;
                       // buf_store[count]= 0x0B ;
                        count++;
                        ctrl= CONTROL_DISC;
                    }

                    else if(buf[j]== FLAG){
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
                    if(buf[j] == (ADDRESS_SET ^ CONTROL_SET)){  /// BCC1 
                                              
                        state = BCC_OK;
                       // buf_store[count]= (ADDRESS_SET ^ CONTROL_SET);
                        count++;
                    }
                    else if(buf[j]== FLAG){
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
                    if( control == 0 || control == 4){
                        state= DATA;           
                    }
                    else state= START;
                    printf("BCC\n");
                    break;
                case DATA:
                   // BUF_STORE COMEÇA AQUI TUDO PARA TRAS ELIMINA E COUNT TAMBEM ELIMINA , MUDAR O TAMANHO DE H E K DO BCC2
                    if (buf[j]== 0x7d && buf[j+1]== 0x5e){
                        buf_store[count]=0x7e;
                        j++;
                        count++;
                        dat+=1;
                    }
                    else {
                        buf_store[count]=buf[j];
                        count++;
                        dat+=1;
                    }
                    if(buf[j]==0x7E){
                        buf_store[count]=buf[j];
                        state= BCC2;
                    }
                    bcc2=buf_store[count-dat-1];
                    break;
                case BCC2:  
                    for (int h=(count-dat);h<(count-2);h++ ){ // pega no segundo de data até ao buf -flag -bcc2 e faz xor e guarda em bcc2
                       bcc2 = (bcc2 ^ buf_store[h])    
                       
                    }
                    if(bcc2==buf_store[count-1]){ //caso bcc2 esteja igual ao penultimo elemento do bufferstore avança para stop pois já deu store da flag 
                       
                        if( control== 0){
                            ctrl=CONTROL_RR1;
                        }
                        else if( control == 4){
                            ctrl=CONTROL_RR0;
                        }
                        state=SEND;
                    }
                    else {
                       if(control==0){
                            ctrl=CONTROL_REJ0;
                            for(int k=(count-2);h>(count-dat);h--){
                                buf_store[k]={"/0"};
                            }
                        }
                        if else(control==4){
                            ctrl=CONTROL_REJ1;
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
        /*if(state == STP){
	        printf("CORRECT\n");
            buf_ua[0] = FLAG;              //
            buf_ua[1] = ADDRESS_UA;           //
            buf_ua[2] = CONTROL_UA;           //
            buf_ua[3] = ADDRESS_UA ^ CONTROL_UA; //
            //buf[3] = ADDRESS + TEST;
            buf_ua[4] = FLAG;

            int bytes = write(fd, buf_ua, BUF_SIZE);
            printf("%d bytes written\n", bytes);
            sleep(1);
            state = START;
        } */

    }

    // The while() cycle should be changed in order to respect the specifications
    // of the protocol indicated in the Lab guide
	//unsigned char buf[BUF_SIZE]={0};
    //	buf[0];	



    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
