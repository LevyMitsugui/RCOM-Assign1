
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
#define ADDRESS_SET  0x03
#define CONTROL_SET 0x03 
volatile int STOP = FALSE;
enum STATE { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK , STP};
int main(int argc, char *argv[])
{
	enum STATE state;
	state= START;
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
//	int j=0;
    while (STOP == FALSE)
    {
        // Returns after 5 chars have been input
        int bytes = read(fd, buf, BUF_SIZE);
        buf[bytes] = '\0'; // Set end of string to '\0', so we can printf

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
				}
                else state=START;
				printf("start\n");
				break;
			case FLAG_RCV:
				if(buf[j]== ADDRESS_SET){
					state=A_RCV;
				}
                else if(buf[j]== FLAG){
                    state=FLAG_RCV;                
                }
                else state= START;
				printf("flag\n");
				break;
			case A_RCV:
				if(buf[j]== CONTROL_SET){
					state = C_RCV;
				}
                else if(buf[j]== FLAG){
                    state=FLAG_RCV;                
                }
                else state= START;
				printf("adress\n");
				break;
			case C_RCV:
				if(buf[j] == (ADDRESS_SET ^ CONTROL_SET)){
					state = BCC_OK;
				}
                else if(buf[j]== FLAG){
                    state=FLAG_RCV;                
                }
                else state= START;
				printf("control\n");
				break;
			case BCC_OK:
				if(buf[j] == FLAG){
					state=STP;
				}
                else state= START;
 				printf("BCC\n");
				break;
			case STP:
				printf("stop\n");
				STOP = TRUE ;
		       		break;

		}	
		if(STOP==TRUE) break;

	}
        if((buf[1] ^ buf[2]) == buf[3]){
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
        } else {
            break;
        }
//	j++;
       // STOP= TRUE ;
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
