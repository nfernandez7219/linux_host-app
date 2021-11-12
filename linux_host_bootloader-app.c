#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
//#include <termios.h>
//#include "Bootloader.h"
 
#define BL_CMD_UNLOCK			0xa0
#define BL_CMD_DATA				0xa1
#define BL_CMD_VERIFY			0xa2
#define BL_CMD_RESET			0xa3
#define BL_CMD_BNKSWAP_RESET	0xa4

#define BL_RESP_OK				0x50
#define BL_RESP_ERROR			0x51
#define BL_RESP_INVALID			0x52
#define BL_RESP_CRC_OK			0x53
#define BL_RESP_CRC_FAIL		0x54

#define BL_GUARD				0x5048434D
#define ERASE_SIZE				256
#define BOOTLOADER_SIZE			2048

#define MAX_FLASH_SIZE          (1*1024*1024)   /* 1 MB */
#define MAX_BOOTLOADER_SIZE     (8*1024)        /* 8Kb */
#define MAX_BIN                 ((MAX_FLASH_SIZE - MAX_BOOTLOADER_SIZE) >> 1)

//devices = {"SAME7X" : [8192, 8192]}

int			open_serial_port(const char *device, uint32_t baudrate);
int			write_port(int fd, uint8_t *buffer, size_t size);
ssize_t		read_port(int fd, uint8_t *buffer, size_t size);
void		crc32_tab_gen();
uint32_t	crc_32(int *data);
uint32_t	check_option(int opt);

static char binmap[MAX_BIN];
uint32_t	crc_table[256];


// Opens the specified serial port, sets it up for binary communication,
// configures its read timeouts, and sets its baud rate.
// Returns a non-negative file descriptor on success, or -1 on failure.
int open_serial_port(const char * device, uint32_t baud_rate)
{
    struct termios options;

    int fd = open(device, O_RDWR | O_NOCTTY);

    if (fd == -1)
    {
        perror(device);
        return -1;
    }
 
    // Flush away any bytes previously read or written.
    int result = tcflush(fd, TCIOFLUSH);
    if (result)
    {
        perror("tcflush failed");  // just a warning, not a fatal error
    }

    // Get the current configuration of the serial port.
    result = tcgetattr(fd, &options);
    if (result)
    {
        perror("tcgetattr failed");
        close(fd);
        return -1;
    }

    // Turn off any options that might interfere with our ability to send and
    // receive raw binary bytes.
    options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    options.c_oflag &= ~(ONLCR | OCRNL);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // Set up timeouts: Calls to read() will return as soon as there is
    // at least one byte available or when 100 ms has passed.
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;

    // This code only supports certain standard baud rates. Supporting
    // non-standard baud rates should be possible but takes more work.
    switch (baud_rate)
    {
        case 4800:   cfsetospeed(&options, B4800);   break;
        case 9600:   cfsetospeed(&options, B9600);   break;
        case 19200:  cfsetospeed(&options, B19200);  break;
        case 38400:  cfsetospeed(&options, B38400);  break;
        case 115200: cfsetospeed(&options, B115200); break;
        default:
            fprintf(stderr,
                    "warning: baud rate %u is not supported, using 9600.\n",
                    baud_rate);
                    cfsetospeed(&options, B9600);
            break;
    }

    cfsetispeed(&options, cfgetospeed(&options));

    result = tcsetattr(fd, TCSANOW, &options);

    if (result)
    {
        perror("tcsetattr failed");
        close(fd);
        return -1;
    }

    return fd;
}
 
// Writes bytes to the serial port, returning 0 on success and -1 on failure.
int write_port(int fd, uint8_t * buffer, size_t size)
{
    ssize_t result = write(fd, buffer, size);
    if (result != (ssize_t)size)
    {
        perror("failed to write to port");
        return -1;
    }

    return 0;
}
 
// Reads bytes from the serial port.
// Returns after all the desired bytes have been read, or if there is a
// timeout or other error.
// Returns the number of bytes successfully read into the buffer, or -1 if
// there was an error reading.
ssize_t read_port(int fd, uint8_t * buffer, size_t size)
{
    size_t received = 0;
    while (received < size)
    {
        ssize_t r = read(fd, buffer + received, size - received);
        if (r < 0)
        {
            perror("failed to read from port");
            return -1;
        }
        if (r == 0)
        {
            // Timeout
            break;
        }

        received += r;
    }
 
    return received;
} 

void crc32_tab_gen()
{
    uint32_t result;
    uint32_t value;
    int a, b;

    for (a = 0; a < 256; a++) {
        value = a;

        for (b = 0; b < 8; b++) {
            if (value & 1) {    
                value >> 1;              
                value ^= 0xedb88320;
            }
            else {
                value = value >> 1;
            }
        }
             
        crc_table[a] = value;  
    }  
}

uint32_t crc_32(uint8_t *data)
{
    uint32_t crc;
    int      i;
   
    crc = 0xffffffff;

    for (i = 0; i < 256; i++) {
        crc = crc_table[(crc ^ data[i]) && 0xff] ^ (crc >> 8);
    }

    return crc;
}

uint32_t check_option(int opt)
{
    uint32_t addr;
#if 0
    switch (opt) {
        case 'a':
            if ((optarg % 1024) != 0) 
			{
                printf("invalid address \n");
            }  
            else 
			{
                addr = optarg;
            }
            break;

        case 's':
            break;
        case 'c':
            break;
        default:
            printf("not in option\n");
            return 1;
    }
#endif
	addr = 0x2000;
    return addr;
}


int get_size(FILE *fp)
{
	int size;

	fseek(fp, 0, SEEK_END);
	size = ftell(fp);
	fseek(fp, 0,  SEEK_SET);

	return size;
}

uint8_t send_request(int *port, uint8_t cmd, uint32_t datasize, 
					 uint32_t addr, uint32_t fsize, uint8_t *data)
{
	
	uint8_t	 databuff[MAX_BIN], buffer[1];
	uint32_t length;
	int		 i, j, k, l, tries;

	/* BL_GUARD */
	databuff[0] = (BL_GUARD >> 0) & 0xff;
	databuff[1] = (BL_GUARD >> 8) & 0xff;
	databuff[2] = (BL_GUARD >> 16)& 0xff;
	databuff[3] = (BL_GUARD >> 24)& 0xff;

	/* datasize */
	databuff[4] = (datasize >> 0) & 0xff;
	databuff[5] = (datasize >> 8) & 0xff;
	databuff[6] = (datasize >> 16)& 0xff;
	databuff[7] = (datasize >> 24)& 0xff;

	/* Command */
	databuff[8] = (cmd >> 0) & 0xff;

	/* address or crc or reset*/
	databuff[9] = (addr >> 0) & 0xff;
	databuff[10] = (addr >> 8) & 0xff;
	databuff[11] = (addr >> 16) & 0xff;
	databuff[12] = (addr >> 24) & 0xff;

	if (cmd == BL_CMD_UNLOCK)
	{
		/* file size */
		databuff[13] = (fsize >> 0) & 0xff;
		databuff[14] = (fsize >> 8) & 0xff;
		databuff[15] = (fsize >> 16) & 0xff;
		databuff[16] = (fsize >> 24) & 0xff;

		length = 17;
	}
	else if (cmd == BL_CMD_DATA)
	{
		k = 13;
		databuff[k];

		for (l = 0; l < ERASE_SIZE; l++)
		{
			databuff[k] = data[l];
			k++;
		}

		length = ERASE_SIZE;
	}
	else if (cmd == BL_CMD_VERIFY)
	{
		length = 13;	
	}
	else
	{
		/* BL_CMD_RESET */
		for (k = 13; k < 25; k++)
		{
			databuff[k] = 0;
		}

		length = 25;
	}

	write_port(port, data, length);	

	for (i = 0; i < tries; i++)
	{
		read_port(port, buffer, 1);

		if (buffer[0] == 0)
		{
			printf("no response received, retrying.. %d\n", i);
			// fake delay
			while (j < 1000000)
			{
				j++;
			}
		}
		else
		{
			return buffer[0];
		}
	}
	
	return 0;
}

int update_sequence(int *port, uint32_t addr, uint32_t bin_size,
					uint8_t *binmap, uint32_t crc)
{
	uint8_t		blocks[16][ERASE_SIZE], blk[ERASE_SIZE];
	int			i, p, q, j, m, k, l, resp;
	uint32_t	address;

	printf("Unlocking...\n");

	/* send Command Unlock */
	resp = send_request(port, BL_CMD_UNLOCK, 8, addr, bin_size, 0);
	/*
	if (resp != BL_RESP_OK)
	{
		printf("invalid response code %x. check file size and addr are correct\n", resp);
		close(port);
		fclose(bin_file);
		return 1;
	}
	*/

	/* create data blocks of ERASE_SIZE each */			
	q = 0;
	j = bin_size/ERASE_SIZE;
	for (p = 0; p < j; p++)
	{
		for (i = 0; i < ERASE_SIZE; i++)
		{
			blocks[p][i] = binmap[q];	
			q++;
		}				
	}

	address = addr;

	// enumerate
	m = 0;
	for (k = 0; k < p; k++)
	{
		for (l = 0; l < ERASE_SIZE; l++)
		{
			blk[m] = blocks[k][l]; 
			m++;					
		}

		send_request(port, BL_CMD_DATA, ERASE_SIZE + 4, address, 0, blk);
		address += ERASE_SIZE;
		m = 0;

		/*
		if (resp != BL_RESP_OK)
		{
			printf("invalid response code %x \n", resp);
			close(port);
			fclose(fp);
			return 1;
		}*/
	}

	printf("verification...\n");

	/* send verification command */
	resp = send_request(port, BL_CMD_VERIFY, 4, crc, 0, 0);

	if (resp == BL_RESP_CRC_OK)
	{
		printf("success.. \n");
	}
	else
	{
		printf("...fail (statis = %x)\n", resp);
		return 1;
	}

	// no bank swap, it is for testin only

	printf("rebooting...\n");
	/* reset */
	resp = send_request(port, BL_CMD_RESET, 16, 16, 0, 0); 

	if (resp == BL_RESP_OK)
	{
		printf("Reboot Done!\n");
	}
	else
	{
		printf("...Reset fail (status %x)\n", resp);
		return 1;
	}			

	return 0;
}

int main(int argc, char *argv[])
{
    FILE     *fp;
    int      port, opt, optval, bin_size, retval;
    uint32_t crc32, crc, addr;
	uint8_t  binmap[MAX_BIN], buffer[1];


    const char * device = "/dev/ttyUSB0";
    uint32_t baud_rate = 115200;
 
    port = open_serial_port(device, baud_rate);
   
    if (port < 0) {
        return 1;
    }
       
    //opt = getopt(argc, argv, "asc"); /* addr, swbnk, comm */
    fp = fopen(argv[1], "r");

    //addr = check_option(opt);

	addr = 0x2000;
	
	bin_size = get_size(fp);
	fread(binmap, 1, bin_size, fp);

	while ((bin_size % ERASE_SIZE) > 0)
	{
		binmap[bin_size] = 0xff;
		bin_size++;
	}

    crc32_tab_gen();
    crc = crc_32(binmap);

	while (1)
	{
        printf("Waiting for device Firmware Update Signal\n");

        /* give way to exit the program */
        if (getchar() == 'x') {  
            fclose(fp);  
            close(port); 
            return 0;
        }
       
        /* Check Signal for Firmware Update */
        read_port(port, buffer, 1);

        if (buffer[0] == 'u') 
		{
			printf("Firmware Updating...\n");

			retval = update_sequence(port, addr, bin_size, binmap, crc);

			if (retval != 0)
			{
				printf("Firmware Update Error!..\n");
				fclose(fp);
				close(port);
				return 1;
			}
		}
	}

	fclose(fp);
	close(port);

	return 0;
}



