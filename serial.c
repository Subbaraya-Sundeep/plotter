/*
 * Copyright (c) 2016 Subbaraya Sundeep <sundeep.babi@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This program sends coordinates for stepper motors and whether Pen to be
 * Down or Up via serial port to Arduino.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <malloc.h>

int set_interface_attribs (int fd, int speed, int parity)
{
	struct termios tty;

	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		printf ("error %d from tcgetattr", errno);
 		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
 	tty.c_iflag &= ~IGNBRK;
	tty.c_lflag = 0;                // no signaling chars, no echo,
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls, // enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
 	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0) {
		printf ("error %d from tcsetattr", errno);
		return -1;
	}

	return 0;
}

void set_blocking (int fd, int should_block)
{
	struct termios tty;

	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0) {
		printf ("error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
		printf ("error %d setting term attributes", errno);
}

char bufx [5];
char bufy [5];
int fd;

int sendx(int x)
{
	int n, len;

	memset(bufx, 0, sizeof(bufx));

	/* -------------------- X --------------- */
	sprintf(bufx, "X%d", x);
	len = strlen(bufx);
	if (strlen(bufx) != 4) {
		if (len == 3) {
			bufx[3] = bufx[2];
			bufx[2] = bufx[1];
			bufx[1] = '0';
		} else {
			bufx[3] = bufx[1];
			bufx[2] = '0';
			bufx[1] = '0';
		}
	}

	printf("%s\t", bufx);
	sendserial(bufx);
}

int sendy(int y)
{
	int n, len;

	memset(bufy, 0, sizeof(bufy));

	/* -------------------- Y --------------- */
	sprintf(bufy, "Y%d", y);
	len = strlen(bufy);
	if (strlen(bufy) != 4)
	{
		if (len == 3) {
			bufy[3] = bufy[2];
			bufy[2] = bufy[1];
			bufy[1] = '0';
		} else {
			bufy[3] = bufy[1];
			bufy[2] = '0';
			bufy[1] = '0';
		}
	}

	printf("%s\t", bufy);
	sendserial(bufy);
}

int sendserial(char *str)
{
	int n;
	char buf[10];

	memset(buf, 0, 5);

	n = write (fd, str, 4); 

	int index = 0;
	int bytes = 0;
	n = 0;

	while (bytes < 4)
	{
		n = read (fd, buf + index, sizeof buf);
		if (n < 0) {
			printf("read error\n");
		} else if (n == 0) {
			printf("no data");
		} else {
			index += n;
			bytes += n;
		}
	}

/*	printf("%s\n", buf);

	if (strcmp(str, buf)) {
		printf("Communication error dude\n");
	} */
	return 0;
}

unsigned char image[96][96];

unsigned char *read_bmp(char *fname,int* _w, int* _h)
{
    unsigned char head[54];
    FILE *f = fopen(fname,"rb");

    // BMP header is 54 bytes
    fread(head, 1, 54, f);

    int w = head[18] + ( ((int)head[19]) << 8) + ( ((int)head[20]) << 16) +
				( ((int)head[21]) << 24);
    int h = head[22] + ( ((int)head[23]) << 8) + ( ((int)head[24]) << 16) +
				( ((int)head[25]) << 24);

	printf("w:%d  h:%d\n", w, h);
    // lines are aligned on 4-byte boundary
    int lineSize = (w / 8 + (w / 8) % 4);
    int fileSize = lineSize * h;

    unsigned char *img = malloc(w * h), *data = malloc(fileSize);

    // skip the header
    fseek(f,54,SEEK_SET);

    // skip palette - two rgb quads, 8 bytes
    fseek(f, 8, SEEK_CUR);

    // read data
    fread(data,1,fileSize,f);

    // decode bits
    int i, j, k, rev_j;
    for(j = 0, rev_j = h - 1; j < h ; j++, rev_j--) {
        for(i = 0 ; i < w / 8; i++) {
            int fpos = j * lineSize + i, pos = rev_j * w + i * 8;
            for(k = 0 ; k < 8 ; k++)
                img[pos + (7 - k)] = (data[fpos] >> k ) & 1;
        }
    }

    free(data);
    *_w = w; *_h = h;
    return img;
}

void sendimg()
{
    int w, h, i, j;
    unsigned char* img = read_bmp("lady.bmp", &w, &h);
  
    for(j = 0 ; j < h ; j++)
    {
        for(i = 0 ; i < w ; i++) {
		image[i][j] = img[j * w + i];
		if (image[i][j] == 0) {		    
			sendx(j);
			sendy(i);		
			sendserial("Pene");
		}
	}
	printf("Y axis finished\n");
	sendserial("RevY");
    }
}

int main(void) 
{
	char *portname = "/dev/ttyUSB0";
	int x, y;

	fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
        	printf ("error %d opening %s: %s", errno, portname, 
						strerror (errno));
        	return;
	}

	sleep (2);             // sleep enough to transmit the 7 plus

	set_interface_attribs (fd, B9600, 0);  // 9600 bps, 8n1 (no parity)
	set_blocking (fd, 1);                // set no blocking

	/* ----------------------------------------------------- */
	sendserial("Home");
	sendimg();
	close(fd);
}

