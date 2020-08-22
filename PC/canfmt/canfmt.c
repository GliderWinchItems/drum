/* *****************************************************************************
* File Name          : canfmt.c
* Date First Issued  : 03/16/2020
* Board              : Linux PC
* Description        : Convert gateway format CAN msgs to make small bit more readable
****************************************************************************** */

/*
gcc -Wall canfmt.c -o canfmt 
gcc -Wall canfmt.c -o canfmt && ./canfmt < ~/GliderWinchItems/GEVCUr/docs/data/log200315.txt | tee x
./canfmt < ~/GliderWinchItems/GEVCUr/docs/data/log200315.txt | tee x
*/

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>

/* Line buffer size */
#define LINESZ 512	// Longest CAN msg line length
char buf[LINESZ];

/* Line number used for locating problems */
unsigned int linect = 0;

FILE* fpIn;

struct CANTBL
{
	uint8_t seq;
	uint32_t id;
	uint8_t dlc;
	uint8_t uc[8];
};

/* ************************************************************************************************************ */
/*  Yes, this is where it starts.                                                                               */
/* ************************************************************************************************************ */
int main(int argc, char **argv)
{
	int m;
	struct CANTBL cantblx;
	uint32_t ui;

	while ( (fgets (&buf[0],LINESZ,stdin)) != NULL)	// Get a line from stdin
	{
		if ((strlen(buf) > 12) && (strlen(buf) < 32))
		{
			sscanf(buf,"%2x",&cantblx.seq);

			// The ascii hex order is little endian.  Convert to an unsigned int
			cantblx.id = 0;
			for (m = 10; m >= 2; m-=2)
			{
				sscanf(&buf[m],"%2x",&ui);
				cantblx.id = (cantblx.id << 8) + ui;
			}
			sscanf(&buf[10],"%2x",(char*)&cantblx.dlc);

			printf("%03u %08X %1d ", cantblx.seq,cantblx.id,cantblx.dlc);

			// Get payload bytes	converted to binary
			for (m = 0; m < cantblx.dlc; m++)
			{
				sscanf(&buf[12+2*m]," %2x",(unsigned int*)&cantblx.uc[m]);
				printf(" %02X",cantblx.uc[m]);
			}
			printf ("\n");
		}
	}
	return 0;
}	

