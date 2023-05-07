//////////////////////////////////////////////////////////////
//
// Saturn project: Artix7 FPGA + Raspberry Pi4 Compute Module
// PCI Express interface from linux on Raspberry pi
// this application uses C code to emulate HPSDR protocol 1 
//
// copyright Laurence Barker November 2021
// licenced under GNU GPL3
//
// version.c:
// print version information from FPGA registers

//
//////////////////////////////////////////////////////////////

#define _DEFAULT_SOURCE
#define _XOPEN_SOURCE 500
#include <assert.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "saturnversion.h"
#include "saturnregisters.h"





#define VMAXPRODUCTID 2							// product ID index limit
#define VMAXSWID 3								// software ID index limit

char* ProductIDStrings[] =
{
	"invalid product ID",
	"Saturn 1st prototype",
	"Saturn 2nd Prototype"
};

char* SWIDStrings[] =
{
	"invalid software ID",
	"Saturn 1st prototype, board test code",
	"Saturn 1st prototype, with DSP",
	"Saturn 2nd prototype, with DSP"

};

char* ClockStrings[] =
{
	"122.88MHz main clock",
	"10MHz Reference clock",
	"EMC config clock",
	"122.88MHz main clock"
};


//
// prints version information from the registers
//
void PrintVersionInfo(void)
{
	uint32_t SoftwareInformation;			// swid & version
	uint32_t ProductInformation;			// product id & version
	uint32_t DateCode;						// date code from user register in FPGA

	uint32_t SWVer, SWID;					// s/w version and id
	uint32_t ProdVer, ProdID;				// product version and id
	uint32_t ClockInfo;						// clock status
	uint32_t Cntr;

	char* ProdString;
	char* SWString;

	//
	// read the raw data from registers
	//
	SoftwareInformation = RegisterRead(VADDRSWVERSIONREG);
	ProductInformation = RegisterRead(VADDRPRODVERSIONREG);
	DateCode = RegisterRead(VADDRUSERVERSIONREG);
	printf("FPGA BIT file data code = %08x\n", DateCode);

	ClockInfo = (SoftwareInformation & 0xF);				// 4 clock bits
	SWVer = (SoftwareInformation >> 4) & 0xFFFF;			// 16 bit sw version
	SWID = SoftwareInformation >> 20;						// 12 bit software ID

	ProdVer = ProductInformation & 0xFFFF;					// 16 bit product version
	ProdID = ProductInformation >> 16;						// 16 bit product ID

	//
	// now chack if IDs are valid and print strings
	//
	if (ProdID > VMAXPRODUCTID)
		ProdString = ProductIDStrings[0];
	else
		ProdString = ProductIDStrings[ProdID];

	if (SWID > VMAXSWID)
		SWString = SWIDStrings[0];
	else
		SWString = SWIDStrings[SWID];

	printf(" Product: %s; Version = %d\n", ProdString, ProdVer);
	printf(" Software loaded: %s; SW Version = %d\n", SWString, SWVer);

	if (ClockInfo == 0xF)
		printf("All clocks present\n");
	else
	{
		for (Cntr = 0; Cntr < 4; Cntr++)
		{
			if (ClockInfo & 1)
				printf("%s present\n", ClockStrings[Cntr]);
			else
				printf("%s not present\n", ClockStrings[Cntr]);
			ClockInfo = ClockInfo >> 1;
		}
	}
}



