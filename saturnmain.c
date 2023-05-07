/////////////////////////////////////////////////////////////
//
// Saturn project: Artix7 FPGA + Raspberry Pi4 Compute Module
// PCI Express interface from linux on Raspberry pi
// this application uses C code to emulate HPSDR protocol 2
//
// copyright Laurence Barker November 2021
// licenced under GNU GPL3
//
// p2app.c:
//
// Protocol2 is defined by "openHPSDR Ethernet Protocol V3.8"
// unlike protocol 1, it uses multiple ports for the data endpoints
//
//////////////////////////////////////////////////////////////

#include <gtk/gtk.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <semaphore.h>
#include <sys/stat.h>
#include <byteswap.h>

#include "saturntypes.h"
#include "saturnregisters.h"              // register I/O for Saturn
#include "saturnversion.h"                      // version I/O for Saturn
#include "saturndrivers.h"                      // version I/O for Saturn

#include "discovered.h"
#include "new_protocol.h"

#if 0
#include "threaddata.h"
#include "generalpacket.h"
#include "IncomingDDCSpecific.h"
#include "IncomingDUCSpecific.h"
#include "InHighPriority.h"
#include "InDUCIQ.h"
#include "InSpkrAudio.h"
#include "OutMicAudio.h"
#include "OutDDCIQ.h"
#include "OutHighPriority.h"
#endif


extern sem_t DDCInSelMutex;                 // protect access to shared DDC input select register
extern sem_t DDCResetFIFOMutex;             // protect access to FIFO reset register
extern sem_t RFGPIOMutex;                   // protect access to RF GPIO register
extern sem_t CodecRegMutex;                 // protect writes to codec

bool IsTXMode;                              // true if in TX
bool SDRActive;                             // true if this SDR is running at the moment
bool ExitRequested;                         // true if "exit checking" thread requests shutdown


#define SDRBOARDID 1                        // Hermes
#define SDRSWVERSION 1                      // version of this software
#define VDISCOVERYSIZE 60                   // discovery packet
#define VDISCOVERYREPLYSIZE 60              // reply packet
#define VWIDEBANDSIZE 1028                  // wideband scalar samples
#define VCONSTTXAMPLSCALEFACTOR 0x0001FFFF  // 18 bit scale value - set to 1/2 of full scale
#define VDMATRANSFERSIZE 4096
#define VDMABUFFERSIZE 131072               // memory buffer to reserve (4x DDC FIFO so OK)
#define VALIGNMENT 4096                     // buffer alignment
#define VBASE 0x1000                        // offset into I/Q buffer for DMA to start
#define VIQSAMPLESPERFRAME 238
#define VIQBYTESPERFRAME 6*VIQSAMPLESPERFRAME       // total bytes in one outgoing frame


static gpointer saturn_rx_thread(gpointer arg);
static GThread *saturn_rx_thread_id;

//
// this runs as its own thread to monitor command line activity. A string "exist" exits the application. 
// thread initiated at the start.
//
void *CheckForExitCommand(void *arg)
{
  char ch;
  printf("spinning up Check For Exit thread\n");
  
  while (1)
  {
    ch = getchar();
    if((ch == 'x') || (ch == 'X'))
    {
      ExitRequested = true;
      break;
    }
  }
}

//
// code to allocate and free dynamic allocated memory
// first the memory buffers:
//
uint8_t* DMAReadBuffer = NULL;                                                          // data for DMA read from DDC
uint32_t DMABufferSize = VDMABUFFERSIZE;
unsigned char* DMAReadPtr;                                                              // pointer for 1st available location in DMA memory
unsigned char* DMAHeadPtr;                                                              // ptr to 1st free location in DMA memory
unsigned char* DMABasePtr;                                                              // ptr to target DMA location in DMA memory

uint8_t* UDPBuffer[VNUMDDC];                                // DDC frame buffer
uint8_t* DDCSampleBuffer[VNUMDDC];                          // buffer per DDC
unsigned char* IQReadPtr[VNUMDDC];                                                      // pointer for reading out an I or Q sample
unsigned char* IQHeadPtr[VNUMDDC];                                                      // ptr to 1st free location in I/Q memory
unsigned char* IQBasePtr[VNUMDDC];                                                      // ptr to DMA location in I/Q memory


bool CreateDynamicMemory(void)                              // return true if error
{
    uint32_t DDC;
    bool Result = false;
//
// first create the buffer for DMA, and initialise its pointers
//
    posix_memalign((void**)&DMAReadBuffer, VALIGNMENT, DMABufferSize);
    DMAReadPtr = DMAReadBuffer + VBASE;                             // offset 4096 bytes into buffer
    DMAHeadPtr = DMAReadBuffer + VBASE;
    DMABasePtr = DMAReadBuffer + VBASE;
    if (!DMAReadBuffer)
    {
        printf("I/Q read buffer allocation failed\n");
        Result = true;
    }
    memset(DMAReadBuffer, 0, DMABufferSize);

    //
    // set up per-DDC data structures
    //
    for (DDC = 0; DDC < VNUMDDC; DDC++)
    {
        UDPBuffer[DDC] = malloc(VDDCPACKETSIZE);
        DDCSampleBuffer[DDC] = malloc(DMABufferSize);
        IQReadPtr[DDC] = DDCSampleBuffer[DDC] + VBASE;          // offset 4096 bytes into buffer
        IQHeadPtr[DDC] = DDCSampleBuffer[DDC] + VBASE;
        IQBasePtr[DDC] = DDCSampleBuffer[DDC] + VBASE;
    }
    return Result;
}


void FreeDynamicMemory(void)
{
    uint32_t DDC;

    free(DMAReadBuffer);
    //
    // free the per-DDC buffers
    //
    for (DDC = 0; DDC < VNUMDDC; DDC++)
    {
        free(UDPBuffer[DDC]);
        free(DDCSampleBuffer[DDC]);
    }
}


//
// main program. Initialise, then handle incoming command/general data
// has a loop that reads & processes incoming command packets
// see protocol documentation
// 
// if invoked "./p2app" - ADCs selected as normal
// if invoked "./p2app 1900000" - ADC1 and ADC2 inputs set to DDS test source at 1900000Hz
//
int saturn_init()
{
  int i, size;
  uint8_t CmdByte;                                                  // command word from PC app
  struct ifreq hwaddr;                                              // holds this device MAC address
  struct sockaddr_in addr_from;                                     // holds MAC address of source of incoming messages
  uint8_t UDPInBuffer[VDDCPACKETSIZE];                              // outgoing buffer
  struct iovec iovecinst;                                           // iovcnt buffer - 1 for each outgoing buffer
  struct msghdr datagram;                                           // multiple incoming message header

  //
  // initialise register access semaphores
  //
  sem_init(&DDCInSelMutex, 0, 1);                                   // for DDC input select register
  sem_init(&DDCResetFIFOMutex, 0, 1);                               // for FIFO reset register
  sem_init(&RFGPIOMutex, 0, 1);                                     // for RF GPIO register
  sem_init(&CodecRegMutex, 0, 1);                                   // for codec writes

//
// setup Saturn hardware
//
  OpenXDMADriver();
  //PrintVersionInfo();
  CodecInitialise();
  InitialiseDACAttenROMs();
  InitialiseCWKeyerRamp();
  SetCWSidetoneEnabled(true);
  SetTXProtocol(true);                                              // set to protocol 2
  SetTXModulationSource(eIQData);                                   // disable debug options
  //HandlerSetEERMode(false);                                         // no EER
  SetByteSwapping(false);                                            // h/w to generate NOT network byte order
  SetSpkrMute(false);
  SetTXAmplitudeScaling(VCONSTTXAMPLSCALEFACTOR);
  SetTXEnable(true);
#if 0
//
// start up thread for exit command checking
//
  ExitRequested = false;
  if(pthread_create(&CheckForExitThread, NULL, CheckForExitCommand, NULL) < 0)
  {
    perror("pthread_create check for exit");
    return EXIT_FAILURE;
  }
  pthread_detach(CheckForExitThread);


  //
  // check if we are using test source DDS
  //
  if (argc == 2)
	{
    TestFrequency = (atoi(argv[1]));
    SetTestDDSFrequency(TestFrequency, false);   
    UseTestDDSSource();                           
  }	

  //
  // create socket for incoming data on the command port
  //
  MakeSocket(SocketData, 0);

  //
  // get this device MAC address
  //
  memset(&hwaddr, 0, sizeof(hwaddr));
  strncpy(hwaddr.ifr_name, "eth0", IFNAMSIZ - 1);
  ioctl(SocketData[VPORTCOMMAND].Socketid, SIOCGIFHWADDR, &hwaddr);
  for(i = 0; i < 6; ++i) DiscoveryReply[i + 5] = hwaddr.ifr_addr.sa_data[i];         // copy MAC to reply message

  MakeSocket(SocketData+VPORTDDCSPECIFIC, 0);            // create and bind a socket
  if(pthread_create(&DDCSpecificThread, NULL, IncomingDDCSpecific, (void*)&SocketData[VPORTDDCSPECIFIC]) < 0)
  {
    perror("pthread_create DDC specific");
    return EXIT_FAILURE;
  }
  pthread_detach(DDCSpecificThread);

  MakeSocket(SocketData+VPORTDUCSPECIFIC, 0);            // create and bind a socket
  if(pthread_create(&DUCSpecificThread, NULL, IncomingDUCSpecific, (void*)&SocketData[VPORTDUCSPECIFIC]) < 0)
  {
    perror("pthread_create DUC specific");
    return EXIT_FAILURE;
  }
  pthread_detach(DUCSpecificThread);

  MakeSocket(SocketData+VPORTHIGHPRIORITYTOSDR, 0);            // create and bind a socket
  if(pthread_create(&HighPriorityToSDRThread, NULL, IncomingHighPriority, (void*)&SocketData[VPORTHIGHPRIORITYTOSDR]) < 0)
  {
    perror("pthread_create High priority to SDR");
    return EXIT_FAILURE;
  }
  pthread_detach(HighPriorityToSDRThread);

  MakeSocket(SocketData+VPORTSPKRAUDIO, 0);            // create and bind a socket
  if(pthread_create(&SpkrAudioThread, NULL, IncomingSpkrAudio, (void*)&SocketData[VPORTSPKRAUDIO]) < 0)
  {
    perror("pthread_create speaker audio");
    return EXIT_FAILURE;
  }
  pthread_detach(SpkrAudioThread);

  MakeSocket(SocketData+VPORTDUCIQ, 0);            // create and bind a socket
  if(pthread_create(&DUCIQThread, NULL, IncomingDUCIQ, (void*)&SocketData[VPORTDUCIQ]) < 0)
  {
    perror("pthread_create DUC I/Q");
    return EXIT_FAILURE;
  }
  pthread_detach(DUCIQThread);

//
// create outgoing mic data thread
// note this shares a port with incoming DUC specific, so don't create a new port
//  MakeSocket(SocketData+VPORTMICAUDIO, 0);
//
  if(pthread_create(&MicThread, NULL, OutgoingMicSamples, (void*)&SocketData[VPORTDUCSPECIFIC]) < 0)
  {
    perror("pthread_create Mic");
    return EXIT_FAILURE;
  }
  pthread_detach(MicThread);


//
// create outgoing high priority data thread
// note this shares a port with incoming DDC specific, so don't create a new port
//  MakeSocket(SocketData+VPORTHIGHPRIORITYFROMSDR, 0);
//
  if(pthread_create(&HighPriorityFromSDRThread, NULL, OutgoingHighPriority, (void*)&SocketData[VPORTDDCSPECIFIC]) < 0)
  {
    perror("pthread_create outgoing hi priority");
    return EXIT_FAILURE;
  }
  pthread_detach(HighPriorityFromSDRThread);


//
// and for now create just one outgoing DDC data thread for DDC 0
// create all the sockets though!
//
  MakeSocket(SocketData + VPORTDDCIQ0, 0);
  MakeSocket(SocketData + VPORTDDCIQ1, 0);
  MakeSocket(SocketData + VPORTDDCIQ2, 0);
  MakeSocket(SocketData + VPORTDDCIQ3, 0);
  MakeSocket(SocketData + VPORTDDCIQ4, 0);
  MakeSocket(SocketData + VPORTDDCIQ5, 0);
  MakeSocket(SocketData + VPORTDDCIQ6, 0);
  MakeSocket(SocketData + VPORTDDCIQ7, 0);
  MakeSocket(SocketData + VPORTDDCIQ8, 0);
  MakeSocket(SocketData + VPORTDDCIQ9, 0);
  if(pthread_create(&DDCIQThread[0], NULL, OutgoingDDCIQ, (void*)&SocketData[VPORTDDCIQ0]) < 0)
  {
    perror("pthread_create DUC I/Q");
    return EXIT_FAILURE;
  }
  pthread_detach(DDCIQThread[0]);


  //
  // now main processing loop. Process received Command packets arriving at port 1024
  // these are identified by the command byte (byte 4)
  // cmd=00: general packet
  // cmd=02: discovery
  // cmd=03: set IP address (not supported)
  // cmd=04: erase (not supported)
  // cmd=05: program (not supported)
  //
  while(1)
  {
    memset(&iovecinst, 0, sizeof(struct iovec));
    memset(&datagram, 0, sizeof(datagram));
    iovecinst.iov_base = &UDPInBuffer;                  // set buffer for incoming message number i
    iovecinst.iov_len = VDDCPACKETSIZE;
    datagram.msg_iov = &iovecinst;
    datagram.msg_iovlen = 1;
    datagram.msg_name = &addr_from;
    datagram.msg_namelen = sizeof(addr_from);
    size = recvmsg(SocketData[0].Socketid, &datagram, 0);         // get one message. If it times out, gets size=-1
    if(size < 0 && errno != EAGAIN)
    {
      perror("recvfrom");
      return EXIT_FAILURE;
    }

    if(ExitRequested)
      break;


//
// only process packets of length 60 bytes on this port, to exclude protocol 1 discovery for example.
// (that means we can't handle the programming packet but we don't use that anyway)
//
    CmdByte = UDPInBuffer[4];
    if(size==VDISCOVERYSIZE)  
      switch(CmdByte)
      {
        //
        // general packet. Get the port numbers and establish listener threads
        //
        case 0:
          printf("P2 General packet to SDR, size= %d\n", size);
          //
          // get "from" MAC address and port; this is where the data goes back to
          //
          memset(&reply_addr, 0, sizeof(reply_addr));
          reply_addr.sin_family = AF_INET;
          reply_addr.sin_addr.s_addr = addr_from.sin_addr.s_addr;
          reply_addr.sin_port = addr_from.sin_port;                       // (but each outgoing thread needs to set its own sin_port)
          HandleGeneralPacket(UDPInBuffer);
          break;

        //
        // discovery packet
        //
        case 2:
          printf("P2 Discovery packet\n");
          if(SDRActive)
            DiscoveryReply[4] = 3;                             // response 2 if not active, 3 if running
          else
            DiscoveryReply[4] = 2;                             // response 2 if not active, 3 if running

          memset(&UDPInBuffer, 0, VDISCOVERYREPLYSIZE);
          memcpy(&UDPInBuffer, DiscoveryReply, VDISCOVERYREPLYSIZE);
          sendto(SocketData[0].Socketid, &UDPInBuffer, VDISCOVERYREPLYSIZE, 0, (struct sockaddr *)&addr_from, sizeof(addr_from));
          break;

        case 3:
        case 4:
        case 5:
          printf("Unsupported packet\n");
          break;

        default:
          break;

      }// end switch (packet type)

//
// now do any "post packet" processing
//
  } //while(1)

#endif
  return EXIT_SUCCESS;
}


void saturn_exit() {
  //
  // clean exit
  //
  printf("Exiting\n");
  sem_destroy(&DDCInSelMutex);
  sem_destroy(&DDCResetFIFOMutex);
  sem_destroy(&RFGPIOMutex);
  sem_destroy(&CodecRegMutex);
  SetMOX(false);
  SetTXEnable(false);
}

void saturn_discovery() {
  if(devices<MAX_DEVICES) {

  struct stat sb; 
  int status = 1;
  char buf[256];
  FILE *fp;
  uint8_t *mac = discovered[devices].info.network.mac_address;

  if (stat("/dev/xdma/card0", &sb) == 0 && S_ISDIR(sb.st_mode))
  {
    saturn_init();
    discovered[devices].protocol = NEW_PROTOCOL;
    discovered[devices].device = NEW_DEVICE_SATURN;
    discovered[devices].software_version = (RegisterRead(VADDRSWVERSIONREG) >> 4) & 0xFFFF; 
    discovered[devices].fpga_version = RegisterRead(VADDRUSERVERSIONREG);
    strcpy(discovered[devices].name,"saturn");
    discovered[devices].frequency_min=0.0;
    discovered[devices].frequency_max=61440000.0;
    memset(buf, 0, 256);
    fp = fopen("/sys/class/net/eth0/address", "rt");
    if (fp) {
      if (fgets(buf, sizeof buf, fp) > 0) {
        sscanf(buf, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &mac[0],
               &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
        status = 0;
      }   
      fclose(fp);
    } else
    for(int i=0;i<6;i++) {
      discovered[devices].info.network.mac_address[i]=0;
    }   
    discovered[devices].status = STATE_AVAILABLE;
    discovered[devices].info.network.address_length=0;
    discovered[devices].info.network.interface_length=0;
    strcpy(discovered[devices].info.network.interface_name,"XDMA");
    discovered[devices].use_tcp=0;
    discovered[devices].use_routing=0;
    discovered[devices].supported_receivers=2;
    fprintf(stderr,"discovery: found SATURN device min=%f max=%f\n",
                            discovered[devices].frequency_min,
                            discovered[devices].frequency_max);

    devices++;
  }
  }
}

uint32_t SequenceCounter[VNUMDDC];                          // UDP sequence count

void start_saturn_receive_thread()
{
  g_print("new_protocol starting SATURN receive thread\n");

  saturn_rx_thread_id = g_thread_new( "SATURN RX", saturn_rx_thread, NULL);
  if( ! saturn_rx_thread_id )
  {
    g_print("g_thread_new failed for saturn_rx_thread\n");
    exit( -1 );
  }
}

static gpointer saturn_rx_thread(gpointer arg) {
  int bytes;

  g_print( "new_protocol: SATURN receive_thread\n");
  int running=1;
  while (running)
  {
    //usleep(100000); // 100 ms
    //process_iq_data(unsigned char *buffer, RECEIVER *rx);
#if 1
//
// memory buffers
//
    uint32_t DMATransferSize;
    bool InitError = false;                                     // becomes true if we get an initialisation error
    
    uint32_t ResidueBytes;
    uint32_t Depth = 0;
    
    int IQReadfile_fd = -1;									    // DMA read file device
    uint32_t RegisterValue;
    bool FIFOOverflow;
    int DDC;                                                    // iterator

//
// variables for analysing a DDC frame
//
    uint32_t FrameLength;                                       // number of words per frame
    uint32_t DDCCounts[VNUMDDC];                                // number of samples per DDC in a frame
    uint32_t RateWord;                                          // DDC rate word from buffer
    uint32_t HdrWord;                                           // check word read form DMA's data
    uint16_t* SrcWordPtr, * DestWordPtr;                        // 16 bit read & write pointers
    uint32_t *LongWordPtr;
    uint32_t PrevRateWord;                                      // last used rate word
    uint32_t Cntr;                                              // sample word counter
    bool HeaderFound;
    uint32_t DecodeByteCount;                                   // bytes to decode

//
// initialise. Create memory buffers and open DMA file devices
//
    PrevRateWord = 0xFFFFFFFF;                                  // illegal value to forc re-calculation of rates
    DMATransferSize = VDMATRANSFERSIZE;                         // initial size, but can be changed
    InitError = CreateDynamicMemory();
    //
    // open DMA device driver
    //
    IQReadfile_fd = open(VDDCDMADEVICE, O_RDWR);
    if (IQReadfile_fd < 0)
    {
        printf("XDMA read device open failed for DDC data\n");
        InitError = true;
    }


//
// now initialise Saturn hardware.
// ***This is debug code at the moment. ***
// clear FIFO
// then read depth
//
//    RegisterWrite(0x1010, 0x0000002A);      // disable DDC data transfer; DDC2=test source
    SetRXDDCEnabled(false);
    usleep(1000);                           // give FIFO time to stop recording 
    SetupFIFOMonitorChannel(eRXDDCDMA, false);
    ResetDMAStreamFIFO(eRXDDCDMA);
    RegisterValue = ReadFIFOMonitorChannel(eRXDDCDMA, &FIFOOverflow);				// read the FIFO Depth register
	printf("DDC FIFO Depth register = %08x (should be 0)\n", RegisterValue);
	Depth=0;

  SetByteSwapping(true);                                            // h/w to generate NOT network byte order

//
// thread loop. runs continuously until commanded by main loop to exit
// for now: add 1 RX data + mic data at 48KHz sample rate. Mic data is constant zero.
// while there is enough I/Q data, make outgoing packets;
// when not enough data, read more.
//
    while(!InitError)
    {
        printf("starting outgoing DDC data\n");
        //
        // initialise outgoing DDC packets - 1 per DDC
        //
        for (DDC = 0; DDC < VNUMDDC; DDC++)
        {
            SequenceCounter[DDC] = 0;
	    SetDDCADC(DDC, 0);
        }
	SetP2SampleRate(2, TRUE, 48, FALSE);
	SetP2SampleRate(3, TRUE, 48, FALSE);
        WriteP2DDCRateRegister();
      //
      // enable Saturn DDC to transfer data
      //
        printf("outDDCIQ: enable data transfer\n");
        SetRXDDCEnabled(true);
        HeaderFound = false;
        while(!InitError && running)
        {

        //
        // loop through all DDC I/Q buffers.
        // while there is enough I/Q data for this DDC in local (ARM) memory, make DDC Packets
        // then put any residues at the heads of the buffer, ready for new data to come in
        //
            for (DDC = 0; DDC < VNUMDDC; DDC++)
            {
                while ((IQHeadPtr[DDC] - IQReadPtr[DDC]) > VIQBYTESPERFRAME)
                {
//                    printf("enough data for packet: DDC= %d\n", DDC);
                    *(uint32_t*)UDPBuffer[DDC] = bswap_32(SequenceCounter[DDC]++);     // add sequence count
                    memset(UDPBuffer[DDC] + 4, 0, 8);                               // clear the timestamp data
                    *(uint16_t*)(UDPBuffer[DDC] + 12) = bswap_16(24);                  // bits per sample
                    *(uint16_t*)(UDPBuffer[DDC] + 14) = bswap_16(VIQSAMPLESPERFRAME);  // I/Q samples for ths frame
                    //
                    // now add I/Q data & send outgoing packet
                    //
                    memcpy(UDPBuffer[DDC] + 16, IQReadPtr[DDC], VIQBYTESPERFRAME);
                    IQReadPtr[DDC] += VIQBYTESPERFRAME;

                    int Error;
                    //Error = sendmsg((ThreadData+DDC)->Socketid, &datagram[DDC], 0);
                    //printf("RRK, DDC:%d data:%x\n", DDC, (uint32_t *)(UDPBuffer[DDC] + 16));
                    saturn_post_iq_data(DDC, UDPBuffer[DDC]);

                    if (Error == -1)
                    {
                        //printf("Send Error, DDC=%d, errno=%d, socket id = %d\n", DDC, errno, (ThreadData+DDC)->Socketid);
                        InitError = true;
                    }
                }
                //
                // now copy any residue to the start of the buffer (before the data copy in point)
                // unless the buffer already starts at or below the base
                // if we do a copy, the 1st free location is always base addr
                //
                ResidueBytes = IQHeadPtr[DDC] - IQReadPtr[DDC];
                //		printf("Residue = %d bytes\n",ResidueBytes);
                if (IQReadPtr[DDC] > IQBasePtr[DDC])                                // move data down
                {
                    if (ResidueBytes != 0) 		// if there is residue to move
                    {
                        memcpy(IQBasePtr[DDC] - ResidueBytes, IQReadPtr[DDC], ResidueBytes);
                        IQReadPtr[DDC] = IQBasePtr[DDC] - ResidueBytes;
                    }
                    else
                        IQReadPtr[DDC] = IQBasePtr[DDC];
                    IQHeadPtr[DDC] = IQBasePtr[DDC];                            // ready for new data at base
                }
            }
            //
            // P2 packet sending complete.There are no DDC buffers with enough data to send out.
            // bring in more data by DMA if there is some, else sleep for a while and try again
            // we have the same issue with DMA: a transfer isn't exactly aligned to the amount we can read out 
            // according to the DDC settings. So we either need to have the part-used DDC transfer variables
            // persistent across DMAs, or we need to recognise an incomplete fragment of a frame as such
            // and copy it like we do with IQ data so the next readout begins at a new frame
            // the latter approach seems easier!
            //
            Depth = ReadFIFOMonitorChannel(eRXDDCDMA, &FIFOOverflow);				// read the FIFO Depth register
            //		printf("read: depth = %d\n", Depth);
            while(Depth < (DMATransferSize/8U))			// 8 bytes per location
            {
                usleep(1000);								// 1ms wait
                Depth = ReadFIFOMonitorChannel(eRXDDCDMA, &FIFOOverflow);				// read the FIFO Depth register
            //			printf("read: depth = %d\n", Depth);
            }
//            printf("DDC DMA read %d bytes from destination to base\n", DMATransferSize);
            DMAReadFromFPGA(IQReadfile_fd, DMAHeadPtr, DMATransferSize, VADDRDDCSTREAMREAD);
            DMAHeadPtr += DMATransferSize;
            //
            // find header: may not be the 1st word
            //
//            DumpMemoryBuffer(DMAReadPtr, DMATransferSize);
            if(HeaderFound == false)                                                    // 1st time: look for header
                for(Cntr=16; Cntr < (DMAHeadPtr - DMAReadPtr); Cntr+=8)                  // search for rate word; ignoring 1st
                {
                    if(*(DMAReadPtr + Cntr + 7) == 0x80)
                    {
//                        printf("found header at offset=%x\n", Cntr);
                        HeaderFound = true;
                        DMAReadPtr += Cntr;                                             // point read buffer where header is 
                        break;
                    }
                }
            if (HeaderFound == false)                                        // if rate flag not set
            {
//                printf("Rate word not found when expected. rate= %08x\n", RateWord);
                InitError = true;
                exit(1);
            }


            //
            // finally copy data to DMA buffers according to the embedded DDC rate words
            // the 1st word is pointed by DMAReadPtr and it should point to a DDC rate word
            // search for it if not!
            // (it should always be left in that state).
            // the top half of the 1st 64 bit word should be 0x8000
            // and that is located in the 2nd 32 bit location.
            // assume that DMA is > 1 frame.
//            printf("headptr = %x readptr = %x\n", DMAHeadPtr, DMAReadPtr);
            DecodeByteCount = DMAHeadPtr - DMAReadPtr;
            while (DecodeByteCount >= 16)                       // minimum size to try!
            {
                if(*(DMAReadPtr + 7) != 0x80)
                {
                    printf("header not found for rate word at addr %x\n", DMAReadPtr);
                    exit(1);
                }
                else                                                                    // analyse word, then process
                {
                    LongWordPtr = (uint32_t*)DMAReadPtr;                            // get 32 bit ptr
                    RateWord = *LongWordPtr;                                      // read rate word

                    if (RateWord != PrevRateWord)
                    {
                        FrameLength = AnalyseDDCHeader(RateWord, &DDCCounts[0]);           // read new settings
//                        printf("new framelength = %d\n", FrameLength);
                        PrevRateWord = RateWord;                                        // so so we know its analysed
                    }
                    if (DecodeByteCount >= ((FrameLength+1) * 8))             // if bytes for header & frame
                    {
                        //THEN COPY DMA DATA TO I / Q BUFFERS
                        DMAReadPtr += 8;                                                // point to 1st location past rate word
                        SrcWordPtr = (uint16_t*)DMAReadPtr;                             // read sample data in 16 bit chunks
                        for (DDC = 0; DDC < VNUMDDC; DDC++)
                        {
                            HdrWord = DDCCounts[DDC];                                   // number of words for this DDC. reuse variable
                            if (HdrWord != 0)
                            {
                                DestWordPtr = (uint16_t *)IQHeadPtr[DDC];
                                for (Cntr = 0; Cntr < HdrWord; Cntr++)                  // count 64 bit words
                                {
                                    *DestWordPtr++ = *SrcWordPtr++;                     // move 48 bits of sample data
                                    *DestWordPtr++ = *SrcWordPtr++;
                                    *DestWordPtr++ = *SrcWordPtr++;
                                    SrcWordPtr++;                                       // and skip 16 bits where theres no data
                                }
                                IQHeadPtr[DDC] += 6 * HdrWord;                          // 6 bytes per sample
                            }
                            // read N samples; write at head ptr
                        }
                        DMAReadPtr += FrameLength * 8;                                  // that's how many bytes we read out
                        DecodeByteCount -= (FrameLength+1) * 8;
                    }
                    else
                        break;                                                          // if not enough left, exit loop
                }
            }
            //
            // now copy any residue to the start of the buffer (before the data copy in point)
            // unless the buffer already starts at or below the base
            // if we do a copy, the 1st free location is always base addr
            //
            ResidueBytes = DMAHeadPtr - DMAReadPtr;
            //		printf("Residue = %d bytes\n",ResidueBytes);
            if (DMAReadPtr > DMABasePtr)                                // move data down
            {
                if (ResidueBytes != 0) 		// if there is residue to move
                {
                    memcpy(DMABasePtr - ResidueBytes, DMAReadPtr, ResidueBytes);
                    DMAReadPtr = DMABasePtr - ResidueBytes;
                }
                else
                    DMAReadPtr = DMABasePtr;
                DMAHeadPtr = DMABasePtr;                            // ready for new data at base
            }
        }     // end of while(!InitError) loop
    }

#endif
  }
  return NULL;
}

void saturn_set_frequency(int ddc, unsigned long phase) {
  //printf("RRKS, ddc:%d phase:%d\n", ddc, phase);
  SetDDCFrequency(ddc, phase, true);
}

void saturn_set_sample_rate(int ddc, int enable, int rate) {
  //printf("RRKS, ddc:%d enable:%d rate:%d\n", ddc, enable, rate/1000);
  SetP2SampleRate(ddc, enable, rate/1000, FALSE);
  WriteP2DDCRateRegister();
}



