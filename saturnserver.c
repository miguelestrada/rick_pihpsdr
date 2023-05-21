/////////////////////////////////////////////////////////////
//
// Saturn project: Artix7 FPGA + Raspberry Pi4 Compute Module
// PCI Express interface from linux on Raspberry pi
// this application uses C code to emulate HPSDR protocol 2
//
// copyright Laurence Barker November 2021
// licenced under GNU GPL3
//
// saturnserver.c:
//
// Contribution of interfacing to PiHPSDR from N1GP (Rick Koch)
//
// Protocol2 is defined by "openHPSDR Ethernet Protocol V3.8"
// unlike protocol 1, it uses multiple ports for the data endpoints
//
// Saturn network interface to PiHPSDR
//
//////////////////////////////////////////////////////////////


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

#include "saturnregisters.h"              // register I/O for Saturn
#include "saturnserver.h"
#include "saturndrivers.h"
#include "saturnmain.h"

struct sockaddr_in reply_addr;              // destination address for outgoing data

bool ReplyAddressSet = false;               // true when reply address has been set
bool StartBitReceived = false;              // true when "run" bit has been set
bool NewMessageReceived = false;            // set whenever a message is received
bool ExitRequested = false;                 // true if "exit checking" thread requests shutdown
bool SkipExitCheck = false;                 // true to skip "exit checking", if running as a service
bool ThreadError = false;                   // true if a thread reports an error
bool ServerActive = false;

sem_t NetDDCRegMutex[5];

#define SDRBOARDID 1                        // Hermes
#define SDRSWVERSION 1                      // version of this software
#define VDISCOVERYSIZE 60                   // discovery packet
#define VDISCOVERYREPLYSIZE 60              // reply packet
#define VWIDEBANDSIZE 1028                  // wideband scalar samples
#define VCONSTTXAMPLSCALEFACTOR 0x0001FFFF  // 18 bit scale value - set to 1/2 of full scale

struct ThreadSocketData SocketData[VPORTTABLESIZE] =
{
  {0, 0, 1024, "Cmd", false,{}, 0, 0},                      // command (incoming) thread
  {0, 0, 1025, "DDC Specific", false,{}, 0, 0},             // DDC specifc (incoming) thread
  {0, 0, 1026, "DUC Specific", false,{}, 0, 0},             // DUC specific (incoming) thread
  {0, 0, 1027, "High Priority In", false,{}, 0, 0},         // High Priority (incoming) thread
  {0, 0, 1028, "Spkr Audio", false,{}, 0, 0},               // Speaker Audio (incoming) thread
  {0, 0, 1029, "DUC I/Q", false,{}, 0, 0},                  // DUC IQ (incoming) thread
  {0, 0, 1025, "High Priority Out", false,{}, 0, 0},        // High Priority (outgoing) thread
  {0, 0, 1026, "Mic Audio", false,{}, 0, 0},                // Mic Audio (outgoing) thread
  {0, 0, 1035, "DDC I/Q 0", false,{}, 0, 0},                // DDC IQ 0 (outgoing) thread
  {0, 0, 1036, "DDC I/Q 1", false,{}, 0, 0},                // DDC IQ 1 (outgoing) thread
  {0, 0, 1037, "DDC I/Q 2", false,{}, 0, 0},                // DDC IQ 2 (outgoing) thread
  {0, 0, 1038, "DDC I/Q 3", false,{}, 0, 0},                // DDC IQ 3 (outgoing) thread
  {0, 0, 1039, "DDC I/Q 4", false,{}, 0, 0},                // DDC IQ 4 (outgoing) thread
  {0, 0, 1040, "DDC I/Q 5", false,{}, 0, 0},                // DDC IQ 5 (outgoing) thread
  {0, 0, 1041, "DDC I/Q 6", false,{}, 0, 0},                // DDC IQ 6 (outgoing) thread
  {0, 0, 1042, "DDC I/Q 7", false,{}, 0, 0},                // DDC IQ 7 (outgoing) thread
  {0, 0, 1043, "DDC I/Q 8", false,{}, 0, 0},                // DDC IQ 8 (outgoing) thread
  {0, 0, 1044, "DDC I/Q 9", false,{}, 0, 0},                // DDC IQ 9 (outgoing) thread
  {0, 0, 1027, "Wideband 0", false,{}, 0, 0},               // Wideband 0 (outgoing) thread
  {0, 0, 1028, "Wideband 1", false,{}, 0, 0}                // Wideband 1 (outgoing) thread
};


//
// default port numbers, used if incoming port number = 0
//
uint16_t DefaultPorts[VPORTTABLESIZE] =
{
  1024, 1025, 1026, 1027, 1028, 
  1029, 1025, 1026, 1035, 1036, 
  1037, 1038, 1039, 1040, 1041, 
  1042, 1043, 1044, 1027, 1028
};


pthread_t saturn_server_thread;
pthread_t DDCSpecificThread;
pthread_t DUCSpecificThread;
pthread_t HighPriorityToSDRThread;
pthread_t SpkrAudioThread;
pthread_t DUCIQThread;
pthread_t DDCIQThread[VNUMDDC];               // array, but not sure how many
pthread_t MicThread;
pthread_t HighPriorityFromSDRThread;
pthread_t CheckForNoActivityThread;           // thread looks for inactvity

//
// function to check if any threads are still active
// loop through the table; report if any are true.
// parameter is to allow the "command" socket to stay open
//
bool CheckActiveThreads(int StartingPoint)
{
  struct ThreadSocketData* Ptr = SocketData+StartingPoint;
  bool Result = false;

  for (int i = StartingPoint; i < VPORTTABLESIZE; i++)          // loop through the socket table
  {
    if(Ptr->Active)                                 // check this thread
      Result = true;
    Ptr++;
  }
    if(Result)
      printf("found an active thread\n");
    return Result;
}



//
// set the port for a given thread. If 0, set the default according to HPSDR spec.
// if port is different from the currently assigned one, set the "change port" bit
//
void SetPort(uint32_t ThreadNum, uint16_t PortNum)
{
  uint16_t CurrentPort;

  CurrentPort = SocketData[ThreadNum].Portid;
  if(PortNum == 0)
    SocketData[ThreadNum].Portid = DefaultPorts[ThreadNum];     //default if not set
  else
    SocketData[ThreadNum].Portid = PortNum;

  if (SocketData[ThreadNum].Portid != CurrentPort)
    SocketData[ThreadNum].Cmdid |= VBITCHANGEPORT;
}



//
// function to make an incoming or outgoing socket, bound to the specified port in the structure
// 1st parameter is a link into the socket data table
//
int MakeSocket(struct ThreadSocketData* Ptr, int DDCid)
{
  struct timeval ReadTimeout;                                       // read timeout
  int yes = 1;
//  struct sockaddr_in addr_cmddata;
  //
  // create socket for incoming data
  //
  if((Ptr->Socketid = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
  {
    perror("socket fail");
    return EXIT_FAILURE;
  }

  //
  // set 1ms timeout, and re-use any recently open ports
  //
  setsockopt(Ptr->Socketid, SOL_SOCKET, SO_REUSEADDR, (void *)&yes , sizeof(yes));
  ReadTimeout.tv_sec = 0;
  ReadTimeout.tv_usec = 1000;
  setsockopt(Ptr->Socketid, SOL_SOCKET, SO_RCVTIMEO, (void *)&ReadTimeout , sizeof(ReadTimeout));

  //
  // bind application to the specified port
  //
  memset(&Ptr->addr_cmddata, 0, sizeof(struct sockaddr_in));
  Ptr->addr_cmddata.sin_family = AF_INET;
  Ptr->addr_cmddata.sin_addr.s_addr = htonl(INADDR_ANY);
  Ptr->addr_cmddata.sin_port = htons(Ptr->Portid);

  if(bind(Ptr->Socketid, (struct sockaddr *)&Ptr->addr_cmddata, sizeof(struct sockaddr_in)) < 0)
  {
    perror("bind");
    return EXIT_FAILURE;
  }

  struct sockaddr_in checkin;
  socklen_t len = sizeof(checkin);
  if(getsockname(Ptr->Socketid, (struct sockaddr *)&checkin, &len)==-1)
    perror("getsockname");

  Ptr->DDCid = DDCid;                       // set DDC number, for outgoing ports
  return 0;
}


//
// this runs as its own thread to see if messages have stopped being received.
// if nomessages in a second, goes back to "inactive" state.
//
void* CheckForActivity(void *arg)
{
  bool PreviouslyActiveState;               
  while(1)
  {
    sleep(1000);                               // wait for 1 second
    PreviouslyActiveState = ServerActive;      // see if active on entry
    if (!NewMessageReceived)                // if no messages received,
    {
      ServerActive = false;                    // set back to inactive
      ReplyAddressSet = false;
      StartBitReceived = false;
      if(PreviouslyActiveState)
        printf("Reverted to Inactive State after no activity\n");
    }
    NewMessageReceived = false;
  }
  printf("ENDING CheckForActivity thread\n");
}

//
// Shutdown()
// perform ordely shutdown of the program
//
void Shutdown()
{
  close(SocketData[0].Socketid);                          // close incoming data socket
  ExitRequested = true;
  saturn_exit();
  printf("Shutdown COMPLETE\n");
}


void start_saturn_server()
{
  if(pthread_create(&saturn_server_thread, NULL, saturn_server, NULL) < 0)
  {
    perror("pthread_create saturn_server thread");
    return;
  }
  pthread_detach(saturn_server_thread);
}

//
// Initialise, then handle incoming command/general data
// has a loop that reads & processes incoming command packets
// see protocol documentation
// 
void* saturn_server(void *arg)
{
  int i, size;
  uint8_t UDPInBuffer[VDISCOVERYSIZE];
//
// part written discovery reply packet
//
  uint8_t DiscoveryReply[VDISCOVERYREPLYSIZE] = 
  {
    0,0,0,0,                                      // sequence bytes
    2,                                            // 2 if not active; 3 if active
    0,0,0,0,0,0,                                  // SDR (raspberry i) MAC address
    5,                                            // board type. changed from "hermes" to "orion mk2"
    38,                                           // protocol version 3.8
    20,                                           // this SDR firmware version. >17 to enable QSK
    0,0,0,0,0,0,                                  // Mercury, Metis, Penny version numbers
    5,                                            // 6 DDC's for network client
    1,                                            // phase word
    0,                                            // endian mode
    0,0,                                          // beta version, reserved byte (total 25 useful bytes)
    0,0,0,0,0,0,0,0,0,0,                          // 10 bytes padding
    0,0,0,0,0,0,0,0,0,0,                          // 10 bytes padding
    0,0,0,0,0,0,0,0,0,0,0,0,0,0                   // 15 bytes padding
  };

  uint8_t CmdByte;                                                  // command word from PC app
  struct ifreq hwaddr;                                              // holds this device MAC address
  struct sockaddr_in addr_from;                                     // holds MAC address of source of incoming messages
  struct iovec iovecinst;                                           // iovcnt buffer - 1 for each outgoing buffer
  struct msghdr datagram;                                           // multiple incoming message header

  uint32_t TestFrequency;                                           // test source DDS freq
  int CmdOption;                                                    // command line option

//
// start up thread to check for no longer getting messages, to set back to inactive
//
  if(pthread_create(&CheckForNoActivityThread, NULL, CheckForActivity, NULL) < 0)
  {
    perror("pthread_create check for exit");
    return NULL;
  }
  pthread_detach(CheckForNoActivityThread);

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
    return NULL;
  }
  pthread_detach(DDCSpecificThread);

  MakeSocket(SocketData+VPORTDUCSPECIFIC, 0);            // create and bind a socket
  if(pthread_create(&DUCSpecificThread, NULL, IncomingDUCSpecific, (void*)&SocketData[VPORTDUCSPECIFIC]) < 0)
  {
    perror("pthread_create DUC specific");
    return NULL;
  }
  pthread_detach(DUCSpecificThread);

  MakeSocket(SocketData+VPORTHIGHPRIORITYTOSDR, 0);            // create and bind a socket
  if(pthread_create(&HighPriorityToSDRThread, NULL, IncomingHighPriority, (void*)&SocketData[VPORTHIGHPRIORITYTOSDR]) < 0)
  {
    perror("pthread_create High priority to SDR");
    return NULL;
  }
  pthread_detach(HighPriorityToSDRThread);
#if 0
  MakeSocket(SocketData+VPORTSPKRAUDIO, 0);            // create and bind a socket
  if(pthread_create(&SpkrAudioThread, NULL, IncomingSpkrAudio, (void*)&SocketData[VPORTSPKRAUDIO]) < 0)
  {
    perror("pthread_create speaker audio");
    return NULL;
  }
  pthread_detach(SpkrAudioThread);

  MakeSocket(SocketData+VPORTDUCIQ, 0);            // create and bind a socket
  if(pthread_create(&DUCIQThread, NULL, IncomingDUCIQ, (void*)&SocketData[VPORTDUCIQ]) < 0)
  {
    perror("pthread_create DUC I/Q");
    return NULL;
  }
  pthread_detach(DUCIQThread);
#endif

//
// create outgoing mic data thread
// note this shares a port with incoming DUC specific, so don't create a new port
// instead copy socket settings from DUCSPECIFIC socket:
//
#if 0
  SocketData[VPORTMICAUDIO].Socketid = SocketData[VPORTDUCSPECIFIC].Socketid;
  memcpy(&SocketData[VPORTMICAUDIO].addr_cmddata, &SocketData[VPORTDUCSPECIFIC].addr_cmddata, sizeof(struct sockaddr_in));
  if(pthread_create(&MicThread, NULL, OutgoingMicSamples, (void*)&SocketData[VPORTMICAUDIO]) < 0)
  {
    perror("pthread_create Mic");
    return NULL;
  }
  pthread_detach(MicThread);
#endif

//
// create outgoing high priority data thread
// note this shares a port with incoming DDC specific, so don't create a new port
// instead copy socket settings from VPORTDDCSPECIFIC socket:
//
#if 0
  SocketData[VPORTHIGHPRIORITYFROMSDR].Socketid = SocketData[VPORTDDCSPECIFIC].Socketid;
  memcpy(&SocketData[VPORTHIGHPRIORITYFROMSDR].addr_cmddata, &SocketData[VPORTDDCSPECIFIC].addr_cmddata, sizeof(struct sockaddr_in));
  if(pthread_create(&HighPriorityFromSDRThread, NULL, OutgoingHighPriority, (void*)&SocketData[VPORTHIGHPRIORITYFROMSDR]) < 0)
  {
    perror("pthread_create outgoing hi priority");
    return NULL;
  }
  pthread_detach(HighPriorityFromSDRThread);
#endif


//
// and for now create just one outgoing DDC data thread for DDC 0
// create all the sockets though!
//
  MakeSocket(SocketData + VPORTDDCIQ0, 0);
  MakeSocket(SocketData + VPORTDDCIQ1, 1);
  MakeSocket(SocketData + VPORTDDCIQ2, 2);
  MakeSocket(SocketData + VPORTDDCIQ3, 3);
  MakeSocket(SocketData + VPORTDDCIQ4, 4);
  MakeSocket(SocketData + VPORTDDCIQ5, 5);
  MakeSocket(SocketData + VPORTDDCIQ6, 6);
  MakeSocket(SocketData + VPORTDDCIQ7, 7);
  MakeSocket(SocketData + VPORTDDCIQ8, 8);
  MakeSocket(SocketData + VPORTDDCIQ9, 9);

  //
  // now main processing loop. Process received Command packets arriving at port 1024
  // these are identified by the command byte (byte 4)
  // cmd=00: general packet
  // cmd=02: discovery
  // cmd=03: set IP address (not supported)
  // cmd=04: erase (not supported)
  // cmd=05: program (not supported)
  //
  while(!ExitRequested)
  {
    memset(&iovecinst, 0, sizeof(struct iovec));
    memset(&datagram, 0, sizeof(datagram));
    iovecinst.iov_base = &UDPInBuffer;                  // set buffer for incoming message number i
    iovecinst.iov_len = VDISCOVERYSIZE;
    datagram.msg_iov = &iovecinst;
    datagram.msg_iovlen = 1;
    datagram.msg_name = &addr_from;
    datagram.msg_namelen = sizeof(addr_from);
    size = recvmsg(SocketData[0].Socketid, &datagram, 0);         // get one message. If it times out, gets size=-1
    if(size < 0 && errno != EAGAIN)
    {
      perror("recvfrom, port 1024");
      return NULL;
    }
    if(ThreadError)
      break;

//
// only process packets of length 60 bytes on this port, to exclude protocol 1 discovery for example.
// (that means we can't handle the programming packet but we don't use that anyway)
//
    CmdByte = UDPInBuffer[4];
    if(size==VDISCOVERYSIZE)  
    {
      NewMessageReceived = true;
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
          //RRK, not network client
          //saturn_handle_general_packet(UDPInBuffer);
          ReplyAddressSet = true;
          if(ReplyAddressSet && StartBitReceived)
            ServerActive = true;                                       // only set active if we have start bit too
          break;

        //
        // discovery packet
        //
        case 2:
          printf("P2 Discovery packet\n");

          if(ServerActive)
            DiscoveryReply[4] = 3;                             // response 2 if not active, 3 if running
          else
            DiscoveryReply[4] = 2;

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
    }
//
// now do any "post packet" processing
//
  } //while(1)
  if(ThreadError)
    printf("Thread error reported - exiting\n");
  //
  // clean exit
  //
  printf("Exiting\n");
  Shutdown();
  return NULL;
}


//
// listener thread for incoming high priority packets
//
void *IncomingHighPriority(void *arg)                   // listener thread
{
  struct ThreadSocketData *ThreadData;                  // socket etc data for this thread
  struct sockaddr_in addr_from;                         // holds MAC address of source of incoming messages
  uint8_t UDPInBuffer[VHIGHPRIOTIYTOSDRSIZE];           // incoming buffer
  struct iovec iovecinst;                               // iovcnt buffer - 1 for each outgoing buffer
  struct msghdr datagram;                               // multiple incoming message header
  int size;                                             // UDP datagram length
  bool RunBit;                                          // true if "run" bit set
  uint32_t DDCPhaseIncrement;                           // delta phase for a DDC
  uint8_t Byte, Byte2;                                  // received dat being decoded
  uint32_t LongWord;
  uint16_t Word;
  int i;                                                // counter

  ThreadData = (struct ThreadSocketData *)arg;
  ThreadData->Active = true;
  printf("spinning up high priority incoming thread with port %d\n", ThreadData->Portid);

  //
  // main processing loop
  //
  while(!ExitRequested)
  {
    memset(&iovecinst, 0, sizeof(struct iovec));
    memset(&datagram, 0, sizeof(datagram));
    iovecinst.iov_base = &UDPInBuffer;                  // set buffer for incoming message number i
    iovecinst.iov_len = VHIGHPRIOTIYTOSDRSIZE;
    datagram.msg_iov = &iovecinst;
    datagram.msg_iovlen = 1;
    datagram.msg_name = &addr_from;
    datagram.msg_namelen = sizeof(addr_from);
    size = recvmsg(ThreadData->Socketid, &datagram, 0);         // get one message. If it times out, ges size=-1
    if(size < 0 && errno != EAGAIN)
    {
      perror("recvfrom, high priority");
      printf("error number = %d\n", errno);
      return NULL;
    }

    //
    // if correct packet, process it
    //
    if(size == VHIGHPRIOTIYTOSDRSIZE)
    {
      NewMessageReceived = true;
      printf("high priority packet received\n");
      Byte = (uint8_t)(UDPInBuffer[4]);
      RunBit = (bool)(Byte&1);
      if(RunBit)
      {
        StartBitReceived = true;
        if(ReplyAddressSet && StartBitReceived)
          ServerActive = true;                                       // only set active if we have replay address too
      }
      else
      {
        ServerActive = false;                                       // set state of whole app
        printf("set to inactive by client app\n");
        StartBitReceived = false;
      }
//
// now properly decode DDC frequencies
//
      for (i=0; i<VNUMDDC/2; i++)
      {
        LongWord = ntohl(*(uint32_t *)(UDPInBuffer+i*4+9));
        SetDDCFrequency(i+VNUMDDC/2, LongWord, true);                   // temporarily set above
      }
#if 0 //RRK, not network client
      IsTXMode = (bool)(Byte&2);
      SetMOX(IsTXMode);

      //
      // DUC frequency & drive level
      //
      LongWord = ntohl(*(uint32_t *)(UDPInBuffer+329));
      SetDUCFrequency(LongWord, true);
      Byte = (uint8_t)(UDPInBuffer[345]);
      SetTXDriveLevel(Byte);
      //
      // transverter, speaker mute, open collector, user outputs
      //
      Byte = (uint8_t)(UDPInBuffer[1400]);
      SetXvtrEnable((bool)(Byte&1));
//      SetSpkrMute((bool)((Byte>>1)&1));
      Byte = (uint8_t)(UDPInBuffer[1401]);
      SetOpenCollectorOutputs(Byte);
      Byte = (uint8_t)(UDPInBuffer[1402]);
      SetUserOutputBits(Byte);
      //
      // Alex
      //
      Word = ntohs(*(uint16_t *)(UDPInBuffer+1430));
      AlexManualRXFilters(Word, 2);
      Word = ntohs(*(uint16_t *)(UDPInBuffer+1432));
      AlexManualTXFilters(Word);
      Word = ntohs(*(uint16_t *)(UDPInBuffer+1434));
      AlexManualRXFilters(Word, 0);
      //
      // RX atten during TX and RX
      //
      Byte2 = (uint8_t)(UDPInBuffer[1442]);     // RX2 atten
      Byte = (uint8_t)(UDPInBuffer[1443]);      // RX1 atten
      SetADCAttenuator(eADC1, Byte, true, true);
      SetADCAttenuator(eADC2, Byte2, true, true);
      //
      // CWX bits
      //
      Byte = (uint8_t)(UDPInBuffer[5]);      // CWX
      SetCWXBits((bool)(Byte & 1), (bool)((Byte>>2) & 1), (bool)((Byte>>1) & 1));    // enabled, dash, dot
#endif
    }
  }
//
// close down thread
//
  close(ThreadData->Socketid);                  // close incoming data socket
  ThreadData->Socketid = 0;
  ThreadData->Active = false;                   // indicate it is closed
  return NULL;
}


//
// listener thread for incoming DDC specific packets
//
void *IncomingDDCSpecific(void *arg)                    // listener thread
{
  struct ThreadSocketData *ThreadData;                  // socket etc data for this thread
  struct sockaddr_in addr_from;                         // holds MAC address of source of incoming messages
  uint8_t UDPInBuffer[VDDCSPECIFICSIZE];                // incoming buffer
  struct iovec iovecinst;                               // iovcnt buffer - 1 for each outgoing buffer
  struct msghdr datagram;                               // multiple incoming message header
  int size;                                             // UDP datagram length
  uint8_t Byte1, Byte2;                                 // received data
  bool Dither, Random;                                  // ADC bits
  bool Enabled, Interleaved;                            // DDC settings
  uint16_t Word, Word2;                                 // 16 bit read value
  int i;                                                // counter
  EADCSelect ADC = eADC1;                               // ADC to use for a DDC

  ThreadData = (struct ThreadSocketData *)arg;
  ThreadData->Active = true;
  printf("spinning up DDC specific thread with port %d\n", ThreadData->Portid);
  //
  // main processing loop
  //
  while(!ExitRequested)
  {
    memset(&iovecinst, 0, sizeof(struct iovec));
    memset(&datagram, 0, sizeof(datagram));
    iovecinst.iov_base = &UDPInBuffer;                  // set buffer for incoming message number i
    iovecinst.iov_len = VDDCSPECIFICSIZE;
    datagram.msg_iov = &iovecinst;
    datagram.msg_iovlen = 1;
    datagram.msg_name = &addr_from;
    datagram.msg_namelen = sizeof(addr_from);
    size = recvmsg(ThreadData->Socketid, &datagram, 0);         // get one message. If it times out, ges size=-1
    if(size < 0 && errno != EAGAIN)
    {
      perror("recvfrom, DDC Specific");
      return NULL;
    }
    if(size == VDDCSPECIFICSIZE)
    {
      NewMessageReceived = true;
      printf("DDC specific packet received\n");
#if 0
      // get ADC details:
      Byte1 = *(uint8_t*)(UDPInBuffer+4);                   // get ADC count
      SetADCCount(Byte1);
      Byte1 = *(uint8_t*)(UDPInBuffer+5);                   // get ADC Dither bits
      Byte2 = *(uint8_t*)(UDPInBuffer+6);                   // get ADC Random bits
      Dither  = (bool)(Byte1&1);
      Random  = (bool)(Byte2&1);
      SetADCOptions(eADC1, false, Dither, Random);          // ADC1 settings
      Byte1 = Byte1 >> 1;                                   // move onto ADC bits
      Byte2 = Byte2 >> 1;
      Dither  = (bool)(Byte1&1);
      Random  = (bool)(Byte2&1);
      SetADCOptions(eADC2, false, Dither, Random);          // ADC2 settings
#endif  
      //
      // main settings for each DDC
      // reuse "dither" for interleaved with next;
      // reuse "random" for DDC enabled.
      // be aware an interleaved "odd" DDC will usually be set to disabled, and we need to revert this!
      //
      Word = *(uint16_t*)(UDPInBuffer + 7);                 // get DDC enables 15:0 (note it is already low byte 1st!)
      for(i=0; i<VNUMDDC/2; i++)
      {
        Enabled = (bool)(Word & 1);                        // get enable state
        Byte1 = *(uint8_t*)(UDPInBuffer+i*6+17);          // get ADC for this DDC
        Word2 = *(uint16_t*)(UDPInBuffer+i*6+18);         // get sample rate for this DDC
        Word2 = ntohs(Word2);                             // swap byte order
        Byte2 = *(uint8_t*)(UDPInBuffer+i*6+22);          // get sample size for this DDC
        SetDDCSampleSize(i+VNUMDDC/2, Byte2);
        if(Byte1 == 0)
          ADC = eADC1;
        else if(Byte1 == 1)
          ADC = eADC2;
        else if(Byte1 == 2)
          ADC = eTXSamples;
        SetDDCADC(i+VNUMDDC/2, ADC);

        Interleaved = false;                                 // assume no synch
        // finally DDC synchronisation: my implementation it seems isn't what the spec intended!
        // check: is DDC1 programmed to sync with DDC0;
        // check: is DDC3 programmed to sync with DDC2;
        // check: is DDC5 programmed to sync with DDC4;
        // check: is DDC7 programmed to sync with DDC6;
        // check: if DDC1 synch to DDC0, enable it;
        // check: if DDC3 synch to DDC2, enable it;
        // check: if DDC5 synch to DDC4, enable it;
        // check: if DDC7 synch to DDC6, enable it;
        // (reuse the Dither variable)
        switch(i)
        {
            case 0:
                Byte1 = *(uint8_t*)(UDPInBuffer + 1363);          // get DDC0 synch
                if (Byte1 == 0b00000010)
                    Interleaved = true;                                // set interleave
                break;

            case 1: 
                Byte1 = *(uint8_t*)(UDPInBuffer + 1363);          // get DDC0 synch
                if (Byte1 == 0b00000010)                          // if synch to DDC1
                    Enabled = true;                                // enable DDC1
                break;

            case 2:
                Byte1 = *(uint8_t*)(UDPInBuffer + 1365);          // get DDC2 synch
                if (Byte1 == 0b00001000)
                    Interleaved = true;                                // set interleave
                break;

            case 3:
                Byte1 = *(uint8_t*)(UDPInBuffer + 1365);          // get DDC2 synch
                if (Byte1 == 0b00001000)                          // if synch to DDC3
                    Enabled = true;                                // enable DDC3
                break;

            case 4:
                Byte1 = *(uint8_t*)(UDPInBuffer + 1367);          // get DDC4 synch
                if (Byte1 == 0b00100000)
                    Interleaved = true;                                // set interleave
                break;
        
            case 5:
                Byte1 = *(uint8_t*)(UDPInBuffer + 1367);          // get DDC4 synch
                if (Byte1 == 0b00100000)                          // if synch to DDC5
                    Enabled = true;                                // enable DDC5
                break;

            case 6:
                Byte1 = *(uint8_t*)(UDPInBuffer + 1369);          // get DDC6 synch
                if (Byte1 == 0b10000000)
                    Interleaved = true;                                // set interleave
                break;

            case 7:
                Byte1 = *(uint8_t*)(UDPInBuffer + 1369);          // get DDC6 synch
                if (Byte1 == 0b10000000)                          // if synch to DDC7
                    Enabled = true;                                // enable DDC7
                break;

        }
        SetP2SampleRate(i+VNUMDDC/2, Enabled, Word2, Interleaved);
        Word = Word >> 1;                                 // move onto next DDC enabled bit
      }
      // now set register, and see if any changes made; reuse Dither again
      Dither = WriteP2DDCRateRegister();
      //if (Dither)
      //  HandlerCheckDDCSettings();
    }
  }
//
// close down thread
//
  close(ThreadData->Socketid);                  // close incoming data socket
  ThreadData->Socketid = 0;
  ThreadData->Active = false;                   // indicate it is closed
  return NULL;
}


//
// listener thread for incoming DUC specific packets
//
void *IncomingDUCSpecific(void *arg)                    // listener thread
{ 
    struct ThreadSocketData *ThreadData;                  // socket etc data for this thread
    struct sockaddr_in addr_from;                         // holds MAC address of source of incoming messages
    uint8_t UDPInBuffer[VDUCSPECIFICSIZE];                // incoming buffer
    struct iovec iovecinst;                               // iovcnt buffer - 1 for each outgoing buffer
    struct msghdr datagram;                               // multiple incoming message header
    int size;                                             // UDP datagram length
    uint8_t Byte;
    uint16_t SidetoneFreq;                                // freq for audio sidetone
    uint8_t IambicSpeed;                                  // WPM
    uint8_t IambicWeight;                                 //
    uint8_t SidetoneVolume;
    uint8_t CWRFDelay;
    uint16_t CWHangDelay;

    ThreadData = (struct ThreadSocketData *)arg;
    ThreadData->Active = true;
    printf("spinning up DUC specific thread with port %d\n", ThreadData->Portid);
    //
    // main processing loop
    //
    while(!ExitRequested)
    {
      memset(&iovecinst, 0, sizeof(struct iovec));
      memset(&datagram, 0, sizeof(datagram));
      iovecinst.iov_base = &UDPInBuffer;                  // set buffer for incoming message number i
      iovecinst.iov_len = VDUCSPECIFICSIZE;
      datagram.msg_iov = &iovecinst;
      datagram.msg_iovlen = 1;
      datagram.msg_name = &addr_from;
      datagram.msg_namelen = sizeof(addr_from);
      size = recvmsg(ThreadData->Socketid, &datagram, 0);         // get one message. If it times out, ges size=-1
      if(size < 0 && errno != EAGAIN)
      {
          perror("recvfrom, DUC specific");
          return NULL;
      }
      if(size == VDUCSPECIFICSIZE)
      {
          NewMessageReceived = true;
          printf("DUC specific packet received\n");
          continue; //RRK, not network client
// iambic settings
          IambicSpeed = *(uint8_t*)(UDPInBuffer+9);               // keyer speed
          IambicWeight = *(uint8_t*)(UDPInBuffer+10);             // keyer weight
          Byte = *(uint8_t*)(UDPInBuffer+5);                      // keyer bool bits
          SetCWIambicKeyer(IambicSpeed, IambicWeight, (bool)((Byte >> 2)&1), (bool)((Byte >> 5)&1), 
                          (bool)((Byte >> 6)&1), (bool)((Byte >> 3)&1), (bool)((Byte >> 7)&1));
// general CW settings
          SetCWSidetoneEnabled((bool)((Byte >> 4)&1));
          EnableCW((bool)((Byte >> 1)&1));                        // CW enabled bit
          SidetoneVolume = *(uint8_t*)(UDPInBuffer+6);            // keyer speed
          SidetoneFreq = *(uint16_t*)(UDPInBuffer+7);             // get frequency
          SidetoneFreq = ntohs(SidetoneFreq);                     // convert from big endian
          SetCWSidetoneVol(SidetoneVolume);
          SetCWSidetoneFrequency(SidetoneFreq);
          CWRFDelay = *(uint8_t*)(UDPInBuffer+13);                // delay before CW on
          CWHangDelay = *(uint16_t*)(UDPInBuffer+11);             // delay before CW off
          CWHangDelay = ntohs(CWHangDelay);                       // convert from big endian
          SetCWPTTDelay(CWRFDelay);
          SetCWHangTime(CWHangDelay);
// mic and line in options
          Byte = *(uint8_t*)(UDPInBuffer+50);                     // mic/line options
          SetMicBoost((bool)((Byte >> 1)&1));
          SetMicLineInput((bool)(Byte&1));
          SetOrionMicOptions((bool)((Byte >> 3)&1), (bool)((Byte >> 4)&1), (bool)((~Byte >> 2)&1));          
          Byte = *(uint8_t*)(UDPInBuffer+51);                     // line in gain
          SetCodecLineInGain(Byte);
      }
    }
//
// close down thread
//
    close(ThreadData->Socketid);                  // close incoming data socket
    ThreadData->Socketid = 0;
    ThreadData->Active = false;                   // indicate it is closed
    return NULL;
}


#define VSPKSAMPLESPERFRAME 64                      // samples per UDP frame
#define VMEMWORDSPERFRAME 32                        // 8 byte writes per UDP msg
#define VSPKSAMPLESPERMEMWORD 2                     // 2 samples (each 4 bytres) per 8 byte word
#define VDMABUFFERSIZE 32768						// memory buffer to reserve
#define VALIGNMENT 4096                             // buffer alignment
#define VBASE 0x1000								// DMA start at 4K into buffer
#define VDMATRANSFERSIZE 256                        // write 1 message at a time


//
// listener thread for incoming DDC (speaker) audio packets
// planned strategy: just DMA spkr data when available; don't copy and DMA a larger amount.
// if sufficient FIFO data available: DMA that data and transfer it out. 
// if it turns out to be too inefficient, we'll have to try larger DMA.
//
void *IncomingSpkrAudio(void *arg)                      // listener thread
{
    struct ThreadSocketData *ThreadData;                  // socket etc data for this thread
    struct sockaddr_in addr_from;                         // holds MAC address of source of incoming messages
    uint8_t UDPInBuffer[VSPEAKERAUDIOSIZE];               // incoming buffer
    struct iovec iovecinst;                               // iovcnt buffer - 1 for each outgoing buffer
    struct msghdr datagram;                               // multiple incoming message header
    int size;                                             // UDP datagram length

//
// variables for DMA buffer 
//
    uint8_t* SpkWriteBuffer = NULL;							// data for DMA to write to spkr
    uint32_t SpkBufferSize = VDMABUFFERSIZE;
    bool InitError = false;                                 // becomes true if we get an initialisation error
    unsigned char* SpkReadPtr;								// pointer for reading out a spkr sample
    unsigned char* SpkHeadPtr;								// ptr to 1st free location in spk memory
    unsigned char* SpkBasePtr;								// ptr to DMA location in spk memory
    uint32_t Depth = 0;
    int DMAWritefile_fd = -1;								// DMA read file device
    bool FIFOOverflow;
    uint32_t RegVal;

    ThreadData = (struct ThreadSocketData *)arg;
    ThreadData->Active = true;
    printf("spinning up speaker audio thread with port %d\n", ThreadData->Portid);

    //
    // setup DMA buffer
    //
    posix_memalign((void**)&SpkWriteBuffer, VALIGNMENT, SpkBufferSize);
    if (!SpkWriteBuffer)
    {
        printf("spkr write buffer allocation failed\n");
        InitError = true;
    }
    SpkReadPtr = SpkWriteBuffer + VBASE;							// offset 4096 bytes into buffer
    SpkHeadPtr = SpkWriteBuffer + VBASE;
    SpkBasePtr = SpkWriteBuffer + VBASE;
    memset(SpkWriteBuffer, 0, SpkBufferSize);

    //
    // open DMA device driver
    //
    DMAWritefile_fd = open(VSPKDMADEVICE, O_RDWR);
    if (DMAWritefile_fd < 0)
    {
        printf("XDMA write device open failed for spk data\n");
        InitError = true;
    }
	ResetDMAStreamFIFO(eSpkCodecDMA);

  //
  // main processing loop
  //
    while(!ExitRequested)
    {
        memset(&iovecinst, 0, sizeof(struct iovec));            // clear buffers
        memset(&datagram, 0, sizeof(datagram));
        iovecinst.iov_base = &UDPInBuffer;                      // set buffer for incoming message number i
        iovecinst.iov_len = VSPEAKERAUDIOSIZE;
        datagram.msg_iov = &iovecinst;
        datagram.msg_iovlen = 1;
        datagram.msg_name = &addr_from;
        datagram.msg_namelen = sizeof(addr_from);
        size = recvmsg(ThreadData->Socketid, &datagram, 0);     // get one message. If it times out, sets size=-1
        if(size < 0 && errno != EAGAIN)
        {
            perror("recvfrom fail, Speaker data");
            return NULL;
        }
        if(size == VSPEAKERAUDIOSIZE)                           // we have received a packet!
        {
            NewMessageReceived = true;
            RegVal += 1;            //debug
            Depth = ReadFIFOMonitorChannel(eSpkCodecDMA, &FIFOOverflow);        // read the FIFO free locations
            printf("speaker packet received; depth = %d\n", Depth);
            while (Depth < VMEMWORDSPERFRAME)       // loop till space available
            {
                usleep(1000);								                    // 1ms wait
                Depth = ReadFIFOMonitorChannel(eSpkCodecDMA, &FIFOOverflow);    // read the FIFO free locations
            }
            // copy sata from UDP Buffer & DMA write it
            memcpy(SpkBasePtr, UDPInBuffer + 4, VDMATRANSFERSIZE);              // copy out spk samples
//            if(RegVal == 100)
//                DumpMemoryBuffer(SpkBasePtr, VDMATRANSFERSIZE);
            DMAWriteToFPGA(DMAWritefile_fd, SpkBasePtr, VDMATRANSFERSIZE, VADDRSPKRSTREAMWRITE);
        }
    }
//
// close down thread
//
    close(ThreadData->Socketid);                  // close incoming data socket
    ThreadData->Socketid = 0;
    ThreadData->Active = false;                   // indicate it is closed
    return NULL;
}


#define VIQSAMPLESPERFRAME 240                      // samples per UDP frame
#define VMEMDUCWORDSPERFRAME 180                       // memory writes per UDP frame
#define VBYTESPERSAMPLE 6							// 24 bit + 24 bit samples
#define VDMABUFFERSIZE 32768						// memory buffer to reserve
#define VALIGNMENT 4096                             // buffer alignment
#define VBASE 0x1000								// DMA start at 4K into buffer
#define VDMADUCTRANSFERSIZE 1440                       // write 1 message at a time

//
// listener thread for incoming DUC I/Q packets
// planned strategy: just DMA spkr data when available; don't copy and DMA a larger amount.
// if sufficient FIFO data available: DMA that data and transfer it out. 
// if it turns out to be too inefficient, we'll have to try larger DMA.
//
void *IncomingDUCIQ(void *arg)                          // listener thread
{
    struct ThreadSocketData *ThreadData;                  // socket etc data for this thread
    struct sockaddr_in addr_from;                         // holds MAC address of source of incoming messages
    uint8_t UDPInBuffer[VDUCIQSIZE];                      // incoming buffer
    struct iovec iovecinst;                               // iovcnt buffer - 1 for each outgoing buffer
    struct msghdr datagram;                               // multiple incoming message header
    int size;                                             // UDP datagram length

                                                          //
// variables for DMA buffer 
//
    uint8_t* IQWriteBuffer = NULL;							// data for DMA to write to DUC
    uint32_t IQBufferSize = VDMABUFFERSIZE;
    bool InitError = false;                                 // becomes true if we get an initialisation error
    unsigned char* IQReadPtr;								// pointer for reading out an I/Q sample
    unsigned char* IQHeadPtr;								// ptr to 1st free location in I/Q memory
    unsigned char* IQBasePtr;								// ptr to DMA location in I/Q memory
    uint32_t Depth = 0;
    int DMAWritefile_fd = -1;								// DMA read file device
    bool FIFOOverflow;
    uint32_t Cntr;                                          // sample counter
    uint8_t* SrcPtr;                                        // pointer to data from Thetis
    uint8_t* DestPtr;                                       // pointer to DMA buffer data

    ThreadData = (struct ThreadSocketData *)arg;
    ThreadData->Active = true;
    printf("spinning up DUC I/Q thread with port %d\n", ThreadData->Portid);
  
    //
    // setup DMA buffer
    //
    posix_memalign((void**)&IQWriteBuffer, VALIGNMENT, IQBufferSize);
    if (!IQWriteBuffer)
    {
        printf("I/Q TX write buffer allocation failed\n");
        InitError = true;
    }
    IQReadPtr = IQWriteBuffer + VBASE;							// offset 4096 bytes into buffer
    IQHeadPtr = IQWriteBuffer + VBASE;
    IQBasePtr = IQWriteBuffer + VBASE;
    memset(IQWriteBuffer, 0, IQBufferSize);

    //
    // open DMA device driver
    //
    DMAWritefile_fd = open(VDUCDMADEVICE, O_RDWR);
    if (DMAWritefile_fd < 0)
    {
        printf("XDMA write device open failed for TX I/Q data\n");
        InitError = true;
    }
        
//
// setup hardware
//
    EnableDUCMux(false);                                  // disable temporarily
    SetTXIQDeinterleaved(false);                          // not interleaved (at least for now!)
    ResetDUCMux();                                        // reset 64 to 48 mux
    ResetDMAStreamFIFO(eTXDUCDMA);
    EnableDUCMux(true);                                   // enable operation

  //
  // main processing loop
  //
    while(!ExitRequested)
    {
        memset(&iovecinst, 0, sizeof(struct iovec));
        memset(&datagram, 0, sizeof(datagram));
        iovecinst.iov_base = &UDPInBuffer;                  // set buffer for incoming message number i
        iovecinst.iov_len = VDUCIQSIZE;
        datagram.msg_iov = &iovecinst;
        datagram.msg_iovlen = 1;
        datagram.msg_name = &addr_from;
        datagram.msg_namelen = sizeof(addr_from);
        size = recvmsg(ThreadData->Socketid, &datagram, 0);         // get one message. If it times out, ges size=-1
        if(size < 0 && errno != EAGAIN)
        {
            perror("recvfrom fail, TX I/Q data");
            return NULL;
        }
        if(size == VDUCIQSIZE)
        {
            printf("DUC I/Q packet received\n");
            NewMessageReceived = true;
            Depth = ReadFIFOMonitorChannel(eTXDUCDMA, &FIFOOverflow);           // read the FIFO free locations
            while (Depth < VMEMDUCWORDSPERFRAME)       // loop till space available
            {
                usleep(500);								                    // 0.5ms wait
                Depth = ReadFIFOMonitorChannel(eTXDUCDMA, &FIFOOverflow);       // read the FIFO free locations
            }
            // copy data from UDP Buffer & DMA write it
//            memcpy(IQBasePtr, UDPInBuffer + 4, VDMADUCTRANSFERSIZE);                // copy out I/Q samples
            // need to swap I & Q samples on replay
            SrcPtr = UDPInBuffer + 4;
            DestPtr = IQBasePtr;
            for (Cntr=0; Cntr < VIQSAMPLESPERFRAME; Cntr++)                     // samplecounter
            {
                *DestPtr++ = *(SrcPtr+3);                           // get I sample (3 bytes)
                *DestPtr++ = *(SrcPtr+4);
                *DestPtr++ = *(SrcPtr+5);
                *DestPtr++ = *(SrcPtr+0);                           // get Q sample (3 bytes)
                *DestPtr++ = *(SrcPtr+1);
                *DestPtr++ = *(SrcPtr+2);
                SrcPtr += 6;                                        // point at next source sample
            }
            DMAWriteToFPGA(DMAWritefile_fd, IQBasePtr, VDMADUCTRANSFERSIZE, VADDRDUCSTREAMWRITE);
        }
    }
//
// close down thread
//
    close(ThreadData->Socketid);                  // close incoming data socket
    ThreadData->Socketid = 0;
    ThreadData->Active = false;                   // indicate it is closed
    return NULL;
}


//
// interface calls to get commands from PC settings
// sample rate, DDC enabled and interleaved are all signalled through the socket 
// data structure
//
// the meanings are:
// enabled - the DDC sends data in its own right
// interleaved - can be set for "even" DDCs; the next higher odd DDC also has its data routed
// through here. That DDC is NOT enabled. 
//


//
// HandlerCheckDDCSettings()
// called when DDC settings have been changed. Check which DDCs are enabled, and sample rate.
// arguably don't need this, as it finds out from the embedded data in the DDC stream
//
void HandlerCheckDDCSettings(void)
{

}
//
// HandlerSetEERMode (bool EEREnabled)
// enables amplitude restoration mode. Generates envelope output alongside I/Q samples.
// NOTE hardware does not properly support this yet!
// TX FIFO must be empty. Stop multiplexer; set bit; restart
// 
void HandlerSetEERMode(bool EEREnabled)
{

}
