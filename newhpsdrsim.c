#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sys/socket.h>
#include <errno.h>
#include <string.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <math.h>

extern struct sockaddr_in addr_new;
extern void audio_write(int16_t r, int16_t l);

#define NUMRECEIVERS 8

#define LENNOISE 192000
#define NOISEDIV (RAND_MAX / 96000)

extern double noiseItab[LENNOISE];
extern double noiseQtab[LENNOISE];

#define IM3a  0.60
#define IM3b  0.20

#define RTXLEN 64512
#define NEWRTXLEN 64320
extern double  isample[RTXLEN];  // shared with newhpsdrsim
extern double  qsample[RTXLEN];  // shared with newhpsdrsim
static int txptr = 10000;

/*
 * These variables represent the state of the machine
 */
// data from general packet
static int ddc_port = 0;
static int duc_port = 0;
static int hp_port = 0;
static int shp_port = 0;
static int audio_port = 0;
static int duc0_port = 0;
static int ddc0_port = 0;
static int mic_port = 0;
static int wide_port = 0;
static int wide_enable = 0;
static int wide_len = 0;
static int wide_size = 0;
static int wide_rate = 0;
static int wide_ppf = 0;
static int port_mm = 0;
static int port_smm = 0;
static int pwm_min = 0;
static int pwm_max = 0;
static int bits = 0;
static int hwtim = 0;
static int pa_enable = 0;
static int alex0_enable = 0;
static int alex1_enable = 0;
static int mm_port = 0;
static int smm_port = 0;
static int iqform = 0;

// data from rx specific packet
static int adc=0;
static int adcdither[8];
static int adcrandom[8];
static int ddcenable[NUMRECEIVERS];
static int adcmap[NUMRECEIVERS];
static int rxrate[NUMRECEIVERS];
static int syncddc[NUMRECEIVERS];

//data from tx specific packet
static int dac=0;
static int cwmode=0;
static int sidelevel=0;
static int sidefreq=0;
static int speed=0;
static int weight=0;
static int hang=0;
static int delay=0;
static int txrate=0;
static int ducbits=0;
static int orion=0;
static int gain=0;
static int txatt=0;

//stat from high-priority packet
static int run=0;
static int ptt[4];
static int cwx=0;
static int dot=0;
static int dash=0;
static unsigned long rxfreq[NUMRECEIVERS];
static unsigned long txfreq=0;
static int txdrive=0;
static int w1400=0;  // Xvtr and Audio enable
static int ocout=0;
static int db9=0;
static int mercury_atts=0;
static int alex0[32];
static int alex1[32];
static int stepatt0=0;
static int stepatt1=0;

//
// floating point representation of TX-Drive and ADC0-Attenuator
//
static double rxatt0_dbl=1.0;
static double rxatt1_dbl=1.0;
static double txatt_dbl=1.0;
static double txdrv_dbl = 0.0;

// End of state variables

static pthread_t ddc_specific_thread_id;
static pthread_t duc_specific_thread_id;
static pthread_t rx_thread_id[NUMRECEIVERS];
static pthread_t tx_thread_id;
static pthread_t mic_thread_id;
static pthread_t audio_thread_id;
static pthread_t highprio_thread_id = 0;
static pthread_t send_highprio_thread_id;

void   *ddc_specific_thread(void*);
void   *duc_specific_thread(void*);
void   *highprio_thread(void*);
void   *send_highprio_thread(void*);
void   *rx_thread(void *);
void   *tx_thread(void *);
void   *mic_thread(void *);
void   *audio_thread(void *);

static double txlevel;

int new_protocol_running() {
  if (run) return 1; else return 0;
}

void new_protocol_general_packet(unsigned char *buffer) {
  static unsigned long seqnum=0;
  unsigned long seqold;
  int rc;

  seqold = seqnum;
  seqnum = (buffer[0] >> 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
  if (seqnum != 0 && seqnum != seqold+1 ) {
    fprintf(stderr,"GP: SEQ ERROR, old=%lu new=%lu\n", seqold, seqnum);
  }

  rc=(buffer[5] << 8) + buffer[6];
  if (rc == 0) rc=1025;
  if (rc != ddc_port) {
    ddc_port=rc;
    fprintf(stderr,"GP: RX specific rcv        port is  %4d\n", rc);
  }
  rc=(buffer[7] << 8) + buffer[8];
  if (rc == 0) rc=1026;
  if (rc != duc_port) {
    duc_port=rc;
    fprintf(stderr,"GP: TX specific rcv        port is  %4d\n", rc);
  }
  rc=(buffer[9] << 8) + buffer[10];
  if (rc == 0) rc=1027;
  if (rc != hp_port) {
    hp_port=rc;
    fprintf(stderr,"GP: HighPrio Port rcv      port is  %4d\n", rc);
  }
  rc=(buffer[11] << 8) + buffer[12];
  if (rc == 0) rc=1025;
  if (rc != shp_port) {
    shp_port=rc;
    fprintf(stderr,"GP: HighPrio Port snd      port is  %4d\n", rc);
  }
  rc=(buffer[13] << 8) + buffer[14];
  if (rc == 0) rc=1028;
  if (rc != audio_port) {
    audio_port=rc;
    fprintf(stderr,"GP: Audio rcv              port is  %4d\n", rc);
  }
  rc=(buffer[15] << 8) + buffer[16];
  if (rc == 0) rc=1029;
  if (rc != duc0_port) {
    duc0_port=rc;
    fprintf(stderr,"GP: TX data rcv base       port is  %4d\n", rc);
  }
  rc=(buffer[17] << 8) + buffer[18];
  if (rc == 0) rc=1035;
  if (rc != ddc0_port) {
    ddc0_port=rc;
    fprintf(stderr,"GP: RX data snd base       port is  %4d\n", rc);
  }
  rc=(buffer[19] << 8) + buffer[20];
  if (rc == 0) rc=1026;
  if (rc != mic_port) {
    mic_port=rc;
    fprintf(stderr,"GP: Microphone data snd    port is  %4d\n", rc);
  }
  rc=(buffer[21] << 8) + buffer[22];
  if (rc == 0) rc=1027;
  if (rc != wide_port) {
    wide_port=rc;
    fprintf(stderr,"GP: Wideband data snd       port is  %4d\n", rc);
  }
  rc=buffer[23]; 
  if (rc != wide_enable) {
    wide_enable = rc;
    fprintf(stderr,"GP: Wideband Enable Flag is %d\n", rc);
  }
  rc=(buffer[24] << 8) + buffer[25]; if (rc == 0) rc=512;
  if (rc != wide_len) {
    wide_len=rc;
    fprintf(stderr,"GP: WideBand Length is %d\n", rc);
  }
  rc=buffer[26]; if (rc == 0) rc=16;
  if (rc != wide_size) {
    wide_size=rc;
    fprintf(stderr,"GP: Wideband sample size is %d\n", rc);
  }
  rc=buffer[27];
  if (rc != wide_rate) {
    wide_rate=rc;
    fprintf(stderr,"GP: Wideband sample rate is %d\n", rc);
  }
  rc=buffer[28];
  if (rc != wide_ppf) {
    wide_ppf = rc;
    fprintf(stderr,"GP: Wideband PPF is %d\n", rc);
  }
  rc = (buffer[29] << 8) + buffer[30];
  if (rc != port_mm) {
    port_mm=rc;
    fprintf(stderr,"MemMapped Registers rcv port is %d\n", rc);
  }
  rc = (buffer[31] << 8) + buffer[32];
  if (rc != port_smm) {
    port_smm=rc;
    fprintf(stderr,"MemMapped Registers snd port is %d\n", rc);
  }
  rc = (buffer[33] << 8) + buffer[34];
  if (rc != pwm_min) {
    pwm_min=rc;
    fprintf(stderr,"GP: PWM Min value is %d\n", rc);
  }
  rc = (buffer[35] << 8) + buffer[36];
  if (rc != pwm_max) {
    pwm_max=rc;
    fprintf(stderr,"GP: PWM Max value is %d\n", rc);
  }
  rc=buffer[37];
  if (rc != bits) {
    bits=rc;
    fprintf(stderr,"GP: ModeBits=x%02x\n", rc);
  }
  rc=buffer[38];
  if (rc != hwtim) {
    hwtim=rc;
    fprintf(stderr,"GP: Hardware Watchdog enabled=%d\n", rc);
  }

  iqform = buffer[39];				if (iqform == 0) iqform=3;
  if (iqform != 3) fprintf(stderr,"GP: Wrong IQ Format requested: %d\n",iqform);

  rc = (buffer[58] & 0x01);
  if (rc != pa_enable) {
    pa_enable=rc;
    fprintf(stderr,"GP: PA enabled=%d\n", rc);
  }

  rc=buffer[59] & 0x01;
  if (rc != alex0_enable) {
    alex0_enable=rc;
    fprintf(stderr,"GP: ALEX0 register enable=%d\n", rc);
  }
  rc=(buffer[59] & 0x02) >> 1;
  if (rc != alex1_enable) {
    alex1_enable=rc;
    fprintf(stderr,"GP: ALEX1 register enable=%d\n", rc);
  }
  //
  // Start HighPrio thread if we arrive here for the first time
  // The HighPrio thread keeps running all the time.
  //
  if (!highprio_thread_id) {
	if (pthread_create(&highprio_thread_id, NULL, highprio_thread, NULL) < 0) {
	  perror("***** ERROR: Create HighPrio thread");
	}
	pthread_detach(highprio_thread_id);

  //
  // init state arrays to zero for the first time
  //
  memset(adcdither, 0, 8*sizeof(int));
  memset(adcrandom, 0, 8*sizeof(int));
  memset(ddcenable, 0, NUMRECEIVERS*sizeof(int));
  memset(adcmap, 0, NUMRECEIVERS*sizeof(int));
  memset(syncddc, 0, NUMRECEIVERS*sizeof(int));

  memset(rxfreq, 0, NUMRECEIVERS*sizeof(unsigned long));
  memset(alex0, 0, 32*sizeof(int));
  memset(alex1, 0, 32*sizeof(int));
  memset(ptt  , 0, 4*sizeof(int));
  }
}

void *ddc_specific_thread(void *data) {
  int sock;
  struct sockaddr_in addr;
  socklen_t lenaddr = sizeof(addr);
  unsigned long seqnum,seqold;
  struct timeval tv;
  unsigned char buffer[2000];
  int yes = 1;
  int rc;
  int i,j;

  sock=socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    perror("***** ERROR: RX specific: socket");
    return NULL;
  }

  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));
  setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, (void *)&yes, sizeof(yes));
  tv.tv_sec = 0;
  tv.tv_usec = 10000;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (void *)&tv, sizeof(tv));


  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(ddc_port);

  if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("***** ERROR: RX specific: bind");
    return NULL;
  }

  while(run) {
     rc = recvfrom(sock, buffer, 1444, 0,(struct sockaddr *)&addr, &lenaddr);
     if (rc < 0 && errno != EAGAIN) {
       perror("***** ERROR: DDC specific thread: recvmsg");
       break;
     }
     if (rc < 0) continue;
     if (rc != 1444) {
       fprintf(stderr,"RX: Received DDC specific packet with incorrect length");
       break;
     }
     seqold = seqnum;
     seqnum = (buffer[0] >> 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
     if (seqnum != 0 &&seqnum != seqold+1 ) {
       fprintf(stderr,"GP: SEQ ERROR, old=%lu new=%lu\n", seqold, seqnum);
     }
     if (adc != buffer[4]) {
       adc=buffer[4];
       fprintf(stderr,"RX: Number of ADCs: %d\n",adc);
     }
     for (i=0; i<8; i++) {
       rc=(buffer[5] >> i) & 0x01;
       if (rc != adcdither[i]) {
         adcdither[i]=rc;
	 fprintf(stderr,"RX: ADC%d dither=%d\n",i,rc);
       }
     }
     for (i=0; i<8; i++) {
       rc=(buffer[6] >> i) & 0x01;
       if (rc != adcrandom[i]) {
         adcrandom[i]=rc;
	 fprintf(stderr,"RX: ADC%d random=%d\n",i,rc);
       }
     }

     for (i=0; i<NUMRECEIVERS; i++) {
       int modified=0;

       rc=(buffer[7 + (i/8)] >> (i % 8)) & 0x01;
       if (rc != ddcenable[i]) {
	 modified=1;
	 ddcenable[i]=rc;
       }
       
       rc=buffer[17+6*i];
       if (rc != adcmap[i]) {
	 modified=1;
	 adcmap[i]=rc;
       }

       rc=(buffer[18+6*i] << 8) + buffer[19+6*i];
       if (rc != rxrate[i]) {
	 modified=1;
	 rxrate[i]=rc;
         modified=1;
       }

       if (syncddc[i] != buffer[1363+i]) {
	 syncddc[i]=buffer[1363+i];
         modified=1;
       }
       rc=(buffer[7 + (i/8)] >> (i % 8)) & 0x01;
       if (rc != ddcenable[i]) {
	 modified=1;
	 ddcenable[i]=rc;
       }
       if (modified) {
	 fprintf(stderr,"RX: DDC%d Enable=%d ADC%d Rate=%d SyncMap=%02x\n",
		i,ddcenable[i], adcmap[i], rxrate[i], syncddc[i]);
       }
     }
  }
  close(sock);
  return NULL;
}

void *duc_specific_thread(void *data) {
  int sock;
  struct sockaddr_in addr;
  socklen_t lenaddr=sizeof(addr);
  unsigned long seqnum,seqold;
  struct timeval tv;
  unsigned char buffer[100];
  int yes = 1;
  int rc;

  sock=socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    perror("***** ERROR: TX specific: socket");
    return NULL;
  }

  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));
  setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, (void *)&yes, sizeof(yes));
  tv.tv_sec = 0;
  tv.tv_usec = 10000;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (void *)&tv, sizeof(tv));

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(duc_port);

  if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("***** ERROR: TX specific: bind");
    return NULL;
  }

  while(run) {
     rc = recvfrom(sock, buffer, 60, 0,(struct sockaddr *)&addr, &lenaddr);
     if (rc < 0 && errno != EAGAIN) {
       perror("***** ERROR: DUC specific thread: recvmsg");
       break;
     }
     if (rc < 0) continue;
     if (rc != 60) {
	fprintf(stderr,"TX: DUC Specific: wrong length\n");
        break;
     }
     seqold = seqnum;
     seqnum = (buffer[0] >> 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
     if (seqnum != 0 &&seqnum != seqold+1 ) {
       fprintf(stderr,"GP: SEQ ERROR, old=%lu new=%lu\n", seqold, seqnum);
     }
     if (dac != buffer[4]) {
	dac=buffer[4];
	fprintf(stderr,"TX: Number of DACs: %d\n", dac);
     }
     if (cwmode != buffer[5]) {
	cwmode=buffer[5];
	fprintf(stderr,"TX: CW mode bits = %x\n",cwmode);
     }
     if (sidelevel != buffer[6]) {
	sidelevel=buffer[6];
	fprintf(stderr,"TX: CW side tone level: %d\n", sidelevel);
     }
     rc=(buffer[7] << 8) + buffer[8];
     if (rc != sidefreq) {
	sidefreq = rc;
	fprintf(stderr,"TX: CW sidetone freq: %d\n", sidefreq);
     }
     if (speed != buffer[9]) {
	speed = buffer[9];
	fprintf(stderr,"TX: CW keyer speed: %d wpm\n", speed);
     }
     if (weight != buffer[10]) {
	weight=buffer[10];
	fprintf(stderr,"TX: CW weight: %d\n", weight);
     }
     rc=(buffer[11] << 8) + buffer[12];
     if (hang != rc) {
	hang = rc;
	fprintf(stderr,"TX: CW hang time: %d msec\n", hang);
     }
     if (delay != buffer[13]) {
	delay=buffer[13];
	fprintf(stderr,"TX: RF delay: %d msec\n", delay);
     }
     rc=(buffer[14] << 8) + buffer[15];
     if (txrate != rc) {
	txrate = rc;
	fprintf(stderr,"TX: DUC sample rate: %d\n", rc);
     }
     if (ducbits != buffer[16]) {
	ducbits=buffer[16];
	fprintf(stderr,"TX: DUC sample width: %d bits\n", ducbits);
     }
     if (orion != buffer[50]) {
	orion=buffer[50];
	fprintf(stderr,"TX: ORION bits (mic etc): %x\n", orion);
     }
     if (gain != buffer[51]) {
	gain= buffer[51];
	fprintf(stderr,"TX: LineIn Gain (dB): %f\n", 12.0 - 1.5*gain);
     }
     if (txatt != buffer[59]) {
	txatt = buffer[59];
	txatt_dbl=pow(10.0, -0.05*txatt);
	fprintf(stderr,"TX: ATT DUC0/ADC0: %d\n", txatt);
     }
  }
  close(sock);
  return NULL;
}

void *highprio_thread(void *data) {
  int sock;
  struct sockaddr_in addr;
  socklen_t lenaddr=sizeof(addr);
  unsigned long seqnum,seqold;
  unsigned char buffer[2000];
  struct timeval tv;
  int yes = 1;
  int rc;
  unsigned long freq;
  int i;

  sock=socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    perror("***** ERROR: HP: socket");
    return NULL;
  }

  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));
  setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, (void *)&yes, sizeof(yes));
  tv.tv_sec = 0;
  tv.tv_usec = 10000;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (void *)&tv, sizeof(tv));

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(hp_port);

  if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("***** ERROR: HP: bind");
    return NULL;
  }

  while(1) {
     rc = recvfrom(sock, buffer, 1444, 0,(struct sockaddr *)&addr, &lenaddr);
     if (rc < 0 && errno != EAGAIN) {
       perror("***** ERROR: HighPrio thread: recvmsg");
       break;
     }
     if (rc < 0) continue;
     if (rc != 1444) {
       fprintf(stderr,"Received HighPrio packet with incorrect length");
       break;
     }
     seqold = seqnum;
     seqnum = (buffer[0] >> 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
     if (seqnum != 0 &&seqnum != seqold+1 ) {
       fprintf(stderr,"GP: SEQ ERROR, old=%lu new=%lu\n", seqold, seqnum);
     }
     rc=(buffer[4] >> 0) & 0x01;
     if (rc != run) {
	run=rc;
	fprintf(stderr,"HP: Run=%d\n", rc);
        // if run=0, wait for threads to complete, otherwise spawn them off
        if (run) {
          if (pthread_create(&ddc_specific_thread_id, NULL, ddc_specific_thread, NULL) < 0) {
            perror("***** ERROR: Create DDC specific thread");
          }
          if (pthread_create(&duc_specific_thread_id, NULL, duc_specific_thread, NULL) < 0) {
            perror("***** ERROR: Create DUC specific thread");
          }
          for (i=0; i< NUMRECEIVERS; i++) {
            if (pthread_create(&rx_thread_id[i], NULL, rx_thread, (void *) (uintptr_t) i) < 0) {
              perror("***** ERROR: Create RX thread");
            }
	  }
          if (pthread_create(&tx_thread_id, NULL, tx_thread, NULL) < 0) {
            perror("***** ERROR: Create TX thread");
          }
          if (pthread_create(&send_highprio_thread_id, NULL, send_highprio_thread, NULL) < 0) {
            perror("***** ERROR: Create SendHighPrio thread");
          }
          if (pthread_create(&mic_thread_id, NULL, mic_thread, NULL) < 0) {
            perror("***** ERROR: Create Mic thread");
          }
          if (pthread_create(&audio_thread_id, NULL, audio_thread, NULL) < 0) {
            perror("***** ERROR: Create Audio thread");
          }
        } else {
          pthread_join(ddc_specific_thread_id, NULL);
          pthread_join(duc_specific_thread_id, NULL);
          for (i=0; i<NUMRECEIVERS; i++) {
            pthread_join(rx_thread_id[i], NULL);
          }
          pthread_join(send_highprio_thread_id, NULL);
          pthread_join(tx_thread_id, NULL);
          pthread_join(mic_thread_id, NULL);
          pthread_join(audio_thread_id, NULL);
        }
     }
     for (i=0; i<4; i++) {
       rc=(buffer[4] >> (i+1)) & 0x01;
       if (rc != ptt[i]) {
	ptt[i]=rc;
	fprintf(stderr,"HP: PTT%d=%d\n", i, rc);
      }
     }
     rc=(buffer[5] >> 0) & 0x01;
     if (rc != cwx) {
	cwx=rc;
	fprintf(stderr,"HP: CWX=%d\n", rc);
     }
     rc=(buffer[5] >> 1) & 0x01;
     if (rc != dot) {
	dot=rc;
	fprintf(stderr,"HP: DOT=%d\n", rc);
     }
     rc=(buffer[5] >> 2) & 0x01;
     if (rc != dash) {
	dash=rc;
	fprintf(stderr,"HP: DASH=%d\n", rc);
     }
     for (i=0; i<NUMRECEIVERS; i++) {
	freq=(buffer[ 9+4*i] << 24) + (buffer[10+4*i] << 16) + (buffer[11+4*i] << 8) + buffer[12+4*i];
        if (bits & 0x08) {
	  freq=round(122880000.0*(double) freq / 4294967296.0);
	}
        if (freq != rxfreq[i]) {
	  rxfreq[i]=freq;
	  fprintf(stderr,"HP: DDC%d freq: %lu\n", i, freq);
	}
     }
     freq=(buffer[329] << 24) + (buffer[330] << 16) + (buffer[331] << 8) + buffer[332];
     if (bits & 0x08) {
	freq=round(122880000.0*(double) freq / 4294967296.0);
     }
     if (freq != txfreq) {
	txfreq=freq;
	fprintf(stderr,"HP: DUC freq: %lu\n", freq);
     }
     rc=buffer[345];
     if (rc != txdrive) {
	txdrive=rc;
	txdrv_dbl=(double) txdrive * 0.003921568627;
	fprintf(stderr,"HP: TX drive= %d (%f)\n", txdrive,txdrv_dbl);
     }
     rc=buffer[1400];
     if (rc != w1400) {
	w1400=rc;
	fprintf(stderr,"HP: Xvtr/Audio enable=%x\n", rc);
     }
     rc=buffer[1401];
     if (rc != ocout) {
	ocout=rc;
	fprintf(stderr,"HP: OC outputs=%x\n", rc);
     }
     rc=buffer[1402];
     if (rc != db9) {
	db9=rc;
	fprintf(stderr,"HP: Outputs DB9=%x\n", rc);
     }
     rc=buffer[1403];
     if (rc != mercury_atts) {
	mercury_atts=rc;
	fprintf(stderr,"HP: MercuryAtts=%x\n", rc);
     }
     // Store Alex0 and Alex1 bits in separate ints
     freq=(buffer[1428] << 24) + (buffer[1429] << 16) + (buffer[1430] << 8) + buffer[1431];
     for (i=0; i<32; i++) {
	rc=(freq >> i) & 0x01;
	if (rc != alex1[i]) {
	    alex1[i]=rc;
	    fprintf(stderr,"HP: ALEX1 bit%d set to %d\n", i, rc);
	}
     }
     freq=(buffer[1432] << 24) + (buffer[1433] << 16) + (buffer[1434] << 8) + buffer[1435];
     for (i=0; i<32; i++) {
	rc=(freq >> i) & 0x01;
	if (rc != alex0[i]) {
	    alex0[i]=rc;
	    fprintf(stderr,"HP: ALEX0 bit%d set to %d\n", i, rc);
	}
     }
     rc=buffer[1442];
     if (rc != stepatt1) {
	stepatt1=rc;
	rxatt1_dbl=pow(10.0, -0.05*stepatt1);
	fprintf(stderr,"HP: StepAtt1 = %d\n", rc);
     }
     rc=buffer[1443];
     if (rc != stepatt0) {
	stepatt0=rc;
	rxatt0_dbl=pow(10.0, -0.05*stepatt0);
	fprintf(stderr,"HP: StepAtt0 = %d\n", stepatt0);
     }
  }
  return NULL;
}

void *rx_thread(void *data) {
  int sock;
  struct sockaddr_in addr;
  // One instance of this thread is started for each DDC
  unsigned long seqnum;
  unsigned char buffer[1444];
  int yes = 1;
  int rc;
  int ddc;
  int i;
  unsigned long time;
  long wait;
  double i0sample,q0sample;
  double i1sample,q1sample;
  double irsample,qrsample;
  double fac;
  int sample;
  unsigned char *p;
  int noisept;
  int myddc;
  long myrate;
  int sync,size;
  int myadc, syncadc;
  int ps=0;
  int rxptr;
  
  struct timespec delay;
#ifdef __APPLE__
  struct timespec now;
#endif

  myddc=(int) (uintptr_t) data;
  if (myddc < 0 || myddc >= NUMRECEIVERS) return NULL;
  seqnum=0;

  sock=socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    perror("***** ERROR: RXthread: socket");
    return NULL;
  }

  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));
  setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, (void *)&yes, sizeof(yes));

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(ddc0_port+myddc);

  if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("***** ERROR: RXthread: bind");
    return NULL;
  }

  noisept=0;
  clock_gettime(CLOCK_MONOTONIC, &delay);
  fprintf(stderr,"RX thread %d, enabled=%d\n", myddc, ddcenable[myddc]);
  rxptr=txptr-5000;  
  if (rxptr < 0) rxptr += NEWRTXLEN;
  while (run) {
	if (!ddcenable[myddc] || rxrate[myddc] == 0 || rxfreq[myddc] == 0) {
	  usleep(5000);
          clock_gettime(CLOCK_MONOTONIC, &delay);
	  rxptr=txptr-5000;  
	  if (rxptr < 0) rxptr += NEWRTXLEN;
          continue;
        }
	myadc=adcmap[myddc];
        // for simplicity, we only allow for a single "synchronized" DDC,
        // this well covers the PURESIGNAL and DIVERSITY cases
	sync=0;
	i=syncddc[myddc];
	while (i) {
	  sync++;
	  i = i >> 1;
	}
	// sync == 0 means no synchronizatsion
        // sync == 1,2,3  means synchronization with DDC0,1,2
	// Usually we send 238 samples per buffer, but with synchronization
	// we send 119 sample *pairs*.
        if (sync) {
	  size=119;
          wait=119000000L/rxrate[myddc]; // time for these samples in nano-secs
	  syncadc=adcmap[sync-1];
	} else {
	  size=238;
          wait=238000000L/rxrate[myddc]; // time for these samples in nano-secs
	}
	//
	// ADC0: noise   (+ distorted TX signal upon TXing)
	// ADC1: noise   20 dB stronger
	// ADC2:         original TX signal (ADC1 on HERMES)
	//
	  
        ps=(sync && (rxrate[myadc]==192) && ptt[0] && (syncadc == adc));
        p=buffer;
        *p++ =(seqnum >> 24) & 0xFF;
        *p++ =(seqnum >> 16) & 0xFF;
        *p++ =(seqnum >>  8) & 0xFF;
        *p++ =(seqnum >>  0) & 0xFF;
        seqnum += 1;
        // do not use time stamps
	*p++ = 0;
	*p++ = 0;
	*p++ = 0;
	*p++ = 0;
	*p++ = 0;
	*p++ = 0;
	*p++ = 0;
	*p++ = 0;
	// 24 bits per sample *ALWAYS*
	*p++ = 0;
	*p++ = 24;
	*p++ = 0;
	*p++ = sync ? 2*size : size;  // should be 238 in either case
	for (i=0; i<size; i++) {
	  //
	  // produce noise depending on the ADC
	  //
	  i1sample=i0sample=noiseItab[noisept];
	  q1sample=q0sample=noiseItab[noisept++];
          if (noisept == LENNOISE) noisept=rand() / NOISEDIV;
	  if (myadc == 1) {
	    i0sample=i0sample*10.0;   // 20 dB more noise on ADC1
	    q0sample=q0sample*10.0;
          }
	  //
	  // PS: produce sample PAIRS,
	  // a) distorted TX data (with Drive and Attenuation) 
	  // b) original TX data (normalized)
	  //
	  if (ps) {
	    irsample = isample[rxptr];
	    qrsample = qsample[rxptr++];
	    if (rxptr >= NEWRTXLEN) rxptr=0;
	    fac=txatt_dbl*txdrv_dbl*(IM3a+IM3b*(irsample*irsample+qrsample*qrsample)*txdrv_dbl*txdrv_dbl);
	    if (myadc == 0) {
	      i0sample += irsample*fac;
	      q0sample += qrsample*fac;
	    }
	  }
	  if (sync) {
	    if (ps) {
	      // synchronized stream: undistorted TX signal with constant max. amplitude
	      i1sample = irsample * 0.329;
	      q1sample = qrsample * 0.329;
	    }
	    sample=i0sample * 8388607.0;
	    *p++=(sample >> 16) & 0xFF;
	    *p++=(sample >>  8) & 0xFF;
	    *p++=(sample >>  0) & 0xFF;
	    sample=q0sample * 8388607.0;
	    *p++=(sample >> 16) & 0xFF;
	    *p++=(sample >>  8) & 0xFF;
	    *p++=(sample >>  0) & 0xFF;
	    sample=i1sample * 8388607.0;
	    *p++=(sample >> 16) & 0xFF;
	    *p++=(sample >>  8) & 0xFF;
	    *p++=(sample >>  0) & 0xFF;
	    sample=q1sample * 8388607.0;
	    *p++=(sample >> 16) & 0xFF;
	    *p++=(sample >>  8) & 0xFF;
	    *p++=(sample >>  0) & 0xFF;
	  } else {
	    sample=i0sample * 8388607.0;
	    *p++=(sample >> 16) & 0xFF;
	    *p++=(sample >>  8) & 0xFF;
	    *p++=(sample >>  0) & 0xFF;
	    sample=q0sample * 8388607.0;
	    *p++=(sample >> 16) & 0xFF;
	    *p++=(sample >>  8) & 0xFF;
	    *p++=(sample >>  0) & 0xFF;
	  }
        }
        delay.tv_nsec += wait;
        while (delay.tv_nsec >= 1000000000) {
          delay.tv_nsec -= 1000000000;
          delay.tv_sec++;
	}
#ifdef __APPLE__
        //
        // The (so-called) operating system for Mac does not have clock_nanosleep(),
        // but is has clock_gettime as well as nanosleep.
        // So, to circumvent this problem, we look at the watch and determine
        // how long we should sleep now.
        //
        clock_gettime(CLOCK_MONOTONIC, &now);
        now.tv_sec =delay.tv_sec  - now.tv_sec;
        now.tv_nsec=delay.tv_nsec - now.tv_nsec;
        while (now.tv_nsec < 0) {
         now.tv_nsec += 1000000000;
         now.tv_sec--;
        }
        nanosleep(&now, NULL);
#else
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &delay, NULL);
#endif
        if (sendto(sock, buffer, 1444, 0, (struct sockaddr*)&addr_new, sizeof(addr_new)) < 0) {
          perror("***** ERROR: RX thread sendto");
          break;
	}
  }
  close(sock);
  return NULL;
}

//
// This thread receives data (TX samples) from the PC
//
void *tx_thread(void * data) {
  int sock;
  struct sockaddr_in addr;
  socklen_t lenaddr=sizeof(addr);
  unsigned long seqnum, seqold;
  unsigned char buffer[1444];
  int yes = 1;
  int rc;
  int i;
  unsigned char *p;
  int noisept;
  int sample;
  double di,dq;
  double sum;
  struct timeval tv;

  sock=socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    perror("***** ERROR: TX: socket");
    return NULL;
  }

  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));
  setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, (void *)&yes, sizeof(yes));
  tv.tv_sec = 0;
  tv.tv_usec = 10000;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (void *)&tv, sizeof(tv));

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(duc0_port);

  if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("***** ERROR: TX: bind");
    return NULL;
  }

  seqnum=0; 
  while(run) {
     rc = recvfrom(sock, buffer, 1444, 0,(struct sockaddr *)&addr, &lenaddr);
     if (rc < 0 && errno != EAGAIN) {
       perror("***** ERROR: TX thread: recvmsg");
       break;
     }
     if (rc < 0) continue;
     if (rc != 1444) {
       fprintf(stderr,"Received TX packet with incorrect length");
       break;
     }
     seqold = seqnum;
     seqnum = (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
     if (seqnum != 0 &&seqnum != seqold+1 ) {
       fprintf(stderr,"TXthread: SEQ ERROR, old=%lu new=%lu\n", seqold, seqnum);
     }
     p=buffer+4;
     sum=0.0;
     for (i=0; i<240; i++) {
	// process 240 TX iq samples
        sample  = (int)((signed char) (*p++))<<16;
        sample |= (int)((((unsigned char)(*p++))<<8)&0xFF00);
        sample |= (int)((unsigned char)(*p++)&0xFF);
	di = (double) sample / 8388608.0;
        sample  = (int)((signed char) (*p++))<<16;
        sample |= (int)((((unsigned char)(*p++))<<8)&0xFF00);
        sample |= (int)((unsigned char)(*p++)&0xFF);
	dq = (double) sample / 8388608.0;
//
//      put TX samples into ring buffer
//
	isample[txptr]=di;
	qsample[txptr++]=dq;
	if (txptr >= NEWRTXLEN) txptr=0;
//
//      accumulate TX power
//
        sum += (di*di+dq*dq);
     }
     txlevel=sum * txdrv_dbl * txdrv_dbl * 0.0041667;
  }
  return NULL;
}

void *send_highprio_thread(void *data) {
  int sock;
  struct sockaddr_in addr;
  unsigned long seqnum;
  unsigned char buffer[60];
  int yes = 1;
  int rc;
  int i;
  unsigned char *p;


  seqnum=0;

  sock=socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    perror("***** ERROR: SendHighPrio thread: socket");
    return NULL;
  }

  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));
  setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, (void *)&yes, sizeof(yes));

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(shp_port);

  if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("***** ERROR: SendHighPrio thread: bind");
    return NULL;
  }

  seqnum=0;
  while (1) {
    if (!run) {
	close(sock);
	break;
    }
    // prepare buffer
    memset(buffer, 0, 60);
    p=buffer;
    *p++ = (seqnum >> 24) & 0xFF;
    *p++ = (seqnum >> 16) & 0xFF;
    *p++ = (seqnum >>  8) & 0xFF;
    *p++ = (seqnum >>  0) & 0xFF;
    *p++ = 0;    // no PTT and CW attached
    *p++ = 0;    // no ADC overload
    *p++ = 1;
    *p++ = 126;    // 1 W exciter power
 
    p +=6;

    rc=(int) (800.0*sqrt(10*txlevel));    
    *p++ = (rc >> 8) & 0xFF;
    *p++ = (rc     ) & 0xFF;

    buffer[49]=63;   // about 13 volts supply 

    if (sendto(sock, buffer, 60, 0, (struct sockaddr*)&addr_new, sizeof(addr_new)) < 0) {
       perror("***** ERROR: HP send thread sendto");
       break;
    }
    seqnum++;
    usleep(50000); // wait 50 msec
  }
  close(sock);
  return NULL;
}

//
// This thread receives the audio samples and plays them
//
void *audio_thread(void *data) {
  int sock;
  struct sockaddr_in addr;
  socklen_t lenaddr=sizeof(addr);
  unsigned long seqnum, seqold;
  unsigned char buffer[260];
  int yes = 1;
  int rc;
  int i;
  unsigned char *p;
  int16_t lsample,rsample;
  struct timeval tv;

  sock=socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    perror("***** ERROR: Audio: socket");
    return NULL;
  }

  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));
  setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, (void *)&yes, sizeof(yes));
  tv.tv_sec = 0;
  tv.tv_usec = 10000;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (void *)&tv, sizeof(tv));

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(audio_port);

  if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("***** ERROR: Audio: bind");
    return NULL;
  }

  seqnum=0;
  while(run) {
     rc = recvfrom(sock, buffer, 260, 0,(struct sockaddr *)&addr, &lenaddr);
     if (rc < 0 && errno != EAGAIN) {
       perror("***** ERROR: Audio thread: recvmsg");
       break;
     }
     if (rc < 0) continue;
     if (rc != 260) {
       fprintf(stderr,"Received Audio packet with incorrect length");
       break;
     }
     seqold = seqnum;
     seqnum = (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
     if (seqnum != 0 &&seqnum != seqold+1 ) {
       fprintf(stderr,"Audio thread: SEQ ERROR, old=%lu new=%lu\n", seqold, seqnum);
     }
     p=buffer+4;
     for (i=0; i<64; i++) {
       lsample  = ((signed char) *p++) << 8;	
       lsample |= (*p++ & 0xff); 
       rsample  = ((signed char) *p++) << 8;	
       rsample |= (*p++ & 0xff); 
       audio_write(lsample,rsample);
    }
  }
  close (sock);
  return NULL;
}

//
// The microphone thread generates
// a two-tone signal
//
void *mic_thread(void *data) {
  int sock;
  struct sockaddr_in addr;
  unsigned long seqnum;
  unsigned char buffer[132];
  unsigned char *p;
  int yes = 1;
  int rc;
  int i;
  double arg1,arg2;
  int sintab[480];  // microphone data
  int sinptr = 0;
  struct timespec delay;
#ifdef __APPLE__
  struct timespec now;
#endif

#define FREQ900  0.11780972450961724644234912687298
#define FREQ1700 0.22252947962927702105777057298230

  seqnum=0;
  arg1=0.0;
  arg2=0.0;
  for (i=0; i<480; i++) {
    sintab[i]=(int) round((sin(arg1)+sin(arg2)) * 5000.0);
    arg1 += FREQ900;
    arg2 += FREQ1700;
  }

  sock=socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    perror("***** ERROR: Mic thread: socket");
    return NULL;
  }

  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));
  setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, (void *)&yes, sizeof(yes));

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(mic_port);

  if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("***** ERROR: Mic thread: bind");
    return NULL;
  }

  seqnum=0;
  memset(buffer, 0, 132);
  clock_gettime(CLOCK_MONOTONIC, &delay);
  while (run) {
    // update seq number
    p=buffer;
    *p++ = (seqnum >> 24) & 0xFF;
    *p++ = (seqnum >> 16) & 0xFF;
    *p++ = (seqnum >>  8) & 0xFF;
    *p++ = (seqnum >>  0) & 0xFF;
    seqnum++;
    // take periodic data from sintab as "microphone samples"
    for (i=0; i< 64; i++) {
	rc=sintab[sinptr++];
        if (sinptr == 480) sinptr=0;
        *p++ = (rc >> 8)  & 0xff;
        *p++ = (rc & 0xff);
    }
    // 64 samples with 48000 kHz, makes 1333333 nsec
    delay.tv_nsec += 1333333;
    while (delay.tv_nsec >= 1000000000) {
      delay.tv_nsec -= 1000000000;
      delay.tv_sec++;
    }
#ifdef __APPLE__
    //
    // The (so-called) operating system for Mac does not have clock_nanosleep(),
    // but is has clock_gettime as well as nanosleep.
    // So, to circumvent this problem, we look at the watch and determine
    // how long we should sleep now.
    //
    clock_gettime(CLOCK_MONOTONIC, &now);
    now.tv_sec =delay.tv_sec  - now.tv_sec;
    now.tv_nsec=delay.tv_nsec - now.tv_nsec;
    while (now.tv_nsec < 0) {
     now.tv_nsec += 1000000000;
     now.tv_sec--;
    }
    nanosleep(&now, NULL);
#else
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &delay, NULL);
#endif
    if (sendto(sock, buffer, 132, 0, (struct sockaddr*)&addr_new, sizeof(addr_new)) < 0) {
      perror("***** ERROR: Mic thread sendto");
      break;
    }
  }
  close(sock);
  return NULL;
}
  
