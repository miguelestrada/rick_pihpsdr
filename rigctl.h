#ifndef RIGCTL_H
#define RIGCTL_H

void launch_rigctl (void);
int launch_serial (void);
void disable_serial (void);
int launch_andromeda_fp_serial (void);
void disable_andromeda_fp_serial (void);

void  close_rigctl_ports (void);
int   rigctlGetMode(void);
int   lookup_band(int);
char * rigctlGetFilter(void);
void set_freqB(long long);
extern int cat_control;
int set_alc(gpointer);
extern int rigctl_busy;

extern int rigctl_port_base;
extern int rigctl_enable;


#endif // RIGCTL_H
