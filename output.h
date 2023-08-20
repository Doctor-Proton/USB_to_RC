void output_task(void *arg);
void init_output_mixer(void);
void output_set_event(void);
int get_input_count(void);
void get_input_debug_line(int channel,char line[],int maxlen);
void new_descriptor_set_event(void);
int print_var_line(char output[],int maxlen, int startindex, int varcount);
void signal_new_device(unsigned short VID, unsigned short PID);
void signal_device_gone(void);
int get_vars_size(void);
int read_vars_bytes(unsigned char *buf, int offset, int len);
int usb_connected(void);
int get_vars_count(void);
void get_var(char *buffer, int len, float *value,int var_index);
char *get_mixer_filename(void);
void reload_mixers(void);

#define TIMING_DEBUG_GPIO 47
#define TIMING_DEBUG_GPIO_2 48
