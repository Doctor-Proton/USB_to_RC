void output_task(void *arg);
void init_output_mixer(void);
void output_set_event(void);
int get_input_count(void);
void get_input_debug_line(int channel,char line[],int maxlen);
void new_descriptor_set_event(void);
int print_var_line(char output[],int maxlen, int startindex, int varcount);
void signal_new_device(unsigned short VID, unsigned short PID);
void signal_device_gone(void);

#define TIMING_DEBUG_GPIO 47
#define TIMING_DEBUG_GPIO_2 48
