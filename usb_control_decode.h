
bool load_HID_report(unsigned char report[],int length);
void clear_HID_report(void);
void load_XID_report(void);
void process_usb_packet(unsigned char buffer[],int length);
void *get_HID_report_pointer(void);
void HID_report_done(void);
void init_usb_control_decode(void);
void copy_input_bits(unsigned char output[],int bitindex, int bitcount);