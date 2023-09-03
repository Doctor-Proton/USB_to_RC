#ifdef __cplusplus
extern "C" {
#endif
void usb_host_task(void *p);
int print_device_descriptor_ssi(char *buffer, int Len);
void print_device_descriptor_vt100();
void get_product_string(char *buffer, int len);
void get_manufacturer_string(char *buffer, int len);
void get_vid_pid(uint16_t *vid, uint16_t *pid);
#ifdef __cplusplus
}
#endif