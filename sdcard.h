#define SD_CARD_PATH "sd0"
#define MOUNT_POINT "/sd0"
int sd_card_init(void);
void sd_unmount(void);
bool is_sd_mounted(void);
