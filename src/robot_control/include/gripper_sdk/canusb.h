#ifndef CANUSB_H
#define CANUSB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <unistd.h>

#define CANUSB_INJECT_SLEEP_GAP_DEFAULT 200 /* ms */
#define CANUSB_TTY_BAUD_RATE_DEFAULT 2000000

typedef enum {
  CANUSB_SPEED_1000000 = 0x01,
  CANUSB_SPEED_800000 = 0x02,
  CANUSB_SPEED_500000 = 0x03,
  CANUSB_SPEED_400000 = 0x04,
  CANUSB_SPEED_250000 = 0x05,
  CANUSB_SPEED_200000 = 0x06,
  CANUSB_SPEED_125000 = 0x07,
  CANUSB_SPEED_100000 = 0x08,
  CANUSB_SPEED_50000 = 0x09,
  CANUSB_SPEED_20000 = 0x0a,
  CANUSB_SPEED_10000 = 0x0b,
  CANUSB_SPEED_5000 = 0x0c,
} CANUSB_SPEED;

typedef enum {
  CANUSB_MODE_NORMAL = 0x00,
  CANUSB_MODE_LOOPBACK = 0x01,
  CANUSB_MODE_SILENT = 0x02,
  CANUSB_MODE_LOOPBACK_SILENT = 0x03,
} CANUSB_MODE;

typedef enum {
  CANUSB_FRAME_STANDARD = 0x01,
  CANUSB_FRAME_EXTENDED = 0x02,
} CANUSB_FRAME;

typedef enum {
  CANUSB_INJECT_PAYLOAD_MODE_RANDOM = 0,
  CANUSB_INJECT_PAYLOAD_MODE_INCREMENTAL = 1,
  CANUSB_INJECT_PAYLOAD_MODE_FIXED = 2,
} CANUSB_PAYLOAD_MODE;

extern int terminate_after;
extern int program_running;
extern int inject_payload_mode;
extern float inject_sleep_gap;
extern int print_traffic;

CANUSB_SPEED canusb_int_to_speed(int speed);
int generate_checksum(const unsigned char *data, int data_len);
int frame_is_complete(const unsigned char *frame, int frame_len);
int frame_send(int tty_fd, const unsigned char *frame, int frame_len);
int frame_recv(int tty_fd, unsigned char *frame, int frame_len_max);
int command_settings(int tty_fd, CANUSB_SPEED speed, CANUSB_MODE mode,
                     CANUSB_FRAME frame);
int send_data_frame(int tty_fd, CANUSB_FRAME frame, unsigned char id_lsb,
                    unsigned char id_msb, unsigned char data[],
                    int data_length_code);
int hex_value(int c);
int convert_from_hex(const char *hex_string, unsigned char *bin_string,
                     int bin_string_len);
int inject_data_frame(int tty_fd, const char *hex_id, const char *hex_data);
void dump_data_frames(int tty_fd);
int adapter_init(const char *tty_device, int baudrate);
void display_help(const char *progname);
void sigterm(int signo);

#ifdef __cplusplus
}
#endif

#endif /* CANUSB_H */
