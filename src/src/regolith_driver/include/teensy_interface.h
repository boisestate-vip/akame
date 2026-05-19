
#ifndef __TEENSY_INTERFACE

#define __TEENSY_INTERFACE

#include <stdint.h>

/* binary protocol shared between the regolith controller teensy
 * and the host. keep in sync with the controller's types.h.   */

/* Layout (28 bytes): pin @ 0, name[16] @ 1, [3 bytes padding @ 17-19], value @ 20, voltage @ 24. */
struct analog_data_collapse {
   uint8_t pin;
   char name[16];
   int value;     /* 3 bytes implicit padding before this (offset 17 → 20) */
   float voltage;
};

struct power_data_collapse {
   char rail[24];
   int initialized;
   float temperature;
   float voltage;
   float current;         /* amps */
   float power;           /* watts */
   float estimated_voltage;
   float estimated_current;
   float estimated_power;
};

#define DATA_404           -2
#define DATA_NACK          -1
#define DATA_ACK            0
#define DATA_ANALOG         1
#define DATA_TOF            2
#define DATA_POWER          3
#define DATA_REG_MASS       4
#define DATA_ESTIMATE_LOAD  5

#define MAX_ANALOG_CHANNELS  2
#define MAX_POWER_MONITORS   8

struct data {
   int type;
   int size;
   union {
      struct {
         int count;
         struct analog_data_collapse values[MAX_ANALOG_CHANNELS];
      } analog;
      struct {
         int is_new;
         uint16_t range;    /* 2 bytes implicit padding after this */
         float peak_signal;
         float ambient;
         float distance_mm;
      } tof;
      struct {
         int count;
         struct power_data_collapse values[MAX_POWER_MONITORS];
      } power;
      struct {
         float mass;
         float temperature;
      } reg_mass;
      struct {
         float estimated_load;
      } estimate_load;
   } as;
};

#define CMD_MAGIC_0       0xff
#define CMD_MAGIC_1       0xfe

#define CMD_SYN           0
#define CMD_ANALOG        1
#define CMD_TOF           2
#define CMD_POWER         3
#define CMD_RESET         4
#define CMD_REG_MASS      5
#define CMD_ESTIMATE_LOAD 6

#define MAX_CMD_ARGS 2

/* Fixed 12-byte binary frame.
 * Layout: magic_0 @ 0, magic_1 @ 1, type @ 2, argc @ 3, argv[0] @ 4, argv[1] @ 8. */
struct cmd {
   uint8_t magic_0;
   uint8_t magic_1;
   char type;
   uint8_t argc;
   float argv[MAX_CMD_ARGS];
};

#endif /* __TEENSY_INTERFACE */
