#define __PACKED  __attribute__((packed))
#define __WEAK __attribute__((weak))

#include <stdio.h>
#include <stdint.h>

#define SOF 0xA5

#define MSG_POSITION_INFO 0x01
#define MSG_MOVE_CMD 0x02
#define MSG_ARM_CMD 0x03

#define ARM1_GRAP1 0x01
#define ARM2_GRAP1 0x02
#define ARM2_PLACE1 0x03
#define ARM2_PLACE2 0x04

const char TOF = 0X5A;

typedef struct __PACKED
{
    uint8_t sof;
    uint8_t msg_id;
    uint16_t length;
    // uint8_t crc8;
} frame_header_t;

typedef struct __PACKED
{
    float x_abs;
    float y_abs;
    float angle_abs;
    uint16_t stuff_num;
} position_info_t;

typedef struct __PACKED
{
    float vx;
    float vy;
    float vw;
} move_cmd_t;

typedef struct __PACKED
{
    uint8_t act_id;
} arm_cmd_t;

int protocol_init();
int receive_data();
int send_data(uint8_t msg_id, uint8_t *data);
void receive_handler(uint8_t msg_id, uint8_t *data);