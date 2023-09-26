#include <thread>
#include <string.h>
#include "tt_pkg/protocol.hpp"
#include "tt_pkg/uart.hpp"

int uart_fd = -1;
uint8_t recv_buf[256];
uint32_t recv_index = 0;

int protocol_init()
{
    uart_fd = uart_open(DEVICE);
    if (uart_fd == -1)
    {
        printf("Cannot open device!\n");
        return -1;
    }
}

int receive_data()
{
    frame_header_t frame_header;
    memset(recv_buf, 0, sizeof(recv_buf));
    memset(&frame_header, 0, sizeof(frame_header));

    uint8_t data = '\0';
    printf("Thread creation success!\n");
    while(1)
    {
        int ok = uart_read(uart_fd, (char *)&data, 1);
        if (ok == -1)
        {
            printf("Recveive failure!\n");
            uart_fd = -1;
            if(protocol_init() == -1)
            {
                break;
            }
            else
            {
                continue;
            }
        }

        // printf("%x ", data);
        if (recv_index == 0)
        {
            if (data == SOF)
            {
                recv_buf[recv_index++] = data;
            }
        }
        else if (recv_index == sizeof(frame_header_t))
        {
            memcpy(&frame_header, recv_buf, sizeof(frame_header_t));
            if (frame_header.msg_id != MSG_POSITION_INFO && frame_header.msg_id != MSG_MOVE_CMD && frame_header.msg_id !=MSG_ARM_CMD)
            {
                recv_index = 0;
                memset(recv_buf, 0, sizeof(recv_buf));
                memset(&frame_header, 0, sizeof(frame_header));

                if (data == SOF)
                {
                    recv_buf[recv_index++] = data;
                }
            }
            else
            {
                recv_buf[recv_index++] = data;
            }
        }
        else if (recv_index == frame_header.length + sizeof(frame_header_t))
        {
            // printf("I get it!\n");
            if (data == TOF)
            {
                receive_handler(frame_header.msg_id, recv_buf + (int)sizeof(frame_header_t));
            }

            recv_index = 0;
            memset(recv_buf, 0, sizeof(recv_buf));
            memset(&frame_header, 0, sizeof(frame_header));

            if (data == SOF)
            {
                recv_buf[recv_index++] = data;
            }
        }
        else
        {
            if (recv_index == 20)
            {
                recv_index = 0;
                memset(recv_buf, 0, sizeof(recv_buf));
                memset(&frame_header, 0, sizeof(frame_header));
                
                if (data == SOF)
                {
                    recv_buf[recv_index++] = data;
                }
            }
            else
            {
                recv_buf[recv_index++] = data;
            }
        }

    }
}

int send_data(uint8_t msg_id, uint8_t *data)
{
    if (uart_fd == -1)
    {
        if(protocol_init() == -1)
        {
            return -1;
        }
    }

    frame_header_t frame_header;
    frame_header.sof = SOF;
    switch (msg_id)
    {
        case MSG_POSITION_INFO:
            frame_header.length = (uint16_t)sizeof(position_info_t);
            frame_header.msg_id = msg_id;
            uart_write(uart_fd, (const char *)&frame_header, sizeof(frame_header_t));
            uart_write(uart_fd, (const char *)data, frame_header.length);
            break;

        case MSG_MOVE_CMD:
            frame_header.length = (uint16_t)sizeof(move_cmd_t);
            frame_header.msg_id = msg_id;
            uart_write(uart_fd, (const char *)&frame_header, sizeof(frame_header_t));
            uart_write(uart_fd, (const char *)data, frame_header.length);
            break;

        case MSG_ARM_CMD:
            frame_header.length = (uint16_t)sizeof(move_cmd_t);
            frame_header.msg_id = msg_id;
            uart_write(uart_fd, (const char *)&frame_header, sizeof(frame_header_t));
            uart_write(uart_fd, (const char *)data, frame_header.length);
            break;
    
        default:
            printf("Invalid msg_id.\n");
            break;
    }
    uart_write(uart_fd, &TOF, 1);
}

__WEAK void receive_handler(uint8_t msg_id, uint8_t *data)
{   
    position_info_t position_info;
    move_cmd_t move_cmd;
    arm_cmd_t arm_cmd;
    switch (msg_id)
    {
        case MSG_POSITION_INFO:
            position_info = *((position_info_t *)data);
            printf("Position_info: %f, %f, %f, %d.\n", position_info.x_abs, position_info.y_abs, position_info.angle_abs, position_info.stuff_num);
            break;

        case MSG_MOVE_CMD:
            move_cmd = *((move_cmd_t *)data);
            printf("Move_cmd: %f, %f, %f.\n", move_cmd.vx, move_cmd.vy, move_cmd.vw);
            break;

        case MSG_ARM_CMD:
            arm_cmd = *((arm_cmd_t *)data);
            printf("Arm_amd: %x.\n", arm_cmd.act_id);
            break;
    
        default:
            printf("Invalid msg_id.\n");
            break;
    }
}