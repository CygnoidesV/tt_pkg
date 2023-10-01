#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <thread>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "tt_pkg/protocol.hpp"
#include "tt_pkg/msg/position_info.hpp"
#include "tt_pkg/msg/move_cmd.hpp"
#include "tt_pkg/msg/arm_cmd.hpp"

rclcpp::Publisher<tt_pkg::msg::PositionInfo>::SharedPtr pub1;

void receive_handler(uint8_t msg_id, uint8_t *data)
{
    position_info_t position_info;
    auto msg = std::make_shared<tt_pkg::msg::PositionInfo>();
    switch (msg_id)
    {
        case MSG_POSITION_INFO:
            position_info = *((position_info_t *)data);
            // printf("Position_info: %f, %f, %f, %d\n", position_info.x_abs, position_info.y_abs, position_info.angle_abs, position_info.stuff_num);
            msg->x_abs = position_info.x_abs;
            msg->y_abs = position_info.y_abs;
            msg->angle_abs = position_info.angle_abs;
            msg->stuff_num = position_info.stuff_num;
            pub1->publish(*msg);
            // printf("[INFO] [communication_node]: Position_info: %f, %f, %f, %d.\n", position_info.x_abs, position_info.y_abs, position_info.angle_abs, position_info.stuff_num);
            break;
    
        default:
            printf("Invalid msg_id.\n");
            break;
    }
}

class Communication: public rclcpp::Node
{
    private:
        // rclcpp::TimerBase::SharedPtr timer1_;
        rclcpp::Subscription<tt_pkg::msg::MoveCmd>::SharedPtr sub1_;
        rclcpp::Subscription<tt_pkg::msg::ArmCmd>::SharedPtr sub2_;

    public:
        Communication(): Node("communication_node")
        {
            sub1_ = this->create_subscription<tt_pkg::msg::MoveCmd>("move_cmd", 10, std::bind(&Communication::sub1_callback, this, std::placeholders::_1));
            sub2_ = this->create_subscription<tt_pkg::msg::ArmCmd>("arm_cmd", 10, std::bind(&Communication::sub2_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "communication_node is started successfully.");
        }

        void sub1_callback(const tt_pkg::msg::MoveCmd::SharedPtr msg) const
        {
            move_cmd_t move_cmd;
            move_cmd.vx = msg->vx;
            move_cmd.vy = msg->vy;
            move_cmd.vw = msg->vw;
            send_data(MSG_MOVE_CMD, (uint8_t *)&move_cmd);
            // printf("Move_cmd: %f %f %f\n", move_cmd.vx, move_cmd.vy, move_cmd.vw);
        }

        void sub2_callback(const tt_pkg::msg::ArmCmd::SharedPtr msg) const
        {
            arm_cmd_t arm_cmd;
            arm_cmd.act_id = msg->act_id;
            send_data(MSG_ARM_CMD, (uint8_t *)&arm_cmd);
            // printf("Arm_cmd: %x\n", arm_cmd.act_id);
        }
};

int main(int argc, char * argv[])
{
    const char *password = "123"; // 设置密码
    // 要执行的命令
    const char *cmd = "sudo -S chmod 777 /dev/ttyUSB0"; // 替换 your_sensitive_command 为实际的敏感命令
    // 打开管道以与sudo交互
    FILE *pipe = popen(cmd, "w");
    if (!pipe) {
        perror("Unable to open pipeline!\n");
        return 1;
    }
    // 将密码写入sudo
    fprintf(pipe, "%s\n", password);
    fflush(pipe);
    // 关闭管道
    pclose(pipe);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<Communication>();
    pub1 = node->create_publisher<tt_pkg::msg::PositionInfo>("position_info", 10);

    if(protocol_init() == -1)
    {
        return 1;
    }
    std::thread receive_thread(receive_data);

    rclcpp::spin(node);
    rclcpp::shutdown();

  return 0;
}