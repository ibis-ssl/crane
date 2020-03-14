// Copyright (c) 2019 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <rclcpp/rclcpp.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <class_loader/visibility_control.hpp>

#include "math.h"
#include <memory>
#include <stdlib.h>
#include  <iostream>
#include  <vector>
#include  <string>
#include <stdio.h>
#include <string.h> 
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h> 
#include <netinet/udp.h>
int check;
int sock;
struct sockaddr_in addr;
const char *opt = "enp3s0f1";


class RealSenderNode : public rclcpp::Node {
public:
  CLASS_LOADER_PUBLIC
  explicit RealSenderNode(const rclcpp::NodeOptions &options) : Node("real_sender_node", options){
    sub_commnads_ = create_subscription<crane_msgs::msg::RobotCommands>("/bt_executor/robot_commands",
        std::bind(&RealSenderNode::robotCommandsCallback, this, std::placeholders::_1));
        std::cout << "start" << std::endl;
  }
  void robotCommandsCallback(crane_msgs::msg::RobotCommands::ConstSharedPtr msg){
    // TODO(okada_tech) : send commands to robots

    uint8_t vel_surge_send_high,vel_surge_send_low,vel_sway_send_high,vel_sway_send_low,vel_angular_vision_send_high,vel_angular_vision_send_low;
    uint8_t vel_angular_consai_send_high,vel_angular_consai_send_low,kick_power_send,dribble_power_send,keeper_EN;
    uint8_t send_packet[12];
    //const char *opt = "eth1";
    for (auto command : msg->robot_commands) {
        #define MAX_VEL_SURGE  2.0 // m/s
        #define MAX_VEL_SWAY   2.0 // m/s
        #define MAX_VEL_ANGULAR  2.0*M_PI


        //vel_surge
        // -2 ~ 2 -> 0 ~ 32767 ~ 65534
        // -2 -> 0
        // 0 -> 32767
        // 2 -> 65534
        uint16_t vel_surge_send=0;
        vel_surge_send= int(32767*(float)(command.target.x/MAX_VEL_SURGE)+ 32767);
        vel_surge_send_low = vel_surge_send  & 0x00FF;    
        vel_surge_send_high = (vel_surge_send & 0xFF00)>>8;
            
        //vel_sway
        //-2 ~ 2 -> 0 ~ 32767 ~ 65534
        //-2 -> 0
        //0 -> 32767
        //2 -> 65534
        uint16_t vel_sway_send = 0;
        vel_sway_send= int(32767*(float)(command.target.y/MAX_VEL_SWAY)+ 32767);
        vel_sway_send_low = vel_sway_send  & 0x00FF;    
        vel_sway_send_high= (vel_sway_send & 0xFF00)>>8;


        //目標角度
        //-pi ~ pi -> 0 ~ 32767 ~ 65534
        //-pi -> 0
        //0 -> 32767
        //pi -> 65534
        float vel_angular_consai=0;

        vel_angular_consai = command.target.theta;
        if (fabs(vel_angular_consai) > M_PI){
           vel_angular_consai = copysign(M_PI, vel_angular_consai);
        }
               
        //-pi ~ pi -> 0 ~ 32767 ~ 65534
        uint16_t vel_angular_consai_send=0;

        vel_angular_consai_send=int(32767*(float)(vel_angular_consai/M_PI) + 32767);
        vel_angular_consai_send_low=  vel_angular_consai_send & 0x00FF;
        vel_angular_consai_send_high= (vel_angular_consai_send & 0xFF00)>>8;

        //Vision角度
        //-pi ~ pi -> 0 ~ 32767 ~ 65534
        //pi -> 0
        //0 -> 32767
        //pi -> 65534
        float vel_angular_vision=0;

        vel_angular_vision = command.current_theta;
        
        if(fabs(vel_angular_vision) > M_PI){
          vel_angular_vision = copysign(M_PI, vel_angular_vision);
        }
        //-pi ~ pi -> 0 ~ 32767 ~ 65534
        uint16_t vel_angular_vision_send=0;

        vel_angular_vision_send=int(32767*(float)(vel_angular_vision/M_PI) + 32767);
        vel_angular_vision_send_low=  vel_angular_vision_send & 0x00FF;
        vel_angular_vision_send_high= (vel_angular_vision_send & 0xFF00)>>8;

        //ドリブル
        //0 ~ 1.0 -> 0 ~ 20
        float dribble_power=0;

        if (command.dribble_power > 0){
                dribble_power = command.dribble_power;
                if(dribble_power > 1.0){
                    dribble_power = 1.0;
                }
                else if(dribble_power < 0){
                    dribble_power = 0.0;
                    }
                dribble_power_send= int(round(20 * dribble_power));
        }
        
        
        //キック
        //0 ~ 1.0 -> 0 ~ 20
        //チップキック有効の場合　0 ~ 1.0 -> 100 ~ 120
        float kick_power=0;
            if (command.kick_power > 0){
                kick_power = command.kick_power;
                if(kick_power > 1.0){
                    kick_power = 1.0;
                }
                else if( kick_power < 0){
                    kick_power = 0;
                }
                if(command.chip_kick_enable){
                    kick_power_send = int((round(20 * kick_power)+100));
                }
                else{
                    kick_power_send = int(round(20 * kick_power));
                }
            }


        //キーパーEN
        // 0 or 1
        keeper_EN = command.local_goalie_enable;


    switch(command.robot_id){
        case 0:
                sock = socket(AF_INET, SOCK_DGRAM, 0);
                setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
                addr.sin_family = AF_INET;
                addr.sin_port = htons(12345);
                addr.sin_addr.s_addr = inet_addr("192.168.20.100");
        break;

        case 1:
                sock = socket(AF_INET, SOCK_DGRAM, 0);
                setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
                addr.sin_family = AF_INET;
                addr.sin_port = htons(12345);
                addr.sin_addr.s_addr = inet_addr("192.168.20.101");
        break;


        case 2:
                sock = socket(AF_INET, SOCK_DGRAM, 0);
                setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
                addr.sin_family = AF_INET;
                addr.sin_port = htons(12345);
                addr.sin_addr.s_addr = inet_addr("192.168.20.102");
        break;

        case 3:
                sock = socket(AF_INET, SOCK_DGRAM, 0);
                addr.sin_family = AF_INET;
                addr.sin_port = htons(12345);
                addr.sin_addr.s_addr = inet_addr("192.168.20.103");
        break;

        case 4:
                sock = socket(AF_INET, SOCK_DGRAM, 0);
                setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
                addr.sin_family = AF_INET;
                addr.sin_port = htons(12345);
                addr.sin_addr.s_addr = inet_addr("192.168.20.104");
       break;

        case 5:
                sock = socket(AF_INET, SOCK_DGRAM, 0);
                setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
                addr.sin_family = AF_INET;
                addr.sin_port = htons(12345);
                addr.sin_addr.s_addr = inet_addr("192.168.20.105");
        break;

        case 6:
                sock = socket(AF_INET, SOCK_DGRAM, 0);
                setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
                addr.sin_family = AF_INET;
                addr.sin_port = htons(12345);
                addr.sin_addr.s_addr = inet_addr("192.168.20.106");
        break;

        case 7:
                sock = socket(AF_INET, SOCK_DGRAM, 0);
                setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
                addr.sin_family = AF_INET;
                addr.sin_port = htons(12345);
                addr.sin_addr.s_addr = inet_addr("192.168.20.107");
        break;

        case 8:
                sock = socket(AF_INET, SOCK_DGRAM, 0);
                setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
                addr.sin_family = AF_INET;
                addr.sin_port = htons(12345);
                addr.sin_addr.s_addr = inet_addr("192.168.20.108");
        break;

        case 9:
                sock = socket(AF_INET, SOCK_DGRAM, 0);
                setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
                addr.sin_family = AF_INET;
                addr.sin_port = htons(12345);
                addr.sin_addr.s_addr = inet_addr("192.168.20.109");
        break;

        case 10:
                sock = socket(AF_INET, SOCK_DGRAM, 0);
                setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
                addr.sin_family = AF_INET;
                addr.sin_port = htons(12345);
                addr.sin_addr.s_addr = inet_addr("192.168.20.110");
        break;

        case 11:
                sock = socket(AF_INET, SOCK_DGRAM, 0);
                setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
                addr.sin_family = AF_INET;
                addr.sin_port = htons(12345);
                addr.sin_addr.s_addr = inet_addr("192.168.20.111");
        break;

        case 12:
                sock = socket(AF_INET, SOCK_DGRAM, 0);
                setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
                addr.sin_family = AF_INET;
                addr.sin_port = htons(12345);
                addr.sin_addr.s_addr = inet_addr("192.168.20.112");
        break;
        


    }



    send_packet[0]=(uint8_t)vel_surge_send_high;
    send_packet[1]=(uint8_t)vel_surge_send_low;
    send_packet[2]=(uint8_t)vel_sway_send_high;
    send_packet[3]=(uint8_t)vel_sway_send_low;
    send_packet[4]=(uint8_t)vel_angular_vision_send_high;
    send_packet[5]=(uint8_t)vel_angular_vision_send_low;
    send_packet[6]=(uint8_t)vel_angular_consai_send_high;
    send_packet[7]=(uint8_t)vel_angular_consai_send_low;
    send_packet[8]=(uint8_t)kick_power_send;
    send_packet[9]=(uint8_t)dribble_power_send;
    send_packet[10]=(uint8_t)keeper_EN;
    send_packet[11]=(uint8_t)check;
    
    if(command.robot_id==0){
      printf("ID=%d Vx=%.3f Vy=%.3f theta=%.3f",command.robot_id,command.target.x,command.target.y,vel_angular_consai);
      printf(" vision=%.3f kick=%.2f chip=%d Dri=%.2f",vel_angular_vision,kick_power,(int)command.chip_kick_enable,dribble_power);
      printf(" keeper=%d check=%d",(int)keeper_EN,(int)check);
      printf("\n");
      
      printf("=%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x",
                (int)send_packet[0],send_packet[1],send_packet[2],send_packet[3],send_packet[4],send_packet[5],send_packet[6]
                ,send_packet[7],send_packet[8],send_packet[9],send_packet[10],send_packet[11]);
      printf("\n");
      printf("\n");
      

      check++;
      if(check>200){
        check=0;
      }
    }
    
    sendto(sock,(uint8_t*)&send_packet, 12, 0, (struct sockaddr *)&addr, sizeof(addr));
    close(sock);


    }

  }

private:
  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr sub_commnads_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::NodeOptions options;
  std::shared_ptr<RealSenderNode> real_sender_node =
      std::make_shared<RealSenderNode>(options);

  exe.add_node(real_sender_node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
