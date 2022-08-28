#include <iostream>
#include <functional>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/comando.hpp>

#include "../../cpp-spline/spline/src/main/cpp/CatmullRom.h"
#include "../../cpp-spline/spline/src/main/cpp/CatmullRom.cpp"


using std::placeholders::_1;

using namespace std;
using namespace std::chrono_literals;
using namespace rclcpp;
using namespace px4_msgs::msg;

struct punto{
    float x;
    float y;
    float z;
    float yaw;
}last_postion;


void set_last_pos(float x, float y, float z, float yaw){
    last_postion.x=x;
    last_postion.y=y;
    last_postion.z=z;
    last_postion.yaw=yaw;
}


float calcola_yaw(float x0,float y0){
    float dx=x0-last_postion.x;
    float dy=y0-last_postion.y;
    float yaw;
    if(dx!=0) yaw=dy/dx;
    else if (dx>0) yaw=1.5708;
    else yaw=-1.5708;  //DA CONTROLLARE
    cout<<"YAW: "<<yaw<<endl;
    return yaw;
}



class OffboardControl : public Node
{

    private:
    //declaration of publisherss, subscribers and variables
        std::atomic<uint64_t> timestamp_; //synchronized timestamp
        rclcpp::TimerBase::SharedPtr timer_;

        Publisher<VehicleCommand>::SharedPtr vehicleCommand_;
        Publisher<OffboardControlMode>::SharedPtr offboardControlMode_;
        Publisher<TrajectorySetpoint>::SharedPtr trajectorySetpoint_;
        Subscription<Timesync>::SharedPtr timeSync_;
        Subscription<Comando>::SharedPtr subComm_;


        int count=0;
        float x=0,y=0,z=0, yaw=0;
        int command=VehicleCommand::VEHICLE_CMD_DO_SET_MODE;


    public:

        //declaration of methods
        void publish_control_mode() const;
        void publish_trajectory_setpoint() const;
        void publish_vehicle_command(uint16_t command, float param1=0.0,float param2=1.0,float param3=0.0,
                                     float param4=0.0,float param5=0.0,float param6=0.0,float param7=0.0) const;
        void send_message();
        void arm() const;
        void disarm() const;

        //constructor
        OffboardControl():Node("offboardControls")
        {
            //creation of advestisers and the subscription
            vehicleCommand_=this->create_publisher<VehicleCommand>("/fmu/vehicle_command/in",10);
            offboardControlMode_=this->create_publisher<OffboardControlMode>("/fmu/offboard_control_mode/in",10);
            trajectorySetpoint_=this->create_publisher<TrajectorySetpoint>("/fmu/trajectory_setpoint/in",10);

            //get timestamp to synchronize the messages, store it in timer_
            timeSync_=this->create_subscription<Timesync>("fmu/timesync/out", 10,[this](const Timesync::UniquePtr msg) {
			    timestamp_.store(msg->timestamp);
			});

            this->arm();

            subComm_=this->create_subscription<Comando>("/command",100,[this](const Comando::UniquePtr msg){
                cout<<"Posizione ricevuta, x:"<<msg->x<<" y:"<<msg->y<<" z:"<<msg->z<<" yaw:"<<msg->yaw<<endl;
                switch(msg->com){
                    case 1:{
                        this->z=msg->z; //TO CHANEGE -
                        this->arm(); //vehicle arm
                        for(int i=0;i<100000;i++);
                        // this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0,0,0,this->yaw,this->x,this->y,this->z);  //takeoff command
                        this->publish_vehicle_command(command, 1, 6,0,0,0,0,0);  //takeoff command
                        set_last_pos(0,0,msg->z,0);
                        break;
                    }
                    case 2:{
                        this->yaw=calcola_yaw(this->x, this->y);
                        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND,0,0,0,this->yaw,this->x,this->y,0);
                        this->disarm();
                        break;
                    }
                    case 3:{
                        this->x=msg->x;  this->y=msg->y;  this->z=msg->z; this->yaw=calcola_yaw(this->x, this->y);
                        break;
                    }
                    case 4:{
                        this->x=msg->x;  this->y=msg->y;  this->z=msg->z; this->yaw=calcola_yaw(this->x, this->y);
                        break;
                    }
                    case 5:{
                        this->yaw=msg->yaw;
                        break;
                    }
                }
            });

            //every 100ms (10hz) execute send_message
            timer_ = this->create_wall_timer(100ms,std::bind(&OffboardControl::send_message,this));
        }


};

int main(int argc, char* argv[]){


    cout << "Starting offboard control node..." << endl;
	rclcpp::init(argc, argv); //initializing ros2
    cout<<"offboard control node started"<<endl;
	rclcpp::spin(std::make_shared<OffboardControl>()); //creating instance of class and spinning
    rclcpp::shutdown();
    return 0;

}


//we need to constantly publish the control mode and the trajectory setpoint, to keep offboard mode active,
// 10hz of frequency obtained using 100ms timewall

//used to set control mode to offboard
void OffboardControl::publish_control_mode() const
{
    OffboardControlMode msg{};
    msg.timestamp = timestamp_.load();
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    offboardControlMode_->publish(msg);
}

//used to define point to reach
void OffboardControl::publish_trajectory_setpoint() const
{
	TrajectorySetpoint msg{};
    //hover at 5 meters from the ground at place (0,0) with a yaw of -3.14 (dritto)
	msg.timestamp = timestamp_.load();
    msg.position={this->x, this->y, this->z};
	msg.yaw = 0; // [-PI:PI] //TODO set this->yaw, al momento si sminchia tutto

	trajectorySetpoint_->publish(msg);
}



//method to publish a command to the vehicle, published only once but in parallel with trajectory_setpoint and control_mode
void OffboardControl::publish_vehicle_command(uint16_t command,float param1,float param2,float param3,float param4,float param5,float param6,float param7) const {
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.command = command;
	msg.param1 = param1;
	msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicleCommand_->publish(msg);
}


//method called every 100 ms, sends both control_mode and tajectory_setpoint
void OffboardControl::send_message(){

    this->publish_control_mode();
    this->publish_trajectory_setpoint();

}


//arming method
void OffboardControl::arm() const 
{
    OffboardControl::publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    cout<<"Vehicle arm sent\n";
}


//disarming method
void OffboardControl::disarm() const 
{
    OffboardControl::publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    cout<<"vehicle disarm sent \n";
}