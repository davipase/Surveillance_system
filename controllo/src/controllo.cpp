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

/** 
 * \brief A struct containing all the information of a point.
 */
struct punto{
    float x; /**< The x value */  
    float y; /**< The y value */  
    float z; /**< The z value */  
    float yaw; /**< The yaw value */  
}last_position;

/** 
 * \relates OffboardControl
 * A function used to set the last position of the drone.
 * \param x The last x position of the drone
 * \param y The last x position of the drone
 * \param z The last x position of the drone
 * \param yaw The last yaw value
 */
void set_last_pos(float x, float y, float z, float yaw);

/** 
 * NOT WORKING
 * A function that calculates the correct yaw of the drone. NOT W
 */
float calcola_yaw(float x0,float y0){
    float dx=x0-last_position.x;
    float dy=y0-last_position.y;
    float yaw;
    if(dx!=0) yaw=dy/dx;
    else if (dx>0) yaw=1.5708;
    else yaw=-1.5708;  //DA CONTROLLARE
    cout<<"YAW: "<<yaw<<endl;
    return yaw;
}


/**
 * \brief Class that keeps the drone in offboard mode and make it move.
 * 
 * The offboard class is responsible for keeping the drone in offboard mode by publishing to the PX4 Autopilot
 * both the OffboardControlMode message and the TrajectorySetpoint message.
 * It receives the commands from the node Comando, interpretes it and sends it to the PX4 via the corresponding topic.
 */
class OffboardControl : public Node
{

    private:

        /** 
         * Atomic unsigned int variable where to store the timestamp.
         */
        std::atomic<uint64_t> timestamp_;

        /** 
         * Timer used for the wall_timer. It will execute the send_message() function every 100 milliseconds.
         */
        rclcpp::TimerBase::SharedPtr timer_;

        /** 
         * VehicleCommand Publisher. It will publish the land and takeoff command to the drone.
         */ 
        Publisher<VehicleCommand>::SharedPtr vehicleCommand_;

        /** 
         * OffboardControlMode Publisher. This message will tell the drone to sstay in offboard mode. Required minimum 10hz.
         */ 
        Publisher<OffboardControlMode>::SharedPtr offboardControlMode_;

        /** 
         * TrajectorySetpoint Publisher. This message will tell the drone the cordinates to reach. Required minimum 10hz.
         */ 
        Publisher<TrajectorySetpoint>::SharedPtr trajectorySetpoint_;

        /** 
         * Timesync subscriber. It will receive from the PX4 the timestamp to be stored in the timetamp_ variable and to be set in every message sent.
         */ 
        Subscription<Timesync>::SharedPtr timeSync_;

        /** 
         * Command subscriber. It will receive the commands to be sent from the node Comando.
         */ 
        Subscription<Comando>::SharedPtr subComm_;

        /** 
         * Variables used to store the position (x,y,z) and the rotation (yaw) value.
         */ 
        float x=0,y=0,z=0, yaw=0;

        /** 
         * Takeoff command.
         */ 
        int command=VehicleCommand::VEHICLE_CMD_DO_SET_MODE;


    public:

        /**
         * The constructor initialize all the Publishers and subscribers with the correct topic, and creates the callback function for the
         * Timesync and the Command subscriber. While the former simply stores the timestamp values in the timestamp_ variable, the latter
         * decypher the meaning of the command based on the command ID, and sends to the drone the correct message.
         */
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
                        this->z=msg->z; //TO CHANGE
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

        /** 
         * The method that sets and publishes the OffboardControlMode messages.
         */ 
        void publish_control_mode() const;
        
        /** 
         * The method that sets and publishes the Trajectorysetpoint messages.
         */ 
        void publish_trajectory_setpoint() const;

        /** 
         * The method that sets and publishes the commands.
         * \param command the command integer identifier (for takeoff and landing)
         */ 
        void publish_vehicle_command(uint16_t command, float param1=0.0,float param2=1.0,float param3=0.0,
                                     float param4=0.0,float param5=0.0,float param6=0.0,float param7=0.0) const;

        /** 
         * This methodcalls both the publish_control_mode() and the publish_trajectory_setpoint() methods.
         */
        void send_message();

        /** 
         * The method that sends the message to arm the drone
         */
        void arm() const;

        /** 
         * The method that sends the message to disarm the drone
         */
        void disarm() const;

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
	msg.timestamp = timestamp_.load();
    msg.position={this->x, this->y, this->z};
	msg.yaw = 0; // [-PI:PI] //TODO set this->yaw

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


void set_last_pos(float x, float y, float z, float yaw){
    last_position.x=x;
    last_position.y=y;
    last_position.z=z;
    last_position.yaw=yaw;
}