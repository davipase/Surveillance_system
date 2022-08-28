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
#include <px4_msgs/msg/vehicle_mocap_odometry.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <math.h>


#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
// #include <std_msgs/header.hpp>
// #include <gazebo/msg/poses_stamped.hpp>
// #include <geometry_msgs/msg/pose_stamped_array.hpp>

#define OFFSET_X 1.009999
#define OFFSET_Y 0.9799
#define OFFSET_Z 0.1044



using namespace std;
using namespace std::chrono_literals;
using namespace rclcpp;
using namespace px4_msgs::msg;

int i=0;


class Prova_mocap : public Node{
    private:
        atomic<uint64_t> timestamp_;
        rclcpp::TimerBase::SharedPtr timer_;
        Subscription<Timesync>::SharedPtr timeSync_;

        Subscription<VehicleVisualOdometry>::SharedPtr sub_mocap_odometry;
        // Subscription<VehicleVisualOdometry>::SharedPtr sub_mocap_data;
        Publisher<VehicleVisualOdometry>::SharedPtr pub_mocap_odometry;
        VehicleVisualOdometry msg;

        // VehicleVisualOdometry msg;

        // Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_mocap_odometry_gz;

        // Publisher<VehicleMocapOdometry>::SharedPtr pub_mocap_odometry;
        // VehicleMocapOdometry msg;

    public:
        Prova_mocap():Node("odometry_prova"){
            timeSync_=this->create_subscription<Timesync>("/fmu/timesync/out", 10,[this](const Timesync::UniquePtr msg) {
			    timestamp_.store(msg->timestamp);
			});

            sub_mocap_odometry = this->create_subscription<VehicleVisualOdometry>("/qualisys_mocap_data",10,[this](const VehicleVisualOdometry::UniquePtr msg_){
                this->msg.timestamp = timestamp_.load();
                this->msg.timestamp_sample = timestamp_.load();
                this->msg.local_frame = 0;//msg->local_frame; 

                //ATTENTO REFERENECE FRAME: PX4 USA NED, GAZEBO Usa FLU, COSA USA QUALISYS?????? CHANGE
                this->msg.x = msg_->x;
                this->msg.y = -msg_->y;
                this->msg.z = -msg_->z;

                this->msg.q = msg_->q;

                if(i==10){
                    cout<<"position msg: ["<<msg_->x<<", "<<msg_->y<<", "<<msg_->z<<"]\n";
                    cout<<"position this: ["<<msg.x<<", "<<msg.y<<", "<<msg.z<<"]\n\n";
                    i=0;
                }else i++;
            }); //TO CHANGE UNCOMMENT

            // sub_mocap_data = this->create_subscription<VehicleVisualOdometry>("/mocap_data",10,[this](const VehicleVisualOdometry::UniquePtr msg){


            //     this->msg.x = (msg->y - OFFSET_Y);
            //     this->msg.y = (msg->x - OFFSET_X);
            //     this->msg.z = -(msg->z - OFFSET_Z);
            // });


            pub_mocap_odometry = this->create_publisher<VehicleVisualOdometry>("/fmu/vehicle_visual_odometry/in",10);
            timer_ = this->create_wall_timer(25ms,std::bind(&Prova_mocap::pub_odometry,this));

            // pub_mocap_odometry = this->create_publisher<VehicleMocapOdometry>("/fmu/vehicle_mocap_odometry/in",10);
            // timer_ = this->create_wall_timer(25ms,std::bind(&Prova_mocap::pub_odometry,this));

        }

        void pub_odometry(){
            // msg.timestamp=timestamp_.load();
            // msg.local_frame=14; //MAV_FRAME_MOCAP_NED
            // msg.x=1;
            // msg.y=1;
            // msg.z=0;
            pub_mocap_odometry->publish(msg);
        }


};


int main(int argc, char* argv[]){
    cout << "Starting mocap prova..." << endl;
	rclcpp::init(argc, argv); //initializing ros2
    cout<<"mocap prova node started"<<endl;
	rclcpp::spin(std::make_shared<Prova_mocap>()); //creating instance of class and spinning
    rclcpp::shutdown();
    return 0;
}