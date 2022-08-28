#include <iostream>
#include <functional>
#include <chrono>
#include <memory>
#include <string>
#include <math.h>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
// #include <px4_msgs/msg/offboard_control_mode.hpp>
// #include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
// #include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/comando.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>


using namespace std;
using namespace std::chrono_literals;
using namespace rclcpp;
using namespace px4_msgs::msg;
using namespace geometry_msgs::msg;

int i=0;
float rpy[3];

void quat2eul(float q0, float q1, float q2, float q3){
    rpy[0] = atan2(2*(q0*q1 + q2*q3),1 - 2*(q1*q1 + q2*q2));
    rpy[1] = asin(2*(q0*q2 - q3*q1));
    rpy[2] = atan2(2*(q0*q3 + q1*q2),1 - 2*(q2*q2 + q3*q3));
}



class Optitrack_data : public Node
{
    private:
        std::atomic<uint64_t> timestamp_; //synchronized timestamp
        rclcpp::TimerBase::SharedPtr timer_;
        Subscription<Timesync>::SharedPtr timeSync_;
        Subscription<PoseStamped>::SharedPtr opt_sub;
        Publisher<VehicleVisualOdometry>::SharedPtr opt_data_pub;
        VehicleVisualOdometry messaggio;

    public:

    Optitrack_data():Node("optitrack_data"){
        timeSync_=this->create_subscription<Timesync>("fmu/timesync/out", 10,[this](const Timesync::UniquePtr msg) {
			timestamp_.store(msg->timestamp);
		});

        opt_sub = this->create_subscription<PoseStamped>("/natnet_ros/RigidBody/pose",10,[this](const PoseStamped::UniquePtr msg){
            if(i==0){
                //TO CHANGE CHECK FRAMES

                messaggio.timestamp = timestamp_.load();
                messaggio.timestamp_sample = timestamp_.load();
                messaggio.x = msg->pose.position.x;
                messaggio.y = msg->pose.position.y;
                messaggio.z = msg->pose.position.z;
                messaggio.q[0] = msg->pose.orientation.x;
                messaggio.q[1] = msg->pose.orientation.y;
                messaggio.q[2] = msg->pose.orientation.z;
                messaggio.q[3] = msg->pose.orientation.w;
                //DAL TOPIX: q[4] = x,y,z,w. Nella conversione: q[4]=w,x,y,z
                quat2eul(messaggio.q[3], messaggio.q[0], messaggio.q[1],messaggio.q[2]);

                cout<<"x:"<<msg->pose.position.x<<"\n";
                cout<<"y:"<<msg->pose.position.y<<"\n";
                cout<<"z:"<<msg->pose.position.z<<"\n";
                // cout<<"ox:"<<msg->pose.orientation.x<<"\n";
                // cout<<"oy:"<<msg->pose.orientation.y<<"\n";
                // cout<<"oz:"<<msg->pose.orientation.z<<"\n";
                // cout<<"w:"<<msg->pose.orientation.w<<"\n\n";
                cout<<"roll:"<<rpy[0]<<"\n";
                cout<<"pitch:"<<rpy[1]<<"\n";
                cout<<"yaw:"<<rpy[2]<<"\n\n";
                i++;
            }
            else{i++;
                i=i%10;
            }
        });

        opt_data_pub = this->create_publisher<VehicleVisualOdometry>("/fmu/vehicle_visual_odometry/in",10);
        timer_ = this->create_wall_timer(25ms,std::bind(&Optitrack_data::publish_opt_data,this));
    }


    void publish_opt_data(){
        opt_data_pub->publish(messaggio);
    }
};



int main(int argc, char* argv[]){
    cout << "Starting optitrack mocap node..." << endl;
	rclcpp::init(argc, argv); //initializing ros2
    cout<<"optitrack mocap node started"<<endl;
	rclcpp::spin(std::make_shared<Optitrack_data>()); //creating instance of class and spinning
    rclcpp::shutdown();
    return 0;
}