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


/**
 * \brief Message relayer fron natnet node to PX4.
 * 
 * The Optitrack_data class subscribes to the Natnet node through the Ros1 bridge to receive t information regarding the drone pose
 * from the optitrack system.
 */
class Optitrack_data : public Node
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
         * Publisher that sends the position data to the PX4.
         */
        Publisher<VehicleVisualOdometry>::SharedPtr opt_data_pub;

        /** 
         * Subscriber that gets information from the Natnet node
         */
        Subscription<PoseStamped>::SharedPtr opt_sub;

        /** 
         * Subscriber that receive the timestamp to be stored in the timestamp_ variable from the PX4.
         */ 
        Subscription<Timesync>::SharedPtr timeSync_;

        /** 
         * Message to be sent to the PX4
         */ 
        VehicleVisualOdometry message;

    public:

        /**
         * The constructor initialize all the Publishers and subscribers with the correct topic, and creates the callback function for the
         * subscribers. The timeSync_ subscriber simply receives the timestamp to be set in every message sent, while the opt_sub receive data
         * from the Natnet node and store it in the message variable.
         */
        Optitrack_data():Node("optitrack_data"){
            timeSync_=this->create_subscription<Timesync>("fmu/timesync/out", 10,[this](const Timesync::UniquePtr msg) {
                timestamp_.store(msg->timestamp);
            });

            opt_sub = this->create_subscription<PoseStamped>("/natnet_ros/RigidBody/pose",10,[this](const PoseStamped::UniquePtr msg){
                message.timestamp = timestamp_.load();
                message.timestamp_sample = timestamp_.load();
                message.x = msg->pose.position.x;
                message.y = msg->pose.position.y;
                message.z = msg->pose.position.z;
                message.q[0] = msg->pose.orientation.x;
                message.q[1] = msg->pose.orientation.y;
                message.q[2] = msg->pose.orientation.z;
                message.q[3] = msg->pose.orientation.w;

                //FROM THE TOPIC: q[4] = x,y,z,w. In the conversion function: q[4]=w,x,y,z
                quat2eul(message.q[3], message.q[0], message.q[1],message.q[2]);

                if(i==0){
                    //TO CHANGE CHECK FRAMES
                    cout<<"x:"<<msg->pose.position.x<<"\n";
                    cout<<"y:"<<msg->pose.position.y<<"\n";
                    cout<<"z:"<<msg->pose.position.z<<"\n";
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

        /** 
         * Method that publishes the message to the PX4.
         */
        void publish_opt_data();
};



int main(int argc, char* argv[]){
    cout << "Starting optitrack mocap node..." << endl;
	rclcpp::init(argc, argv); //initializing ros2
    cout<<"optitrack mocap node started"<<endl;
	rclcpp::spin(std::make_shared<Optitrack_data>()); //creating instance of class and spinning
    rclcpp::shutdown();
    return 0;
}


void publish_opt_data(){
            opt_data_pub->publish(message);
        }