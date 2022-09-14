#include <iostream>
#include <functional>
#include <chrono>
#include <memory>
#include <string>
#include <math.h>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/timesync.hpp>
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

/** 
 * A struct that stores a quaternion information
 */
struct quaternion{
    float w;
    float x;
    float y;
    float z;
};
quaternion rot_quat;
// quaternion drone_to_world;

/** 
 * \relates Optitrack_data
 * Converts the quaternion in rpy
 * \param q0 the x value
 * \param q1 the y value
 * \param q2 the z value
 * \param q3 the w value
 */
void quat2eul(float q0, float q1, float q2, float q3);

/** 
 * \relates quaternion
 * multiplies 2 quaternion, q1*q2
 */
quaternion quat_mult(quaternion q1, quaternion q2);


/**
 * \brief Message relayer fron natnet node to PX4.
 * 
 * The Optitrack_data class subscribes to the Natnet node through the Ros1 bridge to receive t information regarding the drone pose
 * from the optitrack system.
 */
class Optitrack_data : public Node{
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

        /** 
         * \relates Optitrack_data
         * rotation matrix - rotation 90 degrees around x axis
         */
        float rot_mat[3][3]={
            1,  0,  0,
            0,  0, -1,
            0,  1,  0
        };

        quaternion q;
    
    public:

        /**
         * The Constructor of the Optitrack_data class
         * It subscribes to the NatNet topic to retrieve the Mocap data from the NatNet Node (and thus from the Optitrack system)
         * Then it adjust the reference frame (from Forward-Up-Right to North-East-Down) before publishing the data to the drone
         */
        Optitrack_data():Node("Optitrack_data"){
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

                    cout <<"POSITION\n";
                    cout<<"x:"<<msg->pose.position.x<<"\n";
                    cout<<"y:"<<msg->pose.position.y<<"\n";
                    cout<<"z:"<<msg->pose.position.z<<"\n";
                    cout <<"QUATERNION\n";
                    cout<<"w:"<<message.q[0]<<"\n";
                    cout<<"x:"<<msg->pose.orientation.x<<"\n";
                    cout<<"y:"<<msg->pose.orientation.y<<"\n";
                    cout<<"z:"<<msg->pose.orientation.z<<"\n";

                    q.w=message.q[0];
                    q.x=msg->pose.orientation.x;
                    q.y=msg->pose.orientation.y;
                    q.z=msg->pose.orientation.z;

                    float buddy;
                    buddy = q.y;
                    q.y = q.z;
                    q.z = -buddy;

                    cout <<"quaternion AFTER ROTATION 90°:\n";
                    cout<<"w:"<<q.w<<"\n";
                    cout<<"x:"<<q.x<<"\n";
                    cout<<"y:"<<q.y<<"\n";
                    cout<<"z:"<<q.z<<"\n\n";

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


}



int main(int argc, char* argv[]){
    cout << "Starting optitrack mocap node..." << endl;
	rclcpp::init(argc, argv); //initializing ros2
    cout<<"optitrack mocap node started"<<endl;
	rclcpp::spin(std::make_shared<Optitrack_data>()); //creating instance of class and spinning
    rclcpp::shutdown();
    return 0;
}


void quat2eul(float q0, float q1, float q2, float q3){
    rpy[0] = atan2(2*(q0*q1 + q2*q3),1 - 2*(q1*q1 + q2*q2));
    rpy[1] = asin(2*(q0*q2 - q3*q1));
    rpy[2] = atan2(2*(q0*q3 + q1*q2),1 - 2*(q2*q2 + q3*q3));
}


void Optitrack_data::publish_opt_data(){
    opt_data_pub->publish(message);
}


quaternion quat_mult(uaternion q1, quaternion q2){
    quaternion q3;
    q3.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z-q2.z;
    q3.x = q1.x*q2.w + q1.w*q2.x - q1.z*q2.y + q1.y*q2.z;
    q3.y = q1.y*q2.w + q1.z*q2.x + q1.w*q2.y - q1.x-q2.z;
    q3.z = q1.z*q2.w - q1.y*q2.x + q1.x*q2.y + q1.w-q2.z;
    return q3;
}
