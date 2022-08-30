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

//This offsets are based on the spawning position of the drone if Gazebo.
//if it is different that these values, change them accordingly.
#define OFFSET_X 1.009999
#define OFFSET_Y 0.9799
#define OFFSET_Z 0.1044


using namespace std;
using namespace std::chrono_literals;
using namespace rclcpp;
using namespace px4_msgs::msg;

int i=0;

/** \brief Class that simulates a motion capture system in gazebo.
 *
 * The Mocap_sim class is able to simulate a Motion Capture system inside the Gazebo simulation.
 * It gets the position information from the Gazebo_mocap node and the rotation data from the odometry topic of the PX4.
 * It then relays the position information to the drone.
 * For this jnode to work it is compulsory to HANGE THE PX4-AUTOPILOT PARAMETERS accordingly (check for instruction in the README file).
 */
class Mocap_sim : public Node{
    private:

        /** 
         * An atomic usigned int variable where to store the timestamp.
         */
        atomic<uint64_t> timestamp_;

        /** 
         * The timer used for the wall_timer. It will execute the pub_odometry() function every 25 milliseconds.
         */
        rclcpp::TimerBase::SharedPtr timer_;

        /** 
         * Publisher that sends the position data to the PX4.
         */
        Publisher<VehicleVisualOdometry>::SharedPtr pub_mocap_odometry;

        /** 
         * The Timesync subscriber. It will receive from the PX4 the timestamp to be stored in the timetamp_ variable and to be set in every message sent.
         */ 
        Subscription<Timesync>::SharedPtr timeSync_;

        /** 
         * Subscriber that receives rotaion information (and other parameters that stay the same) from the PX4.
         */
        Subscription<VehicleOdometry>::SharedPtr sub_mocap_odometry;

        /** 
         * Subscriber that receives the position data from the Gazebo_mocap node.
         */
        Subscription<VehicleVisualOdometry>::SharedPtr sub_position_data;

        /** 
         * Variable where the message information is stored.
         */
        VehicleVisualOdometry msg;

    public:
        /** 
         * Mocap_sim constructor.
         * it initialize the publishers and subscribers and creates the callbas functions.
         * It fuses the message information coming from the two sources into a single mesge that is later sent to the drone
         * with a frequency of 40hz.
         */
        Mocap_sim():Node("odometry_prova"){
            timeSync_=this->create_subscription<Timesync>("/fmu/timesync/out", 10,[this](const Timesync::UniquePtr msg) {
			    timestamp_.store(msg->timestamp);
			});

            sub_mocap_odometry = this->create_subscription<VehicleOdometry>("/fmu/vehicle_odometry/out",10,[this](const VehicleOdometry::UniquePtr msg_){
                this->msg.timestamp = timestamp_.load();
                this->msg.timestamp_sample = msg_->timestamp_sample;
                this->msg.local_frame = 20;//msg->local_frame; 


                this->msg.q = msg_->q;
                this->msg.q_offset = msg_->q_offset;
                this->msg.pose_covariance = msg_->pose_covariance;
                this->msg.velocity_frame = msg_->velocity_frame;
                this->msg.vx = msg_->vx;
                this->msg.vy = msg_->vy;
                this->msg.vz = msg_->vz;
                this->msg.rollspeed = msg_->rollspeed;
                this->msg.pitchspeed = msg_->pitchspeed;
                this->msg.yawspeed = msg_->yawspeed;
                this->msg.velocity_covariance = msg_->velocity_covariance;

                if(i==10){
                    cout<<"position msg: ["<<msg_->x<<", "<<msg_->y<<", "<<msg_->z<<"]\n";
                    cout<<"position this: ["<<msg.x<<", "<<msg.y<<", "<<msg.z<<"]\n\n";
                    i=0;
                }else i++;
            }); //TO CHANGE UNCOMMENT

            sub_position_data = this->create_subscription<VehicleVisualOdometry>("/mocap_data",10,[this](const VehicleVisualOdometry::UniquePtr msg){
                this->msg.x = (msg->x - OFFSET_X);
                this->msg.y = -(msg->y - OFFSET_Y);
                this->msg.z = -(msg->z - OFFSET_Z);
            });


            pub_mocap_odometry = this->create_publisher<VehicleVisualOdometry>("/fmu/vehicle_visual_odometry/in",10);
            timer_ = this->create_wall_timer(25ms,std::bind(&Mocap_sim::pub_odometry,this));
        }

        /** 
        * Method that publishes the position message.
        */
        void pub_odometry();


};


int main(int argc, char* argv[]){
    cout << "Starting mocap prova..." << endl;
	rclcpp::init(argc, argv); //initializing ros2
    cout<<"mocap prova node started"<<endl;
	rclcpp::spin(std::make_shared<Mocap_sim>()); //creating instance of class and spinning
    rclcpp::shutdown();
    return 0;
}



void Mocap_sim::pub_odometry(){
    pub_mocap_odometry->publish(msg);
}