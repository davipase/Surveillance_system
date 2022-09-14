#include <iostream>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <string>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std;
using namespace std::chrono_literals;
using namespace rclcpp;
using namespace px4_msgs::msg;

int i=0;

/**
 * \relates Gazebo_mocap_publisher
 * Message that is going to be sent to the mocap_sim node
 */
VehicleVisualOdometry msg;

/**
 * \relates Gazebo_mocap_publisher
 * Pointer to an instance of the Gazebo_mocap_publisher Class.
 */
Gazebo_mocap_publisher* gaz_pub = NULL;

/**
* \relates Gazebo_mocap_publisher
* Function used to parse the string to extrapolates the value of x,y,z.
* \param _msg the string retrieved from Gazebo cotaining the pose of all the rigid bodies in the simulation.
*/
void cb(ConstPosesStampedPtr &_msg);


/** \brief Class that retrive position information from Gazebo.
*
* The Gazebo_mocap_publisher class subscribes to Gazebo and retrieve the position data of the drone, data that is then
* sent to the mocap_sim node.
*/
class Gazebo_mocap_publisher : public Node{
  private:
  
    Publisher<VehicleVisualOdometry>::SharedPtr pub_;

  public:
    /**
    * Constructor. Initializes the publisher.
    */
    Gazebo_mocap_publisher():Node("pubb"){
      pub_ = this->create_publisher<VehicleVisualOdometry>("/mocap_data",10);
    }
    /**
    * publishes the message
    */
    void publish_mocap();
};




int main(int argc, char* argv[])
{
  cout << "Starting mocap node..." << endl;
  rclcpp::init(argc, argv); //initializing ros2
  cout<<"mocap node started"<<endl;
  gaz_pub = new Gazebo_mocap_publisher();

  gazebo::client::setup(argc, argv);
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/pose/info", cb);


  rclcpp::spin(std::make_shared<Gazebo_mocap_publisher>()); //creating instance of class and spinning
  while(1);
  gazebo::client::shutdown();
  rclcpp::shutdown();
  return 0;
}


void cb(ConstPosesStampedPtr &_msg)
{
  // Dump the message contents to stdout.
  string s = _msg->DebugString();
  cout<<s<<endl;
  int p = s.find('"iris"');
  int px,py,pz;
  cout<<p<<endl;

  //parsing x
  p+=36;
  cout<<"s[p]:"<<s.substr(p,10)<<endl;
  px=p;
  while(s[p]!=32) p++; //32 = spazio
  string x = s.substr(px,p-px);
  cout<<"x:"<<x<<endl;

  //parsing y
  while(s[p]!= 58) p++;  //58 = :
  p+=2;
  py=p;
  while(s[p]!=32) p++;
  string y = s.substr(py,p-py);
  cout<<"y:"<<y<<endl;

  //parsing z
  while(s[p]!= 58) p++;  //58 = :
  p+=2;
  pz=p;
  while(s[p]!=32) p++;
  string z = s.substr(pz,p-pz);
  cout<<"z:"<<z<<endl;

  int pox,poy,poz,ow;

  //parsing orientation-x
  while(s[p]!= 58) p++;  //58 = :
  p+=2;
  pox=p;
  while(s[p]!=32) p++;
  string ox = s.substr(pox,p-pox);
  cout<<"ox:"<<ox<<endl;

  //parsing orientation-y
  while(s[p]!= 58) p++;  //58 = :
  p+=2;
  poy=p;
  while(s[p]!=32) p++;
  string oy = s.substr(poy,p-poy);
  cout<<"oy:"<<oy<<endl;

  //parsing orientation-z
  while(s[p]!= 58) p++;  //58 = :
  p+=2;
  poz=p;
  while(s[p]!=32) p++;
  string oz = s.substr(poz,p-poz);
  cout<<"oz:"<<oz<<endl;

  //parsing orientation-w
  while(s[p]!= 58) p++;  //58 = :
  p+=2;
  ow=p;
  while(s[p]!=32) p++;
  string w = s.substr(ow,p-ow);
  cout<<w<<endl;


  msg.x=stof(x);
  msg.y=stof(y);
  msg.z=stof(z);
  float q[4];
  msg.q[0] = stof(ox);
  msg.q[1] = stof(oy);
  msg.q[2] = stof(oz);
  msg.q[3] = stof(w);
  // msg.q = q;
  gaz_pub->publish_mocap();

}


void publish_mocap(){
  pub_->publish(msg);
}