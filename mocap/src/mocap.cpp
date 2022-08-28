/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

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
VehicleVisualOdometry msg;



class MocapPublisher : public Node{
  private:
    Publisher<VehicleVisualOdometry>::SharedPtr pub_;

  public:
    MocapPublisher():Node("pubb"){
      pub_ = this->create_publisher<VehicleVisualOdometry>("/mocap_data",10);
    }

    void publish_mocap(){
      pub_->publish(msg);
    }
};

MocapPublisher* mp = NULL;


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
  mp->publish_mocap();

}




/////////////////////////////////////////////////
int main(int argc, char* argv[])
{
  // // Load gazebo
  // cout<<"A";
  // init(argc, argv);
  // cout<<"B";
  // gazebo::client::setup(argc, argv);

  // // Create our node for communication
  // gazebo::transport::NodePtr node(new gazebo::transport::Node());
  // node->Init();
  // // Listen to Gazebo world_stats topic
  //  gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/pose/info", cb);


  // spin(std::make_shared<MocapPublisher>());
  // while(1)
  // // Make sure to shut everything down.
  // gazebo::client::shutdown();
  // shutdown();
  // return 0;



  cout << "Starting mocap node..." << endl;
	rclcpp::init(argc, argv); //initializing ros2
  cout<<"mocap node started"<<endl;
  mp = new MocapPublisher();

  gazebo::client::setup(argc, argv);
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/pose/info", cb);


	rclcpp::spin(std::make_shared<MocapPublisher>()); //creating instance of class and spinning
  while(1);
  gazebo::client::shutdown();
  rclcpp::shutdown();
  return 0;

  
}
