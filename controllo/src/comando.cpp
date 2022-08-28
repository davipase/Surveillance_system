#include <iostream>
#include <functional>
#include <chrono>
#include <memory>
#include <string>
#include <list>
#include <thread>
#include <iterator>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/comando.hpp>
#include <math.h>

#include "../../cpp-spline/spline/src/main/cpp/CatmullRom.h"
#include "../../cpp-spline/spline/src/main/cpp/CatmullRom.cpp"

using std::placeholders::_1;



using namespace std;
using namespace std::chrono_literals;
using namespace rclcpp;
using namespace px4_msgs::msg;

struct punto{
    float x=0;
    float y=0;
    float z=0;
    // float yaw;
}p,last_position;

float last_yaw=0;


list<punto> punti;

void set_last_position(float x, float y, float z){
    last_position.x=x;
    last_position.y=y;
    last_position.z=-z;
}

float calcola_yaw(float x0, float x1, float y0, float y1){
    if((x1==x0 && y1==y0) || x1==NULL) return last_yaw;
    else{
        float yaw=atan2((y1-y0),(x1-x0));
        while(yaw > 3.14 || yaw < -3.14){
            if(yaw>3.14) yaw-=6.28;
            else yaw+=6.28;
        }
        last_yaw=yaw;
        return -yaw;
    } 

}


class Command : public Node
{
    private:
        atomic<uint64_t> timestamp_; //synchronized timestamp
        Publisher<Comando>::SharedPtr pub_trajectory_;
        // TrajectorySetpoint msg;
        Comando msg;

    public:
        Command():Node("comando"){
            pub_trajectory_ = this->create_publisher<Comando>("/command",10);
        }

        void send_message(/*float com, float x, float y, float z*/){
            // msg.x=x;
            // msg.y=y;
            // msg.z=z;
            set_last_position(msg.x,msg.y,msg.z);
            pub_trajectory_->publish(msg);
            cout<<"Comando mandato\n";
        }

        void send_trajectory(){

            Curve* curve = new CatmullRom();
            curve->add_way_point(Vector(0, 0, 0)); //Il primo viene ignorato
            curve->set_steps(50); // generate 100 interpolate points between the last 4 way points
            for (std::list<punto>::iterator it = punti.begin(); it != punti.end(); it++){
                curve->add_way_point(Vector(it->x,it->y,it->z));
            }
            curve->add_way_point(Vector(0, 0, 0)); //l'ultimo viene ignorato

            std::cout << "nodes: " << curve->node_count() << std::endl;
            std::cout << "total length: " << curve->total_length() << std::endl;

            for (int i = 0; i < curve->node_count(); ++i) {
                float yaw=calcola_yaw(curve->node(i).x,curve->node(i+1).x,curve->node(i).y,curve->node(i+1).y);
                std::cout << "node #" << i << ": " << curve->node(i).toString() << "yaw:"<<yaw<<endl;
                set_msg(3,(float)(curve->node(i).x),(float)(curve->node(i).y),(float)(curve->node(i).z),yaw);
                send_message();
                this_thread::sleep_for(chrono::milliseconds(200));
            }

            delete curve;
        }
        

        void set_msg( int c, float x, float y, float z,float yaw){
            msg.com = c;
            msg.x = x;
            msg.y = y;
            msg.z = -z;
            msg.yaw=yaw;
            set_last_position(x,y,z);
        }

        void print_message(){
            cout<<"Comando: "<<msg.com<<"\nx: "<<msg.x<<"\ny: "<<msg.y<<endl;
        }
};


int main(int argc, char* argv[]){
    init(argc, argv);
    float x=0,y=0,z=0;
    int c;
    Command cmd;
    while(rclcpp::ok()){
        cout<<"Comando:\n0:land and exit program\n1:takeoff\n2:land\n3:goto position\n4:set trajectory (minimo 4) (41: use set trajectory)\n5:set yaw (da -3.14 a +3.14)\n==>";
        cin>>c;
        switch (c){
            case 0:
            {
                cmd.set_msg(2,0,0,0,0);
                cmd.send_message();
                cout<<"Messaggio mandato\n";
                exit(0);
                break;
            }
            case 1:
            {
                cout<<"Altezza di takeoff: ";
                cin>>z;
                cmd.set_msg(1,0,0,z,0);
                set_last_position(0,0,z);
                break;
            }
            case 2:
            {
                cmd.set_msg(2,0,0,0,0);
                set_last_position(last_position.x, last_position.y,0);
                break;
            } 
            case 3:
            {
                cout <<"Coordinata x: ";
                cin >> x;
                cout <<"\nCoordinata y: ";
                cin >> y;
                cout <<"\nCoordinata z: ";
                cin >> z;
                cmd.set_msg(3,x,y,z,0);
                break;
            }
            case 4:
            {
                punti.clear();
                
                do{
                cout <<"Coordinata x: ";
                cin >> p.x;
                cout <<"\nCoordinata y: ";
                cin >> p.y;
                cout <<"\nCoordinata z: ";
                cin >> p.z;
                // cmd.set_msg(4,x,y,z);
                if(p.z>0) punti.push_back(p);
                else set_last_position(p.x,p.y,p.z);
                }while(p.z>0);
                cmd.send_trajectory();
                break;
            }
            case 41:
            {
                punti.push_back(last_position);
                p.x=1; p.y=5; p.z=2; punti.push_back(p);
                p.x=2; p.y=-3; p.z=2; punti.push_back(p);
                p.x=8; p.y=-7; p.z=5; punti.push_back(p);
                p.x=4; p.y=0; p.z=3; punti.push_back(p);
                cmd.send_trajectory();
                break;

            }
            case 5:
            {
                cout<<"Inserisci lo yaw: ";
                float yaw;
                cin >> yaw;
                cout<<"||l_p.z="<<last_position.z<<endl;
                cmd.set_msg(5,last_position.x, last_position.y, last_position.z,yaw);
                break;
            }
            default:{
                cout<<"Comando non valido, atterraggio\n";
                cmd.set_msg(2,0,0,0,0);
                break;
            }
        }
        cmd.send_message();
        cout<<"Messaggio mandato\n";
        cmd.print_message();
    }
}