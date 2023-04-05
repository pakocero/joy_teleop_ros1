#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread.hpp>

class teleop
{
    public:
    teleop();
    void run();
    void speed_control();
    void key_input();


    private:
    ros::NodeHandle _nh;
    ros::Publisher _pub;
    float _lin_vel;
    float _ang_vel;
    std::string cmd_vel_topic;
    double max_lin_vel;
    double max_ang_vel;
};

teleop::teleop()
{
     if(!_nh.getParam("max_lin_vel", max_lin_vel))
    {
        max_lin_vel = 0.5;
    }

    if(!_nh.getParam("max_ang_vel", max_ang_vel))
    {
        max_ang_vel = 0.5;
    }

    if(!_nh.getParam("cmd_vel_topic", cmd_vel_topic))
    {
        cmd_vel_topic = "cmd_vel";
    }

    _pub = _nh.advertise< geometry_msgs::Twist >(cmd_vel_topic, 1);
    _lin_vel = 0.0;
    _ang_vel = 0.0;
}

void teleop::run()
{
    boost::thread speed_control_th(&teleop::speed_control, this);
    boost::thread key_input_th(&teleop::key_input, this);
    ros::spin();
}

void teleop::speed_control()
{
    ros::Rate r(10);
    geometry_msgs::Twist msg;

    while(ros::ok())
    {
        msg.linear.x = _lin_vel;
        msg.angular.z = _ang_vel;

        _pub.publish(msg);

        r.sleep();
    }
}

void teleop::key_input()
{
    std::string input;

    while(ros::ok())
    {
        std::cout<<"w = forward"<<std::endl;
        std::cout<<"x = backward"<<std::endl;
        std::cout<<"a = rotate left"<<std::endl;
        std::cout<<"d = rotate right"<<std::endl;
        std::cout<<"s = stop all movements"<<std::endl;

        std::cin >> input;

        if(input == "w" || input == "W")
        {
            _lin_vel = max_lin_vel;
            _ang_vel = 0;
        }
        else if(input == "a" || input == "A")
        {
            _ang_vel = max_ang_vel;
        }
        else if(input == "d" || input == "D")
        {
            _ang_vel = -max_ang_vel;
        }
        else if(input == "x" || input == "X")
        {
            _lin_vel = -max_lin_vel;
            _ang_vel = 0;
        }
        else if(input == "s" || input == "S")
        {
            _lin_vel = _ang_vel = 0;
        }
        else
        {
            std::cout<<"Invalid input"<<std::endl;
        }
    }
}


int main(int argc, char**argv)
{
    ros::init(argc, argv, "key_teleop");

    teleop T;
    T.run();

    return 0;
}
