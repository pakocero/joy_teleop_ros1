#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/thread.hpp>

//#define DEBUG_PUB // enable published message printed to stdout for debugging
//#define DEBUG_SUB // enable message from subscriber printed to stdout for debugging

class turtlebot_joy_teleop
{
    private:
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    ros::Publisher _pub;

    sensor_msgs::Joy _joy_data;

    // PARAMETERS
    double max_lin_vel;
    double max_ang_vel;
    std::string cmd_vel_topic;

    bool first_joy_msg_ready;

    void sub_callback(sensor_msgs::JoyConstPtr msg);
    void pub_thread();

    public:
    turtlebot_joy_teleop();
    void run();

};

void turtlebot_joy_teleop::sub_callback(sensor_msgs::JoyConstPtr msg)
{
    _joy_data = *msg;

    #ifdef DEBUG_SUB
    ROS_INFO("joy data axis 0: %f", _joy_data.axes[0]);
    ROS_INFO("joy axis 1: %f", _joy_data.axes[1]);
    ROS_INFO("joy button square: %d",_joy_data.buttons[3]);
    #endif

    first_joy_msg_ready = true;
}

void turtlebot_joy_teleop::pub_thread()
{
    ros::Rate r(50);

    ROS_INFO("Move the left analogic lever to move the robot along x axis");
    ROS_INFO("Move the right analogic lever to move the robot around z axis");
    ROS_INFO("Press square button and hold to stop everything");

    while(!first_joy_msg_ready)
    {
        usleep(100);
    }

    while(ros::ok())
    {
        geometry_msgs::Twist msg;

        msg.linear.x = _joy_data.axes[1]*max_lin_vel;
        msg.angular.z = _joy_data.axes[2]*max_ang_vel;

        if(_joy_data.buttons[3])
        {
            msg.linear.x = 0;
            msg.angular.z = 0;
        }
        
        #ifdef DEBUG_PUB
        ROS_INFO("lin: %f    ang: %f", msg.linear.x, msg.angular.z);
        #endif

        _pub.publish(msg);

        r.sleep();
    }

}

turtlebot_joy_teleop::turtlebot_joy_teleop()
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

    _sub = _nh.subscribe("/joy", 1, &turtlebot_joy_teleop::sub_callback, this);
    _pub = _nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1, this);

    first_joy_msg_ready = false;
}

void turtlebot_joy_teleop::run()
{
    boost::thread pub_thread_t(&turtlebot_joy_teleop::pub_thread, this);
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtlebot_joy_teleop");
    turtlebot_joy_teleop joy;
    ROS_INFO("turtlebot_joy_teleop started");
    joy.run();
}
