#include "ros/ros.h"
#include "std_msgs/String.h"


class Listener {

public:
    Listener();

    ~Listener();

    void chatterCallback(const std_msgs::String::ConstPtr &msg);

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
};

Listener::Listener() {
    sub = n.subscribe("chatter", 1000, &Listener::chatterCallback, this);
}

Listener::~Listener() {
}

void Listener::chatterCallback(const std_msgs::String::ConstPtr &msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    Listener listener;
    ros::spin();
    return 0;
}
