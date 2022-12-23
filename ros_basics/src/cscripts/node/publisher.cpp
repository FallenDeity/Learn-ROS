#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

class Talker {
public:
    Talker();

    void run();

private:
    ros::NodeHandle n;
    ros::Publisher chatter_pub;
};

Talker::Talker() {
    chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
}

void Talker::run() {
    ros::Rate loop_rate(10);

    int count{0};
    while (ros::ok()) {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "Hello world!" << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");

    Talker talker;
    talker.run();

    return 0;
}
