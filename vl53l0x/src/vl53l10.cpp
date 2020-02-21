
#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/Range.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include <tof.h> // time of flight sensor library

int main(int argc, char **argv){
	int i;
	int iDistance;
	int model, revision;
	int long_range = 0;
	double poll_rate = 0;

	ros::init(argc, argv, "vl53l0x");
	ros::NodeHandle nh, nh_priv("~");

	nh_priv.param("long_range", long_range, 0);
	nh_priv.param("poll_rate", poll_rate, 100.0);

	// For Raspberry Pi's, the I2C channel is usually 1
	// For other boards (e.g. OrangePi) it's 0
	i = tofInit(0, 0x29, long_range); // set long range mode (up to 2m)
	if (i != 1){
		return -1; // problem - quit
	}

	i = tofGetModel(&model, &revision);
	std::cout << "VL53L0X device successfully opened.\n";
	std::cout << "Model ID - " << model << "\n";
	std::cout << "Revision ID - " << revision << "\n";
	if(long_range)
	    std::cout << "Long Range - " << long_range << "\n";

	ROS_INFO("VL53L0X: start ranging");

	sensor_msgs::Range range;
	ros::Publisher range_pub = nh_priv.advertise<sensor_msgs::Range>("range", 20);

	ros::Rate r(poll_rate);
	while (ros::ok()) {
	    r.sleep();
	    iDistance = tofReadDistance();
	    if (iDistance < 4096){
		//std::cout << "Distance = " << iDistance << "\n";
		range.range = iDistance / 1000.0;
		range_pub.publish(range);
	    }

	    ros::spinOnce();
	}

	ROS_INFO("VL53L0X: stop ranging");

	return 0;
}




