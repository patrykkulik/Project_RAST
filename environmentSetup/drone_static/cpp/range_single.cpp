#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include "gdp2020.h"

double ultra_range_0b;
double ultra_range_1b;
double ultra_range_2b;
double ultra_range_3b;
double ultra_range_0t;
double ultra_range_1t;
double ultra_range_2t;
double ultra_range_3t;
double bottom_max;
double top_max;

void max()
{
	bottom_max = ultra_range_0b + 0.2;
	if (ultra_range_1b + 0.2 > bottom_max)
	{
		bottom_max = ultra_range_1b + 0.2;
	}
	else if (ultra_range_2b + 0.2 > bottom_max)
	{
		bottom_max = ultra_range_2b + 0.2;
	}
	else if (ultra_range_3b + 0.2 > bottom_max)
	{
		bottom_max = ultra_range_3b + 0.2;
	}
	else
	{
		;
	}

	top_max = ultra_range_0t;
	if (ultra_range_1t > top_max)
	{
		top_max = ultra_range_1t;
	}
	else if (ultra_range_2t > top_max)
	{
		top_max = ultra_range_2t;
	}
	else if (ultra_range_3t > top_max)
	{
		top_max = ultra_range_3t;
	}
	else
	{
		;
	}
}



void range_cb_0b(const sensor_msgs::Range::ConstPtr& msg)
{	
	ultra_range_0b = msg->range;
}

void range_cb_1b(const sensor_msgs::Range::ConstPtr& msg)
{	
	ultra_range_1b = msg->range;		
}

void range_cb_2b(const sensor_msgs::Range::ConstPtr& msg)
{	
	ultra_range_2b = msg->range;
}

void range_cb_3b(const sensor_msgs::Range::ConstPtr& msg)
{	
	ultra_range_3b = msg->range;
}

// TOP BOTTOM TOP BOTTOM TOP BOTTOM TOP BOTTOM TOP BOTTOM

void range_cb_0t(const sensor_msgs::Range::ConstPtr& msg)
{	
	ultra_range_0t = msg->range;
}

void range_cb_1t(const sensor_msgs::Range::ConstPtr& msg)
{	
	ultra_range_1t = msg->range;
}

void range_cb_2t(const sensor_msgs::Range::ConstPtr& msg)
{	
	ultra_range_2t = msg->range;
}

void range_cb_3t(const sensor_msgs::Range::ConstPtr& msg)
{	
	ultra_range_3t = msg->range;
	max();
}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "range_sub");
	ros::NodeHandle n;
	
	ros::Subscriber sub_0b = n.subscribe("/distance_0_bottom", 1, range_cb_0b);
	ros::Subscriber sub_1b = n.subscribe("/distance_1_bottom", 1, range_cb_1b);
	ros::Subscriber sub_2b = n.subscribe("/distance_2_bottom", 1, range_cb_2b);
	ros::Subscriber sub_3b = n.subscribe("/distance_3_bottom", 1, range_cb_3b);
	ros::Subscriber sub_0t = n.subscribe("/distance_0_top", 1, range_cb_0t);
	ros::Subscriber sub_1t = n.subscribe("/distance_1_top", 1, range_cb_1t);
	ros::Subscriber sub_2t = n.subscribe("/distance_2_top", 1, range_cb_2t);
	ros::Subscriber sub_3t = n.subscribe("/distance_3_top", 1, range_cb_3t);
	ros::spin();

	return 0;
}
