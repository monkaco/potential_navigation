#ifndef POTENTIAL_H_
#define POTENTIAL_H_

#include <iostream>

#include <stdio.h>
#include <math.h>

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>

#include <potential/Vector2.h>

#include <boost/lexical_cast.hpp>


class Potential 
{
    public:
        /* Variable declarations */
        pose_xy& robo;
        pose_xy& alvo;
        double transition_threshold;
        double obstacle_range_threshold;
        double k_gain;

        // Potential(){
        // transition_threshold = 0;
        // } 

        /* Methods */
        double getAngularDistance (pose_xy robo, pose_xy alvo);
        double getLinearDistance (pose_xy robo, pose_xy alvo);
        
        /* ATTRACTIVE POTENTIAL */
        bool Conic_Quadratic_transition (pose_xy robo, pose_xy alvo);
        double Attractive_Potential (pose_xy robo, pose_xy alvo);
        pose_xy Gradient_Attractive (pose_xy robo, pose_xy alvo);

        /* REPULSIVE POTENTIAL */
        bool Obstacle_within_range (pose_xy robo, pose_xy alvo);
        double Repulsive_Potential (pose_xy robo, pose_xy alvo);
        pose_xy Gradient_Repulsive (pose_xy robo, pose_xy alvo);
        pose_xy Gradient_Obstacle ();

};

/*
class robot
{
    public:
        int id;       //agent ID
        float weight;     //agent weight
        std::string rname; //name of the agent on the topic
        char status;	  //status

        geometry_msgs::Twist speed; //speed ROS structure
		nav_msgs::Odometry poseOdom;    //position of the agent
		Vector2 pose;       //simpler way to represent the position
		double heading;

		ros::Publisher speedPub;    //ROS speed publisher
		ros::Subscriber poseSub; //ROS position subscriber

		void publishSpeed();
		void setSpeedPublisher(ros::NodeHandle& nh);
		void setPoseSubscriber(ros::NodeHandle& nh);
		void setSpeed(double v, double w);
		void setWeight(double p);
		double getPower();
		std::string getName();
		robot(int id, double weight,
			  std::string name, ros::NodeHandle& nh);

		void setSpeedHolo(double dx, double dy);

		void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);


};
*/

#endif /* POTENTIAL_H_ */
