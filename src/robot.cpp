#include <potential/robot.h>

void robot::publishSpeed(){
	speedPub.publish(this->speed);
}

void robot::setSpeedPublisher(ros::NodeHandle& nh){
	std::string topicName = std::string("robot_") + boost::lexical_cast<std::string>(this->id) + std::string("/cmd_vel");
	speedPub = nh.advertise<geometry_msgs::Twist>(topicName, 10);
}

void robot::setPoseSubscriber(ros::NodeHandle& nh){
	std::string topicName = std::string("robot_") + boost::lexical_cast<std::string>(this->id) + std::string("/base_pose_ground_truth");
	poseSub = nh.subscribe(topicName.c_str(), 10, &robot::poseCallback, this); 
}

void robot::poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
	this->poseOdom = *msg;
	pose.x = poseOdom.pose.pose.position.x;
	pose.y = poseOdom.pose.pose.position.y;
	heading = tf::getYaw (poseOdom.pose.pose.orientation);
}

void robot::setSpeed(double v, double w){
	speed.linear.x = v;
	speed.angular.z = w;
}

void robot::setSpeedHolo(double dx, double dy){
	double v, w;

	double yaw = tf::getYaw(poseOdom.pose.pose.orientation);

	v = 1*(cos(yaw)*dx + sin(yaw)*dy);
    w = 0.05*(-sin(yaw)*dx/0.05 + cos(yaw)*dy/0.05);
                
	setSpeed(v, w);
}

void robot::setWeight(double p){
	weight = p;
}

double robot::getPower(){
	return weight;
}

std::string robot::getName(){
	return rname;
}

robot::robot(int id, double weight, 
	    	 std::string name, ros::NodeHandle& nh){
	this->id = id;
	this->weight = weight;
	this->rname = name;
	setPoseSubscriber(nh);
	setSpeedPublisher(nh);
}