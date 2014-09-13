#include <potential/robot.h>
#include "ros/ros.h" 
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/GetMap.h>
#include "nav_msgs/Odometry.h" 
#include "geometry_msgs/Twist.h" 
#include "sensor_msgs/LaserScan.h" 
#include <cmath> 
#include <tf/LinearMath/Quaternion.h> 
#include <tf/LinearMath/Matrix3x3.h> 

class pose_xy{
    public:
        double x;
        double y;
        double heading;


        static double getAngularDistance(pose_xy& p1, pose_xy& p2) 
        { 
            return atan2(p2.y - p1.y, p2.x - p1.x)  -p1.heading; 
        } 

        static double getLinearDistance(pose_xy& p1, pose_xy& p2)  
        { 
            return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2)); 
        }

        pose_xy(double xp = 0, double yp = 0, double hp = 0)
        {
            x = xp;
            y = yp;
            heading = hp;
        }
};

class goal{
    public:
        Vector2 pose;
        nav_msgs::Odometry poseOdom;

        int newPose;
        int onNav;

        ros::Subscriber goalPose;

        void poseCallback(nav_msgs::OdometryConstPtr& msg){
            this->poseOdom = *msg;
            pose.x = poseOdom.pose.pose.position.x;
            pose.y = poseOdom.pose.pose.position.y;

            newPose = 1;
        }

        goal(ros::NodeHandle& nh)
        {
            goalPose = nh.subscribe("/robot_1/base_pose_ground_truth", 1, &goal::poseCallback, this);
            newPose = 0;
            onNav = 0;
        }

        void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
        {
            pose.x = msg->pose.pose.position.x;
            pose.y = msg->pose.pose.position.y;
        }
};

class Base : public pose_xy{ 
public: 
    pose_xy p; 

    Base(){}

    void baseCallback(const nav_msgs::Odometry::ConstPtr& msg) { 
        double roll, pitch; 
        x = (-1)* msg->pose.pose.position.y; 
        y = msg->pose.pose.position.x; 
        printf("Getting base!!!! %f %f\n", x, y);
        tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, \
                                          msg->pose.pose.orientation.y, \
                                          msg->pose.pose.orientation.z, \
                                          msg->pose.pose.orientation.w); 
        tf::Matrix3x3(q).getRPY(roll, pitch, heading); 
    } 
}; 

class Pose : public pose_xy{ 
public: 

    Pose() {} 
    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) { 
        double roll, pitch; 
        x = msg->pose.pose.position.x; 
        y = msg->pose.pose.position.y;
        printf("Getting pose!!!! %f %f\n", x, y); 

        tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, \
                                          msg->pose.pose.orientation.y, \
                                          msg->pose.pose.orientation.z, \
                                          msg->pose.pose.orientation.w); 
        tf::Matrix3x3(q).getRPY(roll, pitch, heading); 
    } 
}; 


class Laser{
    sensor_msgs::LaserScan laser;
public:
    Laser() {}

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
    { 
        laser = *msg;
    } 
    
    pose_xy getMinimumDistanceRobotToPoint (robot& dummy_pose, robot& robot_pose)
    {
        float minimum = laser.range_max;
        int min_index = 0;
        for(int i = 0; i < laser.ranges.size(); i++)
        {
            if (laser.ranges[i] > laser.range_min && laser.ranges[i] < minimum){
                minimum = laser.ranges[i];
                min_index = i;
            }
        }
        pose_xy point;
        double ang_point =robot_pose.heading - (laser.angle_increment * min_index + laser.angle_min);

        point.x = robot_pose.pose.x + minimum*cos(ang_point);
        point.y = robot_pose.pose.y + minimum*sin(ang_point);
        point.heading = ang_point;

        return point;
    }
    

    pose_xy getPointMinimumDistanceGoal(pose_xy goal, pose_xy robot_pose)
    {
        float laserDist;
        double ang_point, dist;
        double minDist;
        pose_xy minPoint, point;
        for(int i = 0; i < laser.ranges.size(); i++)
        {
            if (laser.ranges[i] > laser.range_min){
                laserDist = laser.ranges[i];
                
                ang_point = goal.heading - (laser.angle_increment * i + laser.angle_min);

                point.x = robot_pose.x + laserDist*cos(ang_point);
                point.y = robot_pose.y + laserDist*sin(ang_point);
                point.heading = ang_point;

                dist = pose_xy::getLinearDistance(goal, point);
                if(dist < minDist)
                    minPoint = point;
            }
        }
        return minPoint;
    }

    
    // checks if there is an obstacle in front of the robot
    bool obstacle_in_path(pose_xy goal, pose_xy robot)
    {
        float phi = atan2(goal.y, goal.x);
        float rho = robot.heading - phi;

        float ang = -laser.angle_min - rho;

        int index = ang / laser.angle_increment;
        if(laser.ranges[index] > laser.range_min && laser.ranges[index] < laser.range_max){
            return true;
        }
        else{
            return false;
        }
    }
    

    
    pose_xy find_dReach(pose_xy goal, pose_xy robot)
    {
        float phi = atan2(goal.y, goal.x);
        float rho = robot.heading - phi;

        float ang = -laser.angle_min - rho;

        int index = ang / laser.angle_increment;
        
        float dist = laser.ranges[index];

        pose_xy dReach;
        dReach.x = dist*cos(phi) + robot.x;
        dReach.y = dist*sin(phi) + robot.y;
        dReach.heading = phi;
        return dReach;
    }
    

    /*
    pose_xy findClosestTangentPoint(pose_xy goal, pose_xy robot_pose)
    {

        float laserDist;
        double ang_point, dist;
        double minDist;
        pose_xy minPoint, point;
        bool onObstacle;

        for(int i = 0; i < laser.ranges.size(); i++)
        {
            if (laser.ranges[i] > laser.range_min){

                if( (laser.ranges[i] < laser.range_max && onObstacle == false) || (laser.ranges[i] >= laser.range_max && onObstacle == true))
                {
                    laserDist = laser.ranges[i];
                    
                    ang_point = goal.heading - (laser.angle_increment * i + laser.angle_min);

                    point.x = robot_pose.x + laserDist*cos(ang_point);
                    point.y = robot_pose.y + laserDist*sin(ang_point);
                    point.heading = ang_point;

                    dist = pose_xy::getLinearDistance(goal, point) +  pose_xy::getLinearDistance(point, robot_pose);
                    if(dist < minDist)
                        minPoint = point;
                }
            }
        }
        return minPoint;
    }
    */

};


class StageBot { 
public: 
    Base base; 
    Pose pose; 
    Laser laser;
    double d; 
    double kv;
    double kw;

    StageBot(ros::NodeHandle& nh, int robotID, double dist=0.1, double kvel = 1, double kwp = 0.05): ID(robotID) 
    { 
        commandPub = nh.advertise<geometry_msgs::Twist>("/robot_" + \
                                                        boost::lexical_cast<std::string>(robotID) + \
                                                        "/cmd_vel", 1); 
        laserSub = nh.subscribe("/robot_" +\
                                boost::lexical_cast<std::string>(robotID) +\
                                "/base_scan", 1, \
                                &Laser::laserCallback, &laser); 

        baseSub = nh.subscribe("/robot_" + \
                               boost::lexical_cast<std::string>(robotID) + \
                               "/base_pose_ground_truth", 1, \
                               &Base::baseCallback, &base ); 

        poseSub = nh.subscribe("/robot_" + \
                               boost::lexical_cast<std::string>(robotID) + \
                               "/odom", 1, \
                               &Pose::poseCallback, &pose ); 
        this->d = dist;
        this->kv = kvel;
        this->kw = kwp;
    } 

    void move(double xvel, double yvel) 
    { 
        geometry_msgs::Twist msg; 

        double v, w;

        v = kv*( cos(base.heading)*xvel + sin(base.heading)*yvel );
        w = kw*( -sin(base.heading)*yvel/d + cos(base.heading)*yvel/d );
                

        msg.linear.x = v; 
        msg.angular.z = w; 
        commandPub.publish(msg); 
    } 

    

protected: 
    ros::Publisher commandPub; 
    ros::Subscriber laserSub; 
    ros::Subscriber baseSub; 
    ros::Subscriber poseSub; 
    int ID; 
}; 

class Potential
{
    public:
        /* Variable declarations */
        robot& robo;
        goal& alvo;
        double transition_threshold;
        double obstacle_range_threshold;
        double k_gain;

        /* Methods */
        double getAngularDistance () 
        { 
            return atan2(alvo.pose.y - robo.pose.y, \
                        alvo.pose.x - robo.pose.x) \
                        -robo.heading; 
        } 

        double getLinearDistance () 
        { 
            return sqrt(pow(alvo.pose.x - robo.pose.x, 2) \
                    + pow(alvo.pose.y - robo.pose.y, 2)); 
        }

        /* ATTRACTIVE POTENTIAL */
        bool Conic_Quadratic_transition ()
        {
            bool returnval;
            double dist;

            dist = getLinearDistance ();
            if (dist <= transition_threshold)
            {
                returnval = true;  /* Quadratic */
            }
            else
            {
                returnval = false;   /* Conic */
            }
                return returnval;
        }

        double Attractive_Potential ()
        {
            double returnval;
            double d_goal_star;
            double dist;

            d_goal_star = transition_threshold;
            dist = getLinearDistance ();

            if (Conic_Quadratic_transition ())  /* true -> quadratic function */
            {
                returnval = (1/2) * k_gain * pow (dist, 2);
            }
            else                                /* false -> conic function */
            {
                returnval = d_goal_star * k_gain * dist - (1/2) * k_gain * pow (d_goal_star, 2);
            }

            return returnval;
        }

        Vector2 Gradient_Attractive ()
        {
            Vector2 returnval;
            double dist;
            double d_goal_star = transition_threshold;
            
            dist = getLinearDistance ();

            if (Conic_Quadratic_transition ())  /* true -> quadratic function */
            {
                returnval.x = k_gain * (robo.pose.x - alvo.pose.x);
                returnval.y = k_gain * (robo.pose.y - alvo.pose.y);
            }
            else                                /* false -> conic function */
            {
                returnval.x = d_goal_star * k_gain * (robo.pose.x - alvo.pose.x)/dist;
                returnval.y = d_goal_star * k_gain * (robo.pose.y - alvo.pose.y)/dist;
            }

            return returnval;
        }

        /* REPULSIVE POTENTIAL */
        bool Obstacle_within_range ()
        {
            bool returnval;
            double dist;

            dist = getLinearDistance ();
            if (dist <= obstacle_range_threshold)
            {
                returnval = true;  /* Obstacle within range */
            }
            else
            {
                returnval = false;   /* Obstacle out of range */
            }
            
            return returnval;
        }

        double Repulsive_Potential ()
        {
            double returnval;
            double dist;
            double eta = 1.0; /* Repulsive gain */
            double d_obs;   /* Distance obtained from the map (algorithm brushfire) */
            double Q_star = obstacle_range_threshold;

            /* Treat this situation */
            //dist = getLinearDistance (1/d_obs, 1/Q_star);

            if (Obstacle_within_range ())   /* true -> within range */
            {
                returnval = (1/2) * eta * pow( dist, 2);
            }
            else                            /* false -> out of range */
            {
                returnval = 0.0;
            }

            return returnval;
         }

         Vector2 Gradient_Repulsive ()
        {
            
            // Q_star = obstacle_range_threshold;
            // d_obs;   

            // dist = getLinearDistance (1/d_obs, 1/Q_star);

            // if (Obstacle_within_range ())  /* true -> within range */
            // {
            //    returnval = eta * dist * ( 1/pow(d_obs, 2) ) * Gradient_Obstacle(q_robot);
            // }
            // else                                 false -> conic function 
            // {
            //     returnval = 0.0;
            // }

            // return returnval;
        }
            

        Vector2 Gradient_Obstacle ()
        {
            //static returnval;
            //static c_point = obst_closest_point;
            //static dist = getLinearDistance (q_robot, obst_closest_point);
            
            //returnval = (q_robot.x - c_point.x, q_robot.y - c_point.y)/dist; 

            //return returnval;
        }

};


class Controller { 
public: 

	pose_xy Oi;
	bool last_dir;
	double last_dist;

    Controller(){
    	last_dist = 0;
    } 

    double getAngularDistance(StageBot& robot, StageBot& goal) 
    { 
        return atan2(goal.pose.y - robot.pose.y, \
                     goal.pose.x - robot.pose.x) \
                   -robot.pose.heading; 
    } 

    double getLinearDistance(StageBot& robot, StageBot& goal) 
    { 
        return sqrt(pow(goal.pose.x - robot.pose.x, 2) \
                    + pow(goal.pose.y - robot.pose.y, 2)); 
    }

    double getLinearDistance(pose_xy p1, pose_xy p2) 
    { 
        return pose_xy::getLinearDistance(p1, p2);
    }

    static pose_xy getRobotGoalVector(pose_xy p1, pose_xy p2)
    {
    	pose_xy s;
    	s.x = p1.x - p2.x;
    	s.y = p1.y - p2.y;
    	s.heading  = atan2(s.y, s.x);

    	return s;
    }

}; 



int main(int argc, char **argv) 
{ 
    
    ros::init(argc, argv, "controller"); 
    ros::NodeHandle n("controller"); 
    StageBot robot(n, 0, 0.1, 1000, 0.05); 
    StageBot goal(n, 1); 
    Controller controller; 

    // ros::init(argc, argv, "controller"); 
    // ros::NodeHandle nh; 
    // // Create a client object for the spawn service . This
    // // needs to know the data type of the service and its
    // // name.
    // ros::ServiceClient mapClient  = nh.serviceClient <nav_msgs::GetMap>("potential.map");
    // // Create the request and response objects.
    // nav_msgs::GetMap::Request req_map;
    // nav_msgs::GetMap::Response resp_map;
    // // Fill in the request data members .
    // // req.x = 2;
    // // req.y = 3;
    // // req.theta = M_PI / 2 ;
    // // req.name = " Leo " ;
    // // Actually call the service . This won't return until
    // // the service is complete .
    // bool success = mapClient.call (req_map, resp_map);
    // // Check for success and use the response.
    // if ( success ) 
    // {
    // ROS_INFO_STREAM( "Map read");
    // } 
    // else
    // {
    // ROS_ERROR_STREAM( "Failed to read map." );
    // }

    double K = 1; 
    pose_xy Oi, goalRobotV;
    double dist;
    double last_dist=0;

    double dReached, dFollow;
    pose_xy dFollowp;

    // potential U_att;
    // potential U_rep;
    // U_att = Attractive_Potential ():
    // U_rep = Repulsive_Potential ();
    // U_tot = U_att + U_rep;
    // grad_U_att = Gradient_Attractive ();
    // grad_U_rep = Gradient_Repulsive ();
    // double K_gain;

    ros::Rate rate(20); 

    ros::Duration(0.5).sleep();

    
    while (ros::ok()) 
    { 
    	rate.sleep();
    	ros::spinOnce();
		


        
    	if (pose_xy::getLinearDistance(goal.base, robot.base) < 0.1)
    	{
    		ROS_INFO("GOAL REACHED!");
    		printf("goal->  x: %1.2f \t y: %1.2f\n robot-> x: %1.2f \t y: %1.2f\n", goal.base.x, goal.base.y, robot.base.x, robot.base.y);
        	robot.move(0, 0);

    		return EXIT_SUCCESS;
		}
		else
		{
	        if(robot.laser.obstacle_in_path(goal.base, robot.base))
	        {
	        	// Oi = robot.laser.findClosestTangentPoint(goal.base, robot.base);
	        }

	        dist = pose_xy::getLinearDistance(goal.base, Oi) + pose_xy::getLinearDistance(Oi, robot.base);
	        
	        //boundary following behavior!
	        if(dist > last_dist)
	    	{
	    		// dReached = pose_xy::getLinearDistance(goal.base, robot.base);
	    		// dFollowp = robot.laser.findClosestTangentPoint(goal.base, robot.base);
	    		// dFollow = pose_xy::getLinearDistance(dFollowp, goal.base);
	    		while(dReached >= dFollow)
	    		{
	    			rate.sleep();
	    			ros::spinOnce();
	    			

	    			goalRobotV = Controller::getRobotGoalVector(dFollow, robot.base);
	        		// robot.move(goalRobotV.x, goalRobotV.y);
                    robot.move(0.1, -0.1);
                    ROS_INFO("Locked!");
	    		}
	    	}
	        else
	        {
	        	goalRobotV = Controller::getRobotGoalVector(goal.base, robot.base);
	        	robot.move(goalRobotV.x, goalRobotV.y);
                ROS_INFO("Else!");
	        }
        }
        
	        ros::spinOnce();
    	    dist = pose_xy::getLinearDistance(goal.base, Oi) + pose_xy::getLinearDistance(Oi, robot.base);
	        last_dist = dist;
    } 
    
    return EXIT_SUCCESS; 
}
