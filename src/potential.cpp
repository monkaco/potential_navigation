#include <potential/robot.h>
//#include <potential/potential.h>
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
    public:
    sensor_msgs::LaserScan laser;
    Laser() {}

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
    { 
        laser = *msg;
    } 
    
    pose_xy getMinimumDistanceRobotToPoint (pose_xy dummy_pose, pose_xy robot_pose)
    {
        float minimum = laser.range_max;
        int min_index = 0;
        pose_xy point;

        printf("range: %1.2f \t laser.range.size: %2d\n", -1.23,laser.ranges.size());
        for(int i = 0; i < laser.ranges.size(); i++)
        {
            ROS_INFO("Dentro do for");
            if (laser.ranges[i] > laser.range_min && laser.ranges[i] < minimum)
            {
                minimum = laser.ranges[i];
                min_index = i;
                // printf("range: %1.2f \t min_index: %2d\n", minimum,min_index);
                ROS_INFO("Dentro do IF");
            }
        }

        double ang_point = robot_pose.heading - (laser.angle_increment * min_index + laser.angle_min);

        // point.x = robot_pose.x + minimum*cos(ang_point);
        // point.y = robot_pose.y + minimum*sin(ang_point);
        // ROS_INFO("Laser method");
        point.x = minimum*cos(ang_point);
        point.y = minimum*sin(ang_point);
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
    

};


class StageBot { 
public: 
    Base base; 
    Pose pose; 
    Laser laser;
    double d; 
    double kv;
    double kw;

    StageBot(ros::NodeHandle& nh, int robotID, double dist=0.1, double kvel = 0.5, double kwp = 0.5): ID(robotID) 
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
        pose_xy& robo;
        pose_xy& alvo;
        double transition_threshold;
        double obstacle_range_threshold;
        double k_gain;

        // Potential(){
        // transition_threshold = 0;
        // } 

        /* Methods */

        static double getAngularDistance (pose_xy robo, pose_xy alvo) 
        { 
            return atan2(alvo.y - robo.y, \
                        alvo.x - robo.x) \
                        -robo.heading; 
        } 

        static double getLinearDistance (pose_xy robo, pose_xy alvo) 
        { 
            return sqrt(pow(alvo.x - robo.x, 2) \
                    + pow(alvo.y - robo.y, 2)); 
        }

        /* ATTRACTIVE POTENTIAL */
        static bool Conic_Quadratic_transition (pose_xy robo, pose_xy alvo, double trans_threshold) 
        {
            bool returnval;
            double dist;

            dist = getLinearDistance (robo, alvo);
            if (dist <= trans_threshold)
            {
                returnval = true;  /* Quadratic */
            }
            else
            {
                returnval = false;   /* Conic */
            }
                return returnval;
        }

        static double Attractive_Potential (pose_xy robo, pose_xy alvo, double trans_threshold) 
        {
            double returnval;
            double d_goal_star;
            double dist;
            double k_gain = 1.0;

            d_goal_star = trans_threshold;
            dist = getLinearDistance (robo, alvo) ;

            if (Conic_Quadratic_transition (robo, alvo, trans_threshold))  /* true -> quadratic function */
            {
                returnval = (1/2) * k_gain * pow (dist, 2);
            }
            else                                /* false -> conic function */
            {
                returnval = d_goal_star * k_gain * dist - (1/2) * k_gain * pow (d_goal_star, 2);
            }

            return returnval;
        }

        static pose_xy Gradient_Attractive (pose_xy robo, pose_xy alvo, double trans_threshold) 
        {
            pose_xy returnval;
            double dist;
            double d_goal_star = trans_threshold;
            double k_gain = 1.0;
            
            dist = getLinearDistance (robo, alvo);

            if (Conic_Quadratic_transition (robo, alvo, trans_threshold))  /* true -> quadratic function */
            {
                returnval.y = k_gain * (robo.x - alvo.x);
                returnval.x = k_gain * (robo.y - alvo.y);
                ROS_INFO("Quadratico...");
            }
            else                                /* false -> conic function */
            {
                returnval.y = d_goal_star * k_gain * (robo.x - alvo.x)/dist;
                returnval.x = d_goal_star * k_gain * (robo.y - alvo.y)/dist;
                ROS_INFO("Conico...");
            }

            return returnval;
        }

        static pose_xy Force_Attractive (pose_xy grad_Attractive) 
        {
            pose_xy returnval;
            
            returnval.x = (-1) * grad_Attractive.x;
            returnval.y = (-1) * grad_Attractive.y;

            return returnval;
        }

        /* REPULSIVE POTENTIAL */
        static bool Obstacle_within_range (pose_xy robo, pose_xy alvo, double obs_threshold)
        {
            bool returnval;
            double dist;

            dist = getLinearDistance (robo, alvo);
            if (dist <= obs_threshold)
            {
                returnval = true;  /* Obstacle within range */
            }
            else
            {
                returnval = false;   /* Obstacle out of range */
            }
            
            return returnval;
        }

        static double Repulsive_Potential (pose_xy robo, pose_xy alvo, double obs_threshold)
        {
            double returnval;
            double dist;
            double eta = 1.0; /* Repulsive gain */
            double d_obs;   /* Distance obtained from the map (algorithm brushfire) */
            double Q_star = obs_threshold;

            /* Treat this situation */
            dist = getLinearDistance (1/d_obs, 1/Q_star);

            if (Obstacle_within_range (robo, alvo, obs_threshold))   /* true -> within range */
            {
                returnval = (1/2) * eta * pow( dist, 2);
            }
            else                            /* false -> out of range */
            {
                returnval = 0.0;
            }

            return returnval;
         }

        static pose_xy Gradient_Repulsive (pose_xy robo, pose_xy obs, double obs_threshold)
        {
            pose_xy returnval;
            double Q_star;
            double dist;
            double eta = 0.5;

            // dist = getLinearDistance (1/d_obs, 1/Q_star);
            Q_star = obs_threshold;
            // dist = sqrt(pow(1/obs.x, 2) + pow(1/obs.y, 2)) - 1/Q_star; 
            dist = sqrt(pow(obs.x, 2) + pow(obs.y, 2)) - Q_star; 

            if (Obstacle_within_range (robo, obs, obs_threshold))  /* true -> within range */
            {
               returnval.x = 1/(eta * dist * sqrt( 1/pow(obs.x, 2) + 1/pow(robo.x, 2) ) * (robo.x - obs.x)/sqrt( 1/pow(robo.x, 2) + 1/pow(obs.x, 2) ));
               returnval.y = 1/(eta * dist * sqrt( 1/pow(obs.y, 2) + 1/pow(robo.y, 2) ) * (robo.y - obs.y)/sqrt( 1/pow(robo.y, 2) + 1/pow(obs.y, 2) ));
            }
            else                                 /* false -> conic function */
            {
                returnval.x = 0.0;
                returnval.y = 0.0;
            }

            return returnval;
        }
            
        static pose_xy Force_Repulsive (pose_xy grad_Repulsive) 
        {
            pose_xy returnval;
            
            returnval.x = (1) * grad_Repulsive.x;
            returnval.y = (1) * grad_Repulsive.y;

            return returnval;
        }

        static pose_xy Gradient_Obstacle ()
        {
            //static returnval;
            //static c_point = obst_closest_point;
            //static dist = getLinearDistance (q_robot, obst_closest_point);
            
            //returnval = (q_robot.x - c_point.x, q_robot.y - c_point.y)/dist; 

            //return returnval;
        }

        static pose_xy Force_Total (pose_xy grad_Attractive, pose_xy grad_Repulsive) 
        {
            pose_xy returnval;
            
            returnval.x = grad_Attractive.x + grad_Repulsive.x;
            returnval.y = grad_Attractive.y + grad_Repulsive.y;

            if (returnval.x > 5.0 || returnval.x < -5.0)
            {
                returnval.x = 5.0;
            }
            else if (returnval.y > 5.0 || returnval.y < -5.0)
            {
                returnval.y = 5.0;
            }

            return returnval;
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

    double K = 0.2; 
    pose_xy Oi, goalRobotV;
    double dist;
    double last_dist=0;

    double dReached, dFollow;
    pose_xy dFollowp;
    pose_xy robo;
    pose_xy alvo;

    Potential pot();
    Laser laser1;
    double transition_threshold = 0.8;
    double obstacle_range_threshold = 0.3;
    double U_att;
    double U_rep;
    double U_tot;
    bool res;
    pose_xy grad_U_att;
    pose_xy grad_U_rep;
    pose_xy force_att;
    pose_xy force_rep;
    pose_xy force_tot;
    pose_xy laser_dist;
    float test;

    double minimum = 4.0;
    double min_aux;
    int min_index = 0;
    double ang_point;
    pose_xy nearest_obs;

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
            robot.laser;
            test = robot.laser.laser.ranges[0];


            U_att = Potential::Attractive_Potential (robot.base, goal.base, transition_threshold);
            U_rep = Potential::Repulsive_Potential (robot.base, goal.base, obstacle_range_threshold);
            U_tot = U_att + U_rep;

            grad_U_att = Potential::Gradient_Attractive (robot.base, goal.base, transition_threshold);
            force_att = Potential::Force_Attractive (grad_U_att);

            minimum = robot.laser.laser.range_max;
            for(int i = 0; i < robot.laser.laser.ranges.size(); i++)
            {   
                // min_aux = robot.laser.laser.ranges[i];
                // robot.laser.laser.range_min
                // if ((min_aux > 0.1) && (min_aux < minimum))
                if ((robot.laser.laser.ranges[i] > 0.1) && (robot.laser.laser.ranges[i] < minimum))
                {
                    minimum = robot.laser.laser.ranges[i];
                    min_index = i;
                    // ROS_INFO("### if ###");
                }
                // printf("minimum: %1.2f\t range: %1.2f \n", minimum, robot.laser.laser.ranges[i]);
                // printf("range: %1.2f \n", robot.laser.laser.ranges[i]);
            }
            
            
            // ang_point = robot.base.heading - (robot.laser.laser.angle_increment * min_index + robot.laser.laser.angle_min);
            ang_point = robot.laser.laser.angle_increment * min_index - (robot.laser.laser.angle_max - robot.base.heading);

            nearest_obs.x = robot.base.x + minimum*cos(ang_point);
            nearest_obs.y = robot.base.y + minimum*sin(ang_point);
            nearest_obs.heading = ang_point;

            grad_U_rep = Potential::Gradient_Repulsive (robot.base, nearest_obs, obstacle_range_threshold);
            force_rep = Potential::Force_Repulsive(grad_U_rep);

            force_tot = Potential::Force_Total (force_att, force_rep);

             // robot.move(0,0);
            robot.move(force_tot.x, force_tot.y);
            printf("f_att.x: %1.2f\t f_att.y: %1.2f\n f_rep.x: %1.2f\t f_rep.y: %1.2f \n f_tot.x: %1.2f\t f_tot.y: %1.2f \n", force_att.x, force_att.y,force_rep.x, force_rep.y,force_tot.x, force_tot.y);
            printf("nearest_obs.x: %1.2f\t nearest_obs.y: %1.2f\t nearest_angle: %1.2f\t min_index: %d",  nearest_obs.x, nearest_obs.y, ang_point, min_index);
            // printf("robot->  x: %1.2f \t y: %1.2f\n laser-> x: %1.2f \t y: %1.2f\n nearest_obs.x: %1.2f\t nearest_obs.y: %1.2f\t min.laser.ranges: %1.2f\n", robot.base.x, robot.base.y, robot.base.x, robot.base.y, nearest_obs.x, nearest_obs.y, minimum);
           

        }
        
	        ros::spinOnce();
    	    dist = pose_xy::getLinearDistance(goal.base, Oi) + pose_xy::getLinearDistance(Oi, robot.base);
	        last_dist = dist;
    } 
    
    return EXIT_SUCCESS; 
}
