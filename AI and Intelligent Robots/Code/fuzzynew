#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <fstream>
#include <time.h>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <chrono>
using namespace std;
using namespace std::chrono;

struct EulerAngles{
    double roll, pitch, yaw;};          // yaw is what you want, i.e. Th

struct Quaternion{double w, x, y, z;};

EulerAngles ToEulerAngles(Quaternion q){
    EulerAngles angles;
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        angles.pitch = copysign(M_PI/2, sinp);  //use 90 degrees if out of range
    else
        angles.pitch = asin(sinp);
    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    angles.yaw = atan2(siny_cosp, cosy_cosp);
    
    return angles;
}

class RobotMove { //main class
    public:
        // Tunable parameters
        constexpr const static double FORWARD_SPEED_LOW = 0.1;
        constexpr const static double FORWARD_SPEED_MIDDLE = 0.3;
        constexpr const static double FORWARD_SPEED_HIGH = 0.5;
        constexpr const static double FORWARD_SPEED_STOP = 0;
        constexpr const static double TURN_LEFT_SPEED_HIGH = 1.0;
        constexpr const static double TURN_LEFT_SPEED_MIDDLE = 0.8;
        constexpr const static double TURN_LEFT_SPEED_LOW = 0.6;
        constexpr const static double TURN_RIGHT_SPEED_HIGH = -1.0;
        constexpr const static double TURN_RIGHT_SPEED_LOW = -0.3;
        constexpr const static double TURN_RIGHT_SPEED_MIDDLE = -0.6;
        constexpr const static double TURN_SPEED_ZERO = 0;

        RobotMove();
        void startMoving();
        void moveForward(double forwardSpeed);
        void moveStop();
        void moveRight(double turn_right_speed = TURN_RIGHT_SPEED_HIGH);
        void moveForwardRight(double forwardSpeed, double turn_right_speed);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void transformMapPoint(ofstream& fp, double laserRange, double laserTh);
        void Fuzzy_wallFollowing(double Data1,double Data2);
        void Fuzzy_to1stGap(double laserData1, double laserData2);
        void Fuzzy_toTrajectory(double laserData1, double laserData2);
        void Fuzzy_to2stGap(double laserData1, double laserData2);
        void Fuzzy_toEnd(double laserData1, double laserData2);
    private:
        ros::Time current_time;
        ros::Duration real_time;
        ros::NodeHandle node;
        ros::Publisher commandPub;          // Publisher to the robot's velocity command topic
        ros::Subscriber odomSub;            //Subscriber to robot’s odometry topic
        ros::Subscriber laserSub;           // Subscriber to robot’s laser topic

        Quaternion robotQuat;
        EulerAngles robotAngles;
        double robotHeadAngle;

        //  parameter for landmark and robot position
        double PositionX = 0.3, PositionY = 0.3, landmark1 = 1.15, landmark2 = 0.9;
        double homeX = 0.3, homeY = 0.3;
        double landmark3 = 1.4, landmark4 = 1.88, landmark5 = 0.35;
        double robVelocity = 0;
        //  parameter for laser
        double frontRange=0,mleftRange=0,leftRange=0,rightRange=0,mrightRange=0;
        double backRange=0, backleftRange=0, backrightRange=0, laserData[36];
        double frontAngle=0, mleftAngle=M_PI/4, leftAngle=M_PI/2;
        double rightAngle=-M_PI/2, mrightAngle=-M_PI/4;
        double backAngle=M_PI, backleftAngle=3*M_PI/4, backrightAngle=-3*M_PI/4;

        double Max_Fuzzy_output = 0.6;
};

void RobotMove::transformMapPoint(ofstream& fp, double laserRange, double laserTh){
    double localX, localY, globalX, globalY;
    localX = laserRange * cos(laserTh);
    localY = laserRange * sin(laserTh);
    globalX =(localX*cos(robotHeadAngle)-localY*sin(robotHeadAngle))+ PositionX;
    globalY = (localX*sin(robotHeadAngle)+localY*cos(robotHeadAngle))+ PositionY;
    if (globalX < 0) globalX = 0; else if (globalX > 2.5) globalX = 2.5;
    if (globalY < 0) globalY = 0; else if (globalY > 2.5) globalY = 2.5;
    fp << globalX << " " << globalY << endl;
}

RobotMove::RobotMove(){
    //Advertise a new publisher for the simulated robot's velocity command topic at 10Hz
    commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel",  10);
    // subscribe to the odom topic
    odomSub = node.subscribe("odom",  20,  &RobotMove::odomCallback,  this);
    laserSub = node.subscribe("scan", 1, &RobotMove::scanCallback, this);
}

//send a velocity command
void RobotMove::moveForward(double forwardSpeed){
    geometry_msgs::Twist msg;           //The default constructor to set all commands to 0
    msg.linear.x = forwardSpeed;            //Drive forward at a given speed along the x-axis.
    commandPub.publish(msg);
}

void RobotMove::moveStop(){
    geometry_msgs::Twist msg;
    msg.linear.x = FORWARD_SPEED_STOP;
    commandPub.publish(msg);
}

void RobotMove::moveRight(double turn_right_speed){
    geometry_msgs::Twist msg;
    commandPub.publish(msg);
}

void RobotMove::moveForwardRight(double forwardSpeed,  double turn_right_speed){
    //move forward and right at the same time
    geometry_msgs::Twist msg;
    msg.linear.x = forwardSpeed;
    msg.angular.z = turn_right_speed;
    commandPub.publish(msg);
}

// add the callback function to determine the robot position.
void RobotMove::odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg){
    PositionX = odomMsg->pose.pose.position.x + homeX;
    PositionY = odomMsg->pose.pose.position.y + homeY;
    robVelocity = odomMsg->twist.twist.linear.x;
    robotAngles = ToEulerAngles(robotQuat);
    robotHeadAngle = robotAngles.yaw;
    robotQuat.x = odomMsg->pose.pose.orientation.x;
    robotQuat.y = odomMsg->pose.pose.orientation.y;
    robotQuat.z = odomMsg->pose.pose.orientation.z;
    robotQuat.w = odomMsg->pose.pose.orientation.w;
}

void RobotMove::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    // collect 36 laser readings every 360 degrees scan
    for(int i=0; i<36; i++) // to get 36 laser readings over 360 degrees
        laserData[i] = scan->ranges[i*10]; // to get laser readings every 10 degrees
    // the following code for the control purpose
    frontRange = scan->ranges[0]; // get the range reading at 0 radians
    mleftRange = scan->ranges[89]; // get the range reading at -π/4 radians
    leftRange = scan->ranges[179]; // get the range reading at -π/2 radians
    rightRange = scan->ranges[539]; // get the range reading at π/2 radians
    mrightRange = scan->ranges[629]; // get the range reading at π/4 radians
    backRange = scan->ranges[359]; // get the range reading at π radians
    backleftRange = scan->ranges[269]; // get the range reading at π/2 radians
    backrightRange = scan->ranges[449]; // get the range reading at π/4 radians
}

ofstream openFile(const string& name){          // open files for data storage
    string homedir = getenv("HOME");
    ostringstream path;
    ofstream file;

    //  change the path here
    //  ex:  if your pkg path is /home/wang/Ros/workspace/ai_course/src/tutorial_pkg/
    //          hrer only need /ROS/workspace/ai_course/src/tutorial_pkg/
    path << homedir <<  "/ros_workspace/src/tutorial_pkg/" << name;
    ROS_INFO("File address: %s", path.str().c_str());
    file.open(path.str().c_str());
    if (file.is_open())     ROS_INFO("File opened");
    
    return file;
}

//  Fuzzy controller for moving along the wall
void RobotMove::Fuzzy_wallFollowing(double laserData1, double laserData2)
{
    int fuzzySensor1, fuzzySensor2;
    // sensor data fuzzification
    if (laserData1 < 0.2)   fuzzySensor1 = 1;       // The robot is near to the wall
    else if (laserData1 < 0.5)  fuzzySensor1 = 2;       // The robot is on the right distance
    else    fuzzySensor1 = 3;       // The robot is far from the wall;

    if (laserData2 < 0.3)   fuzzySensor2 = 1;   // The robot is near to the wall
    else if (laserData2 < 0.6)  fuzzySensor2 = 2;   // The robot at the right distance;
    else    fuzzySensor2 = 3;   // The robot is far from the wall;

    // Fuzzy rule base and control output
    if (fuzzySensor1 == 1 && fuzzySensor2 == 1)
        moveForwardRight(FORWARD_SPEED_LOW, TURN_RIGHT_SPEED_LOW);
    else if(fuzzySensor1 == 1 && fuzzySensor2 == 2)
        moveForwardRight(FORWARD_SPEED_LOW, TURN_RIGHT_SPEED_LOW);
    else if(fuzzySensor1 == 1 && fuzzySensor2 == 3)
        moveForwardRight(FORWARD_SPEED_LOW, TURN_LEFT_SPEED_LOW);
    else if(fuzzySensor1 == 2 && fuzzySensor2 == 1)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_LOW);
    else if(fuzzySensor1 == 2 && fuzzySensor2 == 2)
        moveForwardRight(FORWARD_SPEED_HIGH, TURN_SPEED_ZERO);
    else if(fuzzySensor1 == 2 && fuzzySensor2 == 3)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_LEFT_SPEED_LOW);
    else if(fuzzySensor1 == 3 && fuzzySensor2 == 1)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
    else if(fuzzySensor1 == 3 && fuzzySensor2 == 2)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
    else if(fuzzySensor1 == 3 && fuzzySensor2 == 3)
        moveForwardRight(FORWARD_SPEED_HIGH, TURN_LEFT_SPEED_LOW);
    else ROS_INFO("Following the left wall");
}

//  Fuzzy controller for going through the gap 1
void RobotMove::Fuzzy_to1stGap(double laserData1, double laserData2)
{
    int fuzzySensor1, fuzzySensor2;
    // sensor data fuzzification
    if (0 <= laserData1 < 0.2)  fuzzySensor1 = 1;
    else if (laserData1 < 0.5)  fuzzySensor1 = 2;
    else    fuzzySensor1 = 3;

    if (laserData2 < 0.2)   fuzzySensor2 = 1;
    else if (laserData2 < 0.9)  fuzzySensor2 = 2;
    else    fuzzySensor2 = 3;

    // Fuzzy rule base and control output
    if(fuzzySensor1 == 1 && fuzzySensor2 == 1)
        moveForwardRight(FORWARD_SPEED_LOW, TURN_RIGHT_SPEED_LOW);
    else if(fuzzySensor1 == 1 && fuzzySensor2 == 2)
        moveForwardRight(FORWARD_SPEED_LOW, TURN_RIGHT_SPEED_LOW);
    else if(fuzzySensor1 == 1 && fuzzySensor2 == 3)
        moveForwardRight(FORWARD_SPEED_LOW, TURN_LEFT_SPEED_LOW);
    else if(fuzzySensor1 = 2 && fuzzySensor2 == 1)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
    else if(fuzzySensor1 = 2 && fuzzySensor2 == 2)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
    else if(fuzzySensor1 = 2 && fuzzySensor2 == 3)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_LOW);
    else if(fuzzySensor1 = 3 && fuzzySensor2 == 1)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_LEFT_SPEED_HIGH);
    else if(fuzzySensor1 = 3 && fuzzySensor2 == 2)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_LEFT_SPEED_MIDDLE);
    else if(fuzzySensor1 = 3 && fuzzySensor2 == 3)
        moveForwardRight(FORWARD_SPEED_HIGH, TURN_RIGHT_SPEED_LOW);
    else ROS_INFO("Going through the 1st gap");
}

//  Fuzzy controller for going through the Relay point
void RobotMove::Fuzzy_toTrajectory(double laserData1, double laserData2)
{
    int fuzzySensor1, fuzzySensor2;
    // sensor data fuzzification
    if (0 <= laserData1 < 0.7)  fuzzySensor1 = 1;
    else if (laserData1 < 1)  fuzzySensor1 = 2;
    else    fuzzySensor1 = 3;

    if (laserData2 < 0.7)   fuzzySensor2 = 1;
    else if (laserData2 < 1)  fuzzySensor2 = 2;
    else    fuzzySensor2 = 3;

    // Fuzzy rule base and control output
    if(fuzzySensor1 == 1 && fuzzySensor2 == 1)
        moveForwardRight(FORWARD_SPEED_LOW, TURN_RIGHT_SPEED_LOW);
    else if(fuzzySensor1 == 1 && fuzzySensor2 == 2)
        moveForwardRight(FORWARD_SPEED_LOW, TURN_RIGHT_SPEED_LOW);
    else if(fuzzySensor1 == 1 && fuzzySensor2 == 3)
        moveForwardRight(FORWARD_SPEED_LOW, TURN_LEFT_SPEED_LOW);
    else if(fuzzySensor1 = 2 && fuzzySensor2 == 1)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
    else if(fuzzySensor1 = 2 && fuzzySensor2 == 2)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
    else if(fuzzySensor1 = 2 && fuzzySensor2 == 3)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_LOW);
    else if(fuzzySensor1 = 3 && fuzzySensor2 == 1)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_LEFT_SPEED_HIGH);
    else if(fuzzySensor1 = 3 && fuzzySensor2 == 2)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_LEFT_SPEED_MIDDLE);
    else if(fuzzySensor1 = 3 && fuzzySensor2 == 3)
        moveForwardRight(FORWARD_SPEED_HIGH, TURN_RIGHT_SPEED_LOW);
    else ROS_INFO("Going through the Trajectory");
}

//  Fuzzy controller for going through the gap 2
void RobotMove::Fuzzy_to2stGap(double laserData1, double laserData2)
{
    int fuzzySensor1, fuzzySensor2;
    // sensor data fuzzification
    if (0 <= laserData1 < 0.52)  fuzzySensor1 = 1;
    else if (laserData1 < 0.82)  fuzzySensor1 = 2;
    else    fuzzySensor1 = 3;

    if (laserData2 < 0.5)   fuzzySensor2 = 1;
    else if (laserData2 < 0.7)  fuzzySensor2 = 2;
    else    fuzzySensor2 = 3;

    // Fuzzy rule base and control output
    if(fuzzySensor1 == 1 && fuzzySensor2 == 1)
        moveForwardRight(FORWARD_SPEED_LOW, TURN_RIGHT_SPEED_LOW);
    else if(fuzzySensor1 == 1 && fuzzySensor2 == 2)
        moveForwardRight(FORWARD_SPEED_LOW, TURN_RIGHT_SPEED_LOW);
    else if(fuzzySensor1 == 1 && fuzzySensor2 == 3)
        moveForwardRight(FORWARD_SPEED_LOW, TURN_LEFT_SPEED_LOW);
    else if(fuzzySensor1 = 2 && fuzzySensor2 == 1)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
    else if(fuzzySensor1 = 2 && fuzzySensor2 == 2)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
    else if(fuzzySensor1 = 2 && fuzzySensor2 == 3)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_LOW);
    else if(fuzzySensor1 = 3 && fuzzySensor2 == 1)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_LEFT_SPEED_HIGH);
    else if(fuzzySensor1 = 3 && fuzzySensor2 == 2)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_LEFT_SPEED_MIDDLE);
    else if(fuzzySensor1 = 3 && fuzzySensor2 == 3)
        moveForwardRight(FORWARD_SPEED_HIGH, TURN_RIGHT_SPEED_LOW);
    else ROS_INFO("Going through the 2st gap");
}

//  Fuzzy controller for going to the end point
void RobotMove::Fuzzy_toEnd(double laserData1, double laserData2)
{
    int fuzzySensor1, fuzzySensor2;
    // sensor data fuzzification
    if (0 <= laserData1 < 0.38)  fuzzySensor1 = 1;
    else if (laserData1 < 0.52)  fuzzySensor1 = 2;
    else    fuzzySensor1 = 3;

    if (laserData2 < 0.38)   fuzzySensor2 = 1;
    else if (laserData2 < 0.5)  fuzzySensor2 = 2;
    else    fuzzySensor2 = 3;

    // Fuzzy rule base and control output
    if (fuzzySensor1 == 1 && fuzzySensor2 == 1)
        moveForwardRight(FORWARD_SPEED_LOW, TURN_RIGHT_SPEED_LOW);
    else if(fuzzySensor1 == 1 && fuzzySensor2 == 2)
        moveForwardRight(FORWARD_SPEED_LOW, TURN_RIGHT_SPEED_LOW);
    else if(fuzzySensor1 == 1 && fuzzySensor2 == 3)
        moveForwardRight(FORWARD_SPEED_LOW, TURN_LEFT_SPEED_LOW);
    else if(fuzzySensor1 == 2 && fuzzySensor2 == 1)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_LOW);
    else if(fuzzySensor1 == 2 && fuzzySensor2 == 2)
        moveForwardRight(FORWARD_SPEED_HIGH, TURN_SPEED_ZERO);
    else if(fuzzySensor1 == 2 && fuzzySensor2 == 3)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_LEFT_SPEED_LOW);
    else if(fuzzySensor1 == 3 && fuzzySensor2 == 1)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
    else if(fuzzySensor1 == 3 && fuzzySensor2 == 2)
        moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
    else if(fuzzySensor1 == 3 && fuzzySensor2 == 3)
        moveForwardRight(FORWARD_SPEED_HIGH, TURN_LEFT_SPEED_LOW);
    else ROS_INFO("Going through the End");
}

// add the following function
void RobotMove::startMoving(){
    auto currentTime = high_resolution_clock::now();
    auto startTime = currentTime;
    ofstream odomVelFile = openFile("fuzzyVelData.csv");
    ofstream odomTrajFile = openFile("fuzzyTrajData.csv");
    ofstream laserFile = openFile("fuzzylaserData.csv");
    ofstream laserMapFile = openFile("fuzzylaserMapData.csv");

    int stage = 1;
    ros::Rate rate(20);         //Define rate for repeatable operations.
    ROS_INFO("Start moving");
    while (ros::ok()){          // keep spinning loop until user presses Ctrl+C
        switch(stage){
            case 1:         // the robot move forward from home
                if (PositionY < landmark1)
                    Fuzzy_wallFollowing(leftRange, mleftRange);
                else stage = 2;
                break;
            case 2:         // the robot turns right toward the 1st gap
                if (PositionX < landmark2)
                    Fuzzy_to1stGap(leftRange, mleftRange);
                else stage = 3;
                break;
            case 3:         // the robot moves forward fast
                if (PositionX < landmark3)
                    Fuzzy_toTrajectory(leftRange, mleftRange);
                else stage = 4;
                break;
            case 4:         // the robot moves and turns right slowly
                if (PositionX < landmark4)
                    Fuzzy_to2stGap(leftRange, mleftRange);
                else stage = 5;
                break;
            case 5:         // the robot moves towards the charger
                if (PositionY > landmark5)
                    Fuzzy_toEnd(leftRange, mleftRange);
                else stage = 6;
                break;
            case 6:         // stop at the charger position
                moveStop();
                stage = 7;  //  used to exit the while
                break;
        }
        auto currentTime = high_resolution_clock::now();
        duration<double,std::deca> runTime = currentTime  - startTime;
        runTime = runTime * 10;             // convert time to seconds
        odomVelFile << ceil(runTime.count()) << " " << robVelocity << endl;
        odomTrajFile << PositionX << " " << PositionY << endl;

        for(int i=0; i<36; i++) // save laser data for view and check
            laserFile << i << " "<< laserData[i]<<" ";
        laserFile << endl;

        transformMapPoint(laserMapFile, frontRange, frontAngle);
        transformMapPoint(laserMapFile, leftRange, leftAngle);
        transformMapPoint(laserMapFile, rightRange, rightAngle);
        transformMapPoint(laserMapFile, mleftRange, mleftAngle);
        transformMapPoint(laserMapFile, mrightRange, mrightAngle);

        ros::spinOnce();                // Allow ROS to process incoming messages
        rate.sleep();               // Wait until defined time passes.
        ROS_INFO("\nCurrent stage:%d, Robot at [X: %0.2f, Y: %0.2f, Theta: %0.2f], Speed: %0.2f", stage, PositionX, PositionY, robotHeadAngle, robVelocity);
        if (stage == 7)     
            break;
    }
    //  save all files
    odomTrajFile.close();
    odomVelFile.close();
    laserFile.close();
    laserMapFile.close();
    ROS_INFO("Finish ALL");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "RobotMove");         //Initiate new ROS node named "RobotMove"
    RobotMove RobotMove;            // Create new RobotMove object
    RobotMove.startMoving();            // Start the movement

    return 0;
}
