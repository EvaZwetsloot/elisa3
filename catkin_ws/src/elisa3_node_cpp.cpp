#include <sstream>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "elisa3-lib.h"
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <opencv/cv.h>
#include <sensor_msgs/LaserScan.h>

#define DEBUG_ROS_PARAMS 0
#define DEBUG_UPDATE_SENSORS_DATA 0
#define DEBUG_ODOMETRY 0
#define DEBUG_ACCELEROMETER 0
#define DEBUG_SPEED_RECEIVED 0
#define DEBUG_RANGE_SENSORS 0

#define SENSORS_NUM 4
#define ACCELEROMETER 0
#define FLOOR 1
#define PROXIMITY 2
#define MOTOR_POSITION 3

#define ACTUATORS_NUM 4
#define MOTORS 0
#define GREEN_LEDS 1
#define RGB_LED 2
#define IR_TX 3

#define WHEEL_DISTANCE 0.041		// Distance between wheels in meters (axis length)
#define ROBOT_RADIUS 0.025			// meters.

int robotAddress[1];
bool enabledSensors[SENSORS_NUM];
bool changedActuators[ACTUATORS_NUM];
int speedLeft = 0, speedRight = 0;
unsigned char ledNum = 0, ledState = 0;
std::string elisa3Name;
struct timeval currentTime2, lastTime2;
struct timeval currentTime3, lastTime3;

signed int accData[3];
unsigned int floorData[4];
unsigned int proxData[8];
signed int robTheta=0, robXPos=0, robYPos=0;

ros::Publisher proxPublisher[8];
sensor_msgs::Range proxMsg[8];
ros::Publisher laserPublisher;
sensor_msgs::LaserScan laserMsg;
ros::Publisher odomPublisher;
nav_msgs::Odometry odomMsg;
ros::Publisher accelPublisher;
sensor_msgs::Imu accelMsg;
ros::Publisher floorPublisher;
visualization_msgs::Marker floorMsg;

ros::Subscriber cmdVelSubscriber;

int i = 0;
double xPos, yPos, theta;
double robXPosPrev, robYPosPrev, robThetaPrev, robDeltaX, robDeltaY, robDeltaTheta;
double deltaXCorr, deltaYCorr;
double xPosCorr, yPosCorr;
double robDistTraveled, robDistTraveledPrev, robDeltaDistTraveled;
ros::Time currentTime, lastTime, currentTimeMap, lastTimeMap;

#define OBSTACLE_THR 50

unsigned char updateRGB(char *red, char *green, char *blue) {
    static unsigned int i=0;
    unsigned int rndNum;

    i = (i+1)%65000;  // use to change the rgb leds
    if(i==0) {
        rndNum = rand()%400;
        if(rndNum < 100) {
            *red = rand()%100;
            *green = rand()%100;
            *blue = 0;
        } else if(rndNum < 200) {
            *red = rand()%100;
            *green = 0;
            *blue = rand()%100;
        } else if(rndNum < 300) {
            *red = 0;
            *green = rand()%100;
            *blue = rand()%100;
        } else {
            *red = rand()%100;
            *green = rand()%100;
            *blue = rand()%100;
        }
        return 1;
    }
    return 0;
}

void avoidObstacles(unsigned int *prox, char *left, char *right) {
    int rightProxSum=0, leftProxSum=0;	// sum of proximity values on right or left

    // obstacle avoidance using the 3 front proximity sensors
    rightProxSum = prox[0]/2 + prox[1];
    leftProxSum = prox[0]/2 + prox[7];

    rightProxSum /= 5;                 // scale the sum to have a moderate impact on the velocity
    leftProxSum /= 5;
    if(rightProxSum > 60) {             // velocity of the motors set to be from -30 to +30
        rightProxSum = 60;
    }
    if(leftProxSum > 60) {
        leftProxSum = 60;
    }
    *right = 30-leftProxSum;     // set the speed to the motors
    *left = 30-rightProxSum;

}

int main(int argc,char *argv[]) {
//    double init_xpos, init_ypos, init_theta;
//    unsigned char sensorsEnabled = 0;
    int NUM_robots = 8;
    int robotAddress[NUM_robots] = {3802, 3795, 3768, 3772, 3813, 3804, 3820, 3735};
    int i = 0;
    int k = 0;
    unsigned int robProx[NUM_ROBOTS][8];
    // sent to robot
    char robLSpeed[NUM_ROBOTS], robRSpeed[NUM_ROBOTS];
    char robRedLed, robGreenLed, robBlueLed;

    //cmdVelSubscriber = n.subscribe("mobile_base/cmd_vel", 10, handlerVelocity);

    startCommunication(robotAddress, NUM_robots);

    while (1) {

       getAllProximityFromAll(robProx);

        for(k=0; k<NUM_ROBOTS; k++) {
            avoidObstacles(robProx[k], &robLSpeed[k], &robRSpeed[k]);
        }
        setLeftSpeedForAll(robLSpeed);
        setRightSpeedForAll(robRSpeed);

        if(updateRGB(&robRedLed, &robGreenLed, &robBlueLed)) {
            for(k=0; k<NUM_ROBOTS; k++) {
                //setRed(robotAddress[k], robRedLed);
                setGreen(robotAddress[k], 50);
                //setBlue(robotAddress[k], robBlueLed);
            }
        }

        //ros::spinOnce();
//				if(waitForUpdate(robotAddress[0], 10000000)) { // Wait for at most 10 seconds.
//						break; // We have connection problems, stop here.
//				}
    }
		stopCommunication();
}



