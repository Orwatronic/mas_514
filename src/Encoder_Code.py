#include <ros.h>
#include <std_msgs/Int16.h>
 
// Handles startup and shutdown of ROS
ros::NodeHandle nh;

// Variabels
static volatile int angle_left;
static volatile int angle_right;
static float thetaPre_left;
static float thetaPre_right;
static unsigned int tHigh_left, tLow_left;
static unsigned long rise_left, fall_left;
static unsigned int tHigh_right, tLow_right;
static unsigned long rise_right, fall_right;
static int turns_left;
static int turns_right;

std_msgs::Int16 angleRight;
ros::Publisher rightPub("angle_right_wheel", &angleRight);
 
std_msgs::Int16 angleLeft;
ros::Publisher leftPub("angle_left_wheel", &angleLeft);

// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;

// Parameters
static const int unitsFC = 360;
static const float dcMin = 0.029;
static const float dcMax = 0.971;
static const int dutyScale = 1;
static const int q2min = unitsFC / 4;
static const int q3max = q2min * 3;


 // Function for reading left wheel angle and number of turns
void feedback_left() {

 if (digitalRead(2)) {
    rise_left = micros();
    tLow_left = rise_left - fall_left;

    int tCycle_left = tHigh_left + tLow_left;
    if ((tCycle_left < 1000) || (tCycle_left > 1200))
      return;

    float dc_left = (dutyScale * tHigh_left) / (float)tCycle_left;
    float theta_left = ((dc_left - dcMin) * unitsFC) / (dcMax - dcMin);

    if (theta_left < 0.0)
      theta_left = 0.0;
    else if (theta_left > (unitsFC - 1.0))
      theta_left = unitsFC - 1.0;

    if ((theta_left < q2min) && (thetaPre_left > q3max))
      turns_left++;
    else if ((thetaPre_left < q2min) && (theta_left > q3max))
      turns_left--;

    if (turns_left >= 0)
      angle_left = (turns_left * unitsFC) + theta_left;
    else if (turns_left < 0)
      angle_left = ((turns_left + 1) * unitsFC) - (unitsFC - theta_left);

    thetaPre_left = theta_left;
  } else {
    fall_left = micros();
    tHigh_left = fall_left - rise_left;
  }

    angleLeft.data = angle_left; 
}

 // Function for reading right wheel angle and number of turns
void feedback_right() {

 if (digitalRead(3)) {
    rise_right = micros();
    tLow_right = rise_right - fall_right;

    int tCycle_right = tHigh_right + tLow_right;
    if ((tCycle_right < 1000) || (tCycle_right > 1200))
      return;

    float dc_right = (dutyScale * tHigh_right) / (float)tCycle_right;
    float theta_right = ((dc_right - dcMin) * unitsFC) / (dcMax - dcMin);

    if (theta_right < 0.0)
      theta_right = 0.0;
    else if (theta_right > (unitsFC - 1.0))
      theta_right = unitsFC - 1.0;

    if ((theta_right < q2min) && (thetaPre_right > q3max))
      turns_right++;
    else if ((thetaPre_right < q2min) && (theta_right > q3max))
      turns_right--;

    if (turns_right >= 0)
      angle_right = (turns_right * unitsFC) + theta_right;
    else if (turns_right < 0)
      angle_right = ((turns_right + 1) * unitsFC) - (unitsFC - theta_right);

    thetaPre_right = theta_right;
  } else {
    fall_right = micros();
    tHigh_right = fall_right - rise_right;
  }

    angleRight.data = angle_right;
}

void setup() {
  // Interrupt pin #2
  attachInterrupt(0, feedback_left, CHANGE);  
  // Interrupt pin #3
  attachInterrupt(1, feedback_right, CHANGE);  

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
}

void loop() {
  // Record the time
  currentMillis = millis();
 
  // If 100ms have passed, print the number of ticks
  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;
    rightPub.publish( &angleRight );
    leftPub.publish( &angleLeft );
    nh.spinOnce();  
  }
}