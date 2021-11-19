/*
* ------------ MAS514 ---------------------------------------------------------------
* Lab Exercise 3 - Problem 1c - Solution
* Wheel Encoder and Arduino 
*/
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
// Parameters
static const int unitsFC = 360;
static const float dcMin = 0.029;
static const float dcMax = 0.971;
static const int dutyScale = 1;
static const int q2min = unitsFC / 4;
static const int q3max = q2min * 3;

void setup() {
  // serial communication start with 115200 bps
  Serial.begin(115200);
  // Interrupt pin #2
  attachInterrupt(0, feedback_left, CHANGE);  
  // Interrupt pin #3
  attachInterrupt(1, feedback_right, CHANGE);  
}

void loop() {
  // Print Wheel rotation and number of turns (left wheel is inverted)

  Serial.print("Now angle Left Wheel: ");
  Serial.println(-angle_left);

  Serial.print("Now angle Right Wheel: ");
  Serial.println(angle_right);

  Serial.print("Number of Turns Left Wheel: ");
  Serial.println(-turns_left);

  Serial.print("Number of Turns Right Wheel: ");
  Serial.println(turns_right);

}
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
  
}
