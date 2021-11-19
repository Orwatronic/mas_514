/*
* ------------ MAS514 ---------------------------------------------------------------
* Lab Exercise 3 - Problem 2 
* Ultrasonic Sensor HC-SR04 and Arduino 
*/
// defines pins numbers
const int trigPin = 9;
const int echoPin = 10;

// defines variables
long Delta_t; // Duration [micro seconds]
double d; // Measured distance to nearest object [cm]

void setup() {
pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin, INPUT); // Sets the echoPin as an Input
Serial.begin(9600); // Starts the serial communication
}
void loop() {
// Clears the trigPin
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
Delta_t = pulseIn(echoPin, HIGH);

// Calculate the distance (d) below based on the duration (Delta_t) specified above
//--------------------------------------------------------------------------------
d = 0;
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------

// Prints the distance on the Serial Monitor
Serial.print("Distance: ");
Serial.println(d);
}
