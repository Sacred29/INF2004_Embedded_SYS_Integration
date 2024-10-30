#ifndef ultrasonic_driver_h
#define ultrasonic_driver_h
void setupUltrasonicPins(uint trigPin, uint echoPin);
void sentTrigPulse(uint trigPin, uint echoPin);
uint64_t measureEchoPulse(uint trigPin, uint echoPin);
double getDistance(uint trigPin, uint echoPin);
uint64_t getPulse(uint trigPin, uint echoPin);
double getCm(uint trigPin, uint echoPin);
#endif