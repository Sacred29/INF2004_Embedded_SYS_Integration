#ifndef wheel_encoder_driver_h
#define wheel_encoder_driver_h
void setupWheelEncoderPins(uint left_wheel_pin, uint right_wheel_pin);
void leftWheelPulseCounting();
void rightWheelPulseCounting();
void getLeftWheelInfo(float *left_wheel_speed, float *left_distance_travel);
void getRightWheelInfo(float *right_wheel_speed, float *right_distance_travel);
void resetDistanceTraveled(float *left_distance, float *right_distance);
#endif