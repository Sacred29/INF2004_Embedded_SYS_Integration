#ifndef wheel_encoder_driver_h
#define wheel_encoder_driver_h
void setupWheelEncoderPins(uint left_wheel_pin, uint right_wheel_pin);
void leftWheelPulseCounting();
void rightWheelPulseCounting();
float getLeftWheelSpeed(float pulse_width);
float getRightWheelSpeed(float pulse_width);
#endif