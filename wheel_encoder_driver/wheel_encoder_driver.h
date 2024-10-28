#ifndef wheel_encoder_driver_h
#define wheel_encoder_driver_h
void setupWheelEncoderPins(uint inputPin);
float calculate_wheel_speed(float pulse_width);
float wheel_encoder(uint gpio);
#endif