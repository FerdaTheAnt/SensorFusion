# SensorFusion

A small project made mostly to learn something about modern CPP, Boost, CMake, networking and sensor fusion.

So far, the program expects to get JSON sensory data from GPS, accelerometer and gyroscope
over UDP socket 5005.

It uses a complementary filter to fuse gyro and accelerometer data and logs it with the gps to a csv log file.

Two Python matplotlib scripts are included for visualisation, so far only for orientation estimation.
