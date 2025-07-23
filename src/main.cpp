#include "net/UDPReceiver.hpp"
#include "logger/StateLogger.hpp"
#include "SensorFusionEngine.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <boost/json/src.hpp>
namespace json = boost::json;

int main() {
    SensorFusionEngine engine;
    StateLogger logger("logs/fusion_output.csv");
    UDPReceiver receiver(5005, [&engine](const std::string& msg){
        try {
            auto j = json::parse(msg).as_object();

            IMUData imu;
            imu.timestamp = j["timestamp"].as_double();
            auto accel = j["accelerometer"].as_object();
            imu.accel = Eigen::Vector3d(
                accel["x"].as_double(), accel["y"].as_double(), accel["z"].as_double()
            );
            auto gyro = j["gyroscope"].as_object();
            imu.gyro = Eigen::Vector3d(
                gyro["x"].as_double(), gyro["y"].as_double(), gyro["z"].as_double()
            );

            GPSData gps;
            gps.timestamp = j["timestamp"].as_double();
            auto position = j["gps"].as_object();
            gps.position = Eigen::Vector3d(
                position["lat"].as_double(), position["lon"].as_double(), position["alt"].as_double()
            );
            gps.velocity = Eigen::Vector3d::Zero();

            engine.handleIMU(imu);
            engine.handleGPS(gps);
        } catch (std::exception& e) {
            std::cerr << "[Parser] Invalid message: " << e.what() << std::endl;
        }
    });
    receiver.start();

    while (true) {
        auto state = engine.getCurrentState();
        std::cout
            << state.timestamp << ", "
            << state.orientation.w() << " "
            << state.orientation.x() << " "
            << state.orientation.y() << " "
            << state.orientation.z() << "\n";
        logger.log(state);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
