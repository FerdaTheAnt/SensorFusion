#include "net/UDPReceiver.hpp"
#include "logger/StateLogger.hpp"
#include "SensorFusionEngine.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/program_options/parsers.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include <boost/json/src.hpp>
#include <boost/program_options.hpp>
namespace json = boost::json;
namespace po = boost::program_options;

int main(int argc, char *argv[]) {
    SensorFusionEngine engine;
    std::string log_filename;

    po::options_description options_description;
    options_description.add_options()
        ("help,h", "Help")
        ("engine,e", po::value<std::string>()->default_value("kalman"), "Engine to fuse sensor data. Options: kalman, complementary")
        ("file,f", po::value(&log_filename)->default_value("logs/fusion_output.csv"), "Log file path");

    po::variables_map variables_map;

    try {
        po::store(po::parse_command_line(argc, argv, options_description), variables_map); 
        po::notify(variables_map);
    } catch (std::exception &e) {
        std::cerr << e.what();
    }

    if(variables_map.count("help"))
    {
        std::cout << options_description;
        return 0;
    } 
    if(variables_map.count("engine"))
    {
        std::string method = variables_map["engine"].as<std::string>();
        if(method == "complementary")
        {
            engine.setEngineMethod(method);
        }
        else if(method == "kalman")
        {
            engine.setEngineMethod(method);
        }
        else
        {
            std::cout << "Not a valid engine use either kalman or complementary\n";
            return 1;
        }
    }

    //StateLogger logger("logs/fusion_output.csv");
    StateLogger logger(log_filename);

    UDPReceiver init_receiver(5005, [&engine](const std::string& msg){
        try {
            auto j = json::parse(msg).as_object();

            IMUData imu;
            imu.timestamp = j["timestamp"].as_double();
            auto accel = j["accelerometer"].as_object();
            imu.accel = Eigen::Vector3d(
                accel["x"].as_double(), accel["y"].as_double(), accel["z"].as_double()// - 9.8066
            );
            auto gyro = j["gyroscope"].as_object();
            imu.gyro = Eigen::Vector3d(
                gyro["x"].as_double(), gyro["y"].as_double(), gyro["z"].as_double()
            );

            MagnetData magnet;
            magnet.timestamp = j["timestamp"].as_double();
            auto field = j["magnetometer"].as_object();
            magnet.field = Eigen::Vector3d(
                field["x"].as_double(), field["y"].as_double(), field["z"].as_double()
            );
            
            engine.initHandleMagnet(magnet);
            engine.initHandleIMU(imu);

        } catch (std::exception& e) {
            std::cerr << "[Parser] Invalid message: " << e.what() << std::endl;
        }
    });

    init_receiver.start();
    for(int i = 0; i < 10; i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
    init_receiver.stop();
    engine.computeInitialOrientation();


    UDPReceiver receiver(5005, [&engine](const std::string& msg){
        try {
            auto j = json::parse(msg).as_object();

            IMUData imu;
            imu.timestamp = j["timestamp"].as_double();
            auto accel = j["accelerometer"].as_object();
            imu.accel = Eigen::Vector3d(
                accel["x"].as_double(), accel["y"].as_double(), accel["z"].as_double()// - 9.8066
            );
            auto gyro = j["gyroscope"].as_object();
            imu.gyro = Eigen::Vector3d(
                gyro["x"].as_double(), gyro["y"].as_double(), gyro["z"].as_double()
            );

            MagnetData magnet;
            magnet.timestamp = j["timestamp"].as_double();
            auto field = j["magnetometer"].as_object();
            magnet.field = Eigen::Vector3d(
                field["x"].as_double(), field["y"].as_double(), field["z"].as_double()
            );

            GPSData gps;
            gps.timestamp = j["timestamp"].as_double();
            auto position = j["gps"].as_object();
            gps.position = Eigen::Vector3d(
                position["lat"].as_double(), position["lon"].as_double(), position["alt"].as_double()
            );
            gps.velocity = Eigen::Vector3d::Zero();

            engine.handleOrientation(imu, magnet);
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
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }

    return 0;
}
