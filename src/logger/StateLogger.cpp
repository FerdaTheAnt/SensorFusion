#include "logger/StateLogger.hpp"
#include <iomanip>
#include <iostream>

const int LOG_PRECISION = 3;

StateLogger::StateLogger(const std::string& filename) {
    log_file_.open(filename, std::ios::out);
    if(log_file_.is_open()){
        log_file_ << "timestamp,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,orient_w,orient_x,orient_y,orient_z\n";
    }
}

StateLogger::~StateLogger() {
    if(log_file_.is_open()){
        log_file_.close();
    }
}

void StateLogger::log(const FusedState& state) {
    if(!log_file_.is_open()) return;

    log_file_ << std::fixed << std::setprecision(LOG_PRECISION)
        << state.timestamp << ","
        << state.position.x() << "," << state.position.y() << "," << state.position.z() << ","
        << state.velocity.x() << "," << state.velocity.y() << "," << state.velocity.z() << ","
        << state.orientation.w() << "," << state.orientation.x() << ","
        << state.orientation.y() << "," << state.orientation.z()
        << std::endl;
}

