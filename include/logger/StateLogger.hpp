#pragma once

#include <fstream>
#include <string>
#include "sensors.hpp"

class StateLogger {
public:
    explicit StateLogger(const std::string& filename);
    ~StateLogger();

    void log(const FusedState& state);

private:
    std::ofstream log_file_;
};
