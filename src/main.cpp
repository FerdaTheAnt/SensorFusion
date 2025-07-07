#include "csv_loader.hpp"
#include "complementary_filter.hpp"
#include "fusion.hpp"
#include <iostream>

int main() {
    auto imu_data = loadIMUCSV("data/imu.csv");
    if(imu_data.empty()) {
        std::cerr << "No data loaded.\n";
        return 1;
    }
    auto gps_data = loadGPSCSV("data/gps.csv");
    if(gps_data.empty()) {
        std::cerr << "No data loaded.\n";
        return 1;
    }

    std::vector<FusedState> results;
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    results.push_back({imu_data[0].timestamp, orientation});

    for(size_t i = 1; i < imu_data.size(); ++i) {
        auto curr_orientation = runComplementaryFilter(imu_data[i-1], imu_data[i], orientation);
        auto fused = fuseIMUandGPS(imu_data[i], gps_data[i], orientation);
        results.push_back(fused);
        orientation = curr_orientation;
    }

    for (const auto& state : results) {
        std::cout
            << state.timestamp << ", "
            << state.orientation.w() << " "
            << state.orientation.x() << " "
            << state.orientation.y() << " "
            << state.orientation.z() << "\n";
    }

    return 0;
}
