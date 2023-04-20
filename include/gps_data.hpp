#ifndef GPS_DATA_HPP
#define GPS_DATA_HPP

#include <nlohmann/json.hpp>

struct GPSData {
    double lat;
    double lon;
    double alt;
    double hdop;

    GPSData();
    GPSData(double lat, double lon, double alt, double hdop);
    GPSData(const nlohmann::json& json_data);
};

#endif // GPS_DATA_HPP

