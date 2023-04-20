#include "gps_data.hpp"

// Default constructor
GPSData::GPSData() : lat(0), lon(0), alt(0), hdop(0) {}

// Constructor that takes latitude, longitude, altitude, and HDOP values
GPSData::GPSData(double latitude, double longitude, double altitude, double hdop)
    : lat(latitude), lon(longitude), alt(altitude), hdop(hdop) {}

// Constructor that takes a json object and initializes the struct fields
GPSData::GPSData(const nlohmann::json& json_data) {
    lat = json_data["lat"];
    lon = json_data["lon"];
    alt = json_data["alt"];
    hdop = json_data["hdop"];
}

