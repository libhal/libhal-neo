// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <libhal-neo/neo.hpp>

#include <algorithm>
#include <array>
#include <span>

#include <libhal-util/as_bytes.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/streams.hpp>

#include "neo_constants.hpp"

namespace hal::neo {

// constructor
neo_GPS::neo_GPS(hal::serial& p_serial)
  : m_serial(&p_serial)
  , start_of_line_finder(hal::as_bytes(start_of_line))
  , end_of_line_finder(hal::as_bytes(end_of_line), m_gps_buffer)
{
}

result<neo_GPS> neo_GPS::create(hal::serial& p_serial)
{
  neo_GPS new_neo(p_serial);
  return new_neo;
}


hal::result<std::string_view> neo_GPS::read_gps()
{
    using namespace std::literals;

    auto bytes_read_array = HAL_CHECK(m_serial->read(m_gps_buffer)).data;

    // Initialize finders for start and end of line
    auto start_of_line_finder = hal::stream::find(hal::as_bytes(start_of_line));
    auto end_of_line_finder = hal::stream::find(hal::as_bytes(end_of_line));
    auto comma_finder = hal::stream::find(hal::as_bytes(comma_delimiter));
    auto fill_upto_comma_finder = hal::stream::fill_upto(hal::as_bytes(comma_delimiter), m_gps_parsed_buffer);

    m_gps_data_view = "";  // Reset before each read

    auto start_of_line_found = bytes_read_array | start_of_line_finder;
    auto end_of_line_found = start_of_line_found | end_of_line_finder;

    std::string_view gps_data(
        reinterpret_cast<const char*>(start_of_line_found.data()),
        end_of_line_found.data() - start_of_line_found.data());

    for (int i = 0; i < 14; ++i) {
        // Find the comma
        auto comma_found = hal::as_bytes(gps_data) | comma_finder;

        // Check if we have reached the end of the line
        if (comma_found.data() == hal::as_bytes(gps_data).data() + hal::as_bytes(gps_data).size()) {
            break;
        }

        // Skip the comma itself
        hal::stream::skip comma_skipper(1);
        auto start_of_data = comma_found | comma_skipper;

        // Fill up to the next comma
        auto end_of_data = start_of_data | fill_upto_comma_finder;

        // Create a string_view for the data item
        std::string item_data(reinterpret_cast<const char*>(start_of_data.data()),
                            end_of_data.data() - start_of_data.data() - 1); // -1 to exclude comma
        m_gps_data_view += item_data + " | ";

        switch (i) {
            case 0: m_gps_data.time = std::stof(item_data); break;
            case 1: m_gps_data.latitude = std::stof(item_data); break;
            case 2: m_gps_data.latitude_direction = item_data; break;
            case 3: m_gps_data.longitude = std::stof(item_data); break;
            case 4: m_gps_data.longitude_direction = item_data; break;
            case 5: m_gps_data.fix_status = std::stoi(item_data); break;
            case 6: m_gps_data.satellites_used = std::stoi(item_data); break;
            case 7: m_gps_data.hdop = std::stof(item_data); break;
            case 8: m_gps_data.altitude = std::stof(item_data); break;
            case 9: m_gps_data.altitude_units = item_data; break;
            case 10: m_gps_data.height_of_geoid = std::stof(item_data); break;
            case 11: m_gps_data.height_of_geoid_units = item_data; break;
            case 12: m_gps_data.time_since_last_dgps_update = item_data; break;
            case 13: m_gps_data.dgps_station_id_checksum = item_data; break;
        }

        // Find the position of the last comma in gps_data
        auto last_comma_pos = gps_data.find(',', comma_found.data() - hal::as_bytes(gps_data).data());

        // Cut off the processed part of the string
        gps_data.remove_prefix(last_comma_pos + 1);

        // Reset the finders
        comma_finder = hal::stream::find(hal::as_bytes(comma_delimiter));
        fill_upto_comma_finder = hal::stream::fill_upto(hal::as_bytes(comma_delimiter), m_gps_parsed_buffer);
    }

    // return hal::result<neo_GPS::gps_parsed_t>(m_gps_data);
    return hal::result<std::string_view>(m_gps_data_view);
}


hal::result<std::string> neo_GPS::calculate_lon_lat(const neo_GPS::gps_parsed_t& gps_data) {
    std::string lon_lat;
    std::string lon_dir = gps_data.longitude_direction;
    std::string lat_dir = gps_data.latitude_direction;
    float lon = gps_data.longitude;
    float lat = gps_data.latitude;

    if (lon_dir == "W") {
        lon = -lon;
    }
    if (lat_dir == "S") {
        lat = -lat;
    }

    float lon_intpart = static_cast<int>(lon / 100);
    float lon_fractpart = lon - (lon_intpart * 100);
    lon = lon_intpart + (lon_fractpart / 60);

    float lat_intpart = static_cast<int>(lat / 100);
    float lat_fractpart = lat - (lat_intpart * 100);
    lat = lat_intpart + (lat_fractpart / 60);

    lon_lat = std::to_string(lon) + ", " + std::to_string(lat);

    return hal::result<std::string>(lon_lat);
}


}  // namespace hal::neo