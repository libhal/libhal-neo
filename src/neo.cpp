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


hal::result<std::string_view> neo_GPS::read_coordinates()
{
    using namespace std::literals;

    auto bytes_read_array = HAL_CHECK(m_serial->read(m_gps_buffer)).data;

    auto start_of_line_found = bytes_read_array | start_of_line_finder;
    auto end_of_line_found = start_of_line_found | end_of_line_finder;

    std::string_view gps_data(
        reinterpret_cast<const char*>(start_of_line_found.data()), 
        end_of_line_found.data() - start_of_line_found.data());

    if (work_state::finished == end_of_line_finder.state()) {
        start_of_line_finder = hal::stream::find(hal::as_bytes(start_of_line));
        end_of_line_finder = hal::stream::fill_upto(hal::as_bytes(end_of_line), m_gps_buffer);

        return hal::result<std::string_view>(gps_data);
    }
}


hal::result<std::string_view> neo_GPS::build_output_string(const gps_parsed_t& gpsCoordinate) {
    std::string output_string;
    output_string += "Time: " + gpsCoordinate.time + '\n';
    output_string += "Latitude: " + gpsCoordinate.latitude + " " + gpsCoordinate.latitude_dir + '\n';
    output_string += "Longitude: " + gpsCoordinate.longitude + " " + gpsCoordinate.longitude_dir + '\n';
    output_string += "Fix quality: " + std::to_string(gpsCoordinate.fix_quality) + '\n';
    output_string += "Number of satellites: " + std::to_string(gpsCoordinate.num_of_satellites) + '\n';
    output_string += "Horizontal dilution: " + std::to_string(gpsCoordinate.horizontal_dilution) + '\n';
    output_string += "Altitude: " + std::to_string(gpsCoordinate.altitude) + " " + gpsCoordinate.altitude_units + '\n';
    output_string += "Geoidal separation: " + std::to_string(gpsCoordinate.geoidal_separation) + " " + gpsCoordinate.separation_units + '\n';
    output_string += "Age: " + gpsCoordinate.age + '\n';
    output_string += "Checksum: " + gpsCoordinate.checksum;
    return hal::result<std::string_view>(output_string);
}

}  // namespace hal::neo