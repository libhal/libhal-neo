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

#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include <libhal-util/as_bytes.hpp>
#include <libhal-util/streams.hpp>
#include <libhal/functional.hpp>
#include <libhal/serial.hpp>

namespace hal::neo {
class neo_GPS
{
public:
  struct gps_parsed_t
  {
    float time;
    float latitude;
    std::string latitude_direction;
    float longitude;
    std::string longitude_direction;
    int fix_status;
    int satellites_used;
    float hdop;
    float altitude;
    std::string altitude_units;
    float height_of_geoid;
    std::string height_of_geoid_units;
    std::string time_since_last_dgps_update;
    std::string dgps_station_id_checksum;
  };

  [[nodiscard]] static result<neo_GPS> create(hal::serial& p_serial);

  hal::result<std::string_view> read_gps();

  hal::result<std::string> calculate_lon_lat(const gps_parsed_t& gps_data);

  //  hal::result<std::string> gps_data_to_string();

private:
  neo_GPS(hal::serial& p_serial);
  hal::serial* m_serial;
  std::array<hal::byte, 512> m_gps_buffer;
  std::array<hal::byte, 20> m_gps_parsed_buffer;
  gps_parsed_t m_gps_data;
  std::string m_gps_data_view;
  hal::stream::find start_of_line_finder;
  hal::stream::fill_upto end_of_line_finder;
};
}  // namespace hal::neo