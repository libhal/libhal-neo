// Copyright 2024 Khalil Estell
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
class neo_m9n
{
public:
  struct gps_parsed_t
  {
    bool is_locked = false;
    float time;
    float latitude;
    char latitude_direction;
    float longitude;
    char longitude_direction;
    int fix_status;
    int satellites_used;
    float hdop;
    float altitude;
    char altitude_units;
    float height_of_geoid;
    char height_of_geoid_units;
    char time_since_last_dgps_update;
    char dgps_station_id_checksum[10];
  };

  [[nodiscard]] static result<neo_m9n> create(hal::serial& p_serial);
  hal::result<gps_parsed_t> read_raw_gps();
  hal::result<gps_parsed_t> calculate_lon_lat(gps_parsed_t const& p_gps_data);
  hal::result<gps_parsed_t> read();

private:
  neo_m9n(hal::serial& p_serial);
  hal::serial* m_serial;
  std::array<hal::byte, 512> m_gps_buffer;
  gps_parsed_t m_gps_data;
};
}  // namespace hal::neo