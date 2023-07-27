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

class nmea_parser
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

  enum state : std::uint8_t
  {
    none = 0,
    gga,
    gll,
    gsa,
    gsv,
    rmc,
    vtg,
    zda,
  };

  [[nodiscard]] static result<nmea_parser> create(hal::serial& p_serial);
  hal::result<gps_parsed_t> get();
  hal::result<gps_parsed_t> parse(std::span<const hal::byte> remaining, const char* format);
  hal::result<gps_parsed_t> calculate_lon_lat(const gps_parsed_t& p_gps_data);
  hal::result<gps_parsed_t> read();

private:
  nmea_parser(hal::serial& p_serial);
  void reset();
  hal::serial* m_serial;
  hal::stream::find m_gga{ std::span<hal::byte>() };
  hal::stream::find m_gll{ std::span<hal::byte>() };
  hal::stream::find m_gsa{ std::span<hal::byte>() };
  hal::stream::find m_gsv{ std::span<hal::byte>() };
  hal::stream::find m_rmc{ std::span<hal::byte>() };
  hal::stream::find m_vtg{ std::span<hal::byte>() };
  hal::stream::find m_zda{ std::span<hal::byte>() };
  state m_state = state::none;
  std::array<hal::byte, 32> m_buffer{};
  gps_parsed_t m_gps_data;
};
}  // namespace hal::neo
