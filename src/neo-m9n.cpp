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

#include <libhal-neo/neo-m9n.hpp>

#include <algorithm>
#include <array>
#include <span>

#include <libhal-util/as_bytes.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/streams.hpp>

#include "neo-m9n_constants.hpp"

namespace hal::neo {

neo_m9n::neo_m9n(hal::serial& p_serial)
  : m_serial(&p_serial)
{
}

result<neo_m9n> neo_m9n::create(hal::serial& p_serial)
{
  neo_m9n new_neo(p_serial);
  return new_neo;
}

hal::result<neo_m9n::gps_parsed_t> neo_m9n::read_raw_gps()
{
  using namespace std::literals;

  auto bytes_read_array = HAL_CHECK(m_serial->read(m_gps_buffer)).data;
  auto start_of_line_finder = hal::stream_find(hal::as_bytes(start_of_line));
  auto end_of_line_finder = hal::stream_find(hal::as_bytes(end_of_line));

  auto start_of_line_found = bytes_read_array | start_of_line_finder;
  auto end_of_line_found = start_of_line_found | end_of_line_finder;

  std::string_view gps_data(
    reinterpret_cast<const char*>(start_of_line_found.data()),
    end_of_line_found.data() - start_of_line_found.data());

  int ret = sscanf(gps_data.data(),
                   ",%f,%f,%c,%f,%c,%d,%d,%f,%f,%c,%f,%c,,%s,",
                   &m_gps_data.time,
                   &m_gps_data.latitude,
                   &m_gps_data.latitude_direction,
                   &m_gps_data.longitude,
                   &m_gps_data.longitude_direction,
                   &m_gps_data.fix_status,
                   &m_gps_data.satellites_used,
                   &m_gps_data.hdop,
                   &m_gps_data.altitude,
                   &m_gps_data.altitude_units,
                   &m_gps_data.height_of_geoid,
                   &m_gps_data.height_of_geoid_units,
                   &m_gps_data.time_since_last_dgps_update,
                   &m_gps_data.dgps_station_id_checksum);

  m_gps_data.is_locked = (ret < 7) ? false : true;

  return hal::result<neo_m9n::gps_parsed_t>(m_gps_data);
}

hal::result<neo_m9n::gps_parsed_t> neo_m9n::calculate_lon_lat(
  const neo_m9n::gps_parsed_t& p_gps_data)
{

  neo_m9n::gps_parsed_t modified_data = p_gps_data;
  char lon_dir = modified_data.longitude_direction;
  char lat_dir = modified_data.latitude_direction;
  float lon = modified_data.longitude;
  float lat = modified_data.latitude;

  if (lon_dir == 'W') {
    lon = -lon;
  }
  if (lat_dir == 'S') {
    lat = -lat;
  }

  float lon_intpart = static_cast<int>(lon / 100);
  float lon_fractpart = lon - (lon_intpart * 100);
  lon = lon_intpart + (lon_fractpart / 60);

  float lat_intpart = static_cast<int>(lat / 100);
  float lat_fractpart = lat - (lat_intpart * 100);
  lat = lat_intpart + (lat_fractpart / 60);

  modified_data.longitude = lon;
  modified_data.latitude = lat;

  return hal::result<neo_m9n::gps_parsed_t>(modified_data);
}

hal::result<neo_m9n::gps_parsed_t> neo_m9n::read()
{
  auto gps_data = HAL_CHECK(read_raw_gps());
  auto lon_lat = HAL_CHECK(calculate_lon_lat(gps_data));
  return hal::result<neo_m9n::gps_parsed_t>(lon_lat);
}

}  // namespace hal::neo