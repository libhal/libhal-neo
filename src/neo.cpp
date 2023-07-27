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

#include <string_view>


#include <algorithm>
#include <array>
#include <span>

#include <libhal-neo/neo.hpp>
#include <libhal-util/as_bytes.hpp>
#include <libhal-util/streams.hpp>
#include <libhal-util/timeout.hpp>
#include <libhal/serial.hpp>

#include "neo_constants.hpp"

namespace hal::neo {

nmea_parser::nmea_parser(hal::serial& p_serial)
  : m_serial(&p_serial)
{
  reset();
}

result<nmea_parser> nmea_parser::create(hal::serial& p_serial)
{
  nmea_parser new_neo(p_serial);
  return new_neo;
}

hal::result<nmea_parser::gps_parsed_t> nmea_parser::get()
{
  auto data = HAL_CHECK(m_serial->read(m_buffer)).data;
  std::span<const hal::byte> remaining{};

  switch (m_state) {
    case state::none:
      remaining = data | m_gga;
      if (hal::finished(m_gga)) {
        m_state = state::gga;
        break;
      }

      remaining = data | m_gll;
      if (hal::finished(m_gll)) {
        m_state = state::gll;
        break;
      }

      remaining = data | m_gsa;
      if (hal::finished(m_gsa)) {
        m_state = state::gsa;
        break;
      }

      remaining = data | m_gsv;
      if (hal::finished(m_gsv)) {
        m_state = state::gsv;
        break;
      }

      remaining = data | m_rmc;
      if (hal::finished(m_rmc)) {
        m_state = state::rmc;
        break;
      }

      remaining = data | m_vtg;
      if (hal::finished(m_vtg)) {
        m_state = state::vtg;
        break;
      }

      remaining = data | m_zda;
      if (hal::finished(m_zda)) {
        m_state = state::zda;
        break;
      }

      break;
    case state::gga:
      m_gps_data = (parse(remaining, GPGGA_FORMAT)).value();
      break;
    case state::gll:
      m_gps_data = (parse(remaining, GPGLL_FORMAT)).value();
      break;
    case state::gsa:
      m_gps_data = (parse(remaining, GPGSA_FORMAT)).value();
      break;
    case state::gsv:
      m_gps_data = (parse(remaining, GPGSV_FORMAT)).value();
      break;
    case state::rmc:
      m_gps_data = (parse(remaining, GPRMC_FORMAT)).value();
      break;
    case state::vtg:
      m_gps_data = (parse(remaining, GPVTG_FORMAT)).value();
      break;
    case state::zda:
      m_gps_data = (parse(remaining, GPZDA_FORMAT)).value();
      break;
  }

  return m_gps_data;
}

hal::result<nmea_parser::gps_parsed_t> nmea_parser::parse(std::span<const hal::byte> remaining, const char* format)
{

  auto end_of_line_finder = hal::stream::find(hal::as_bytes(end_of_line));
  auto end_of_line_found = remaining | end_of_line_finder;

  std::string_view gps_data(reinterpret_cast<const char*>(remaining.data()),
                            end_of_line_found.data() - remaining.data());

  int ret = sscanf(gps_data.data(),
                   format,
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

  m_gps_data.is_locked = (ret < 10) ? false : true;

  return hal::result<nmea_parser::gps_parsed_t>(m_gps_data);
}

hal::result<nmea_parser::gps_parsed_t> nmea_parser::calculate_lon_lat(
  const nmea_parser::gps_parsed_t& p_gps_data)
{

  nmea_parser::gps_parsed_t modified_data = p_gps_data;
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

  return hal::result<nmea_parser::gps_parsed_t>(modified_data);
}

hal::result<nmea_parser::gps_parsed_t> nmea_parser::read()
{
  auto gps_data = HAL_CHECK(get());
  auto lon_lat = HAL_CHECK(calculate_lon_lat(gps_data));
  reset();
  return hal::result<nmea_parser::gps_parsed_t>(lon_lat);
}

void nmea_parser::reset()
{
  using namespace std::literals;

  m_gga = hal::stream::find(hal::as_bytes("$GPGGA,"sv));
  m_gll = hal::stream::find(hal::as_bytes("$GPGLL,"sv));
  m_gsa = hal::stream::find(hal::as_bytes("$GPGSA,"sv));
  m_gsv = hal::stream::find(hal::as_bytes("$GPGSV,"sv));
  m_rmc = hal::stream::find(hal::as_bytes("$GPRMC,"sv));
  m_vtg = hal::stream::find(hal::as_bytes("$GPVTG,"sv));
  m_zda = hal::stream::find(hal::as_bytes("$GPZDA,"sv));
}

}  // namespace hal::neo