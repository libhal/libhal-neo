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
}

GGA_Sentence::GGA_Sentence()
{
}

GSA_Sentence::GSA_Sentence()
{
}

GSV_Sentence::GSV_Sentence()
{
}

RMC_Sentence::RMC_Sentence()
{
}

result<nmea_parser> nmea_parser::create(hal::serial& p_serial)
{
  nmea_parser new_nmea_parser(p_serial);
  return new_nmea_parser;
}

result<GGA_Sentence> GGA_Sentence::create()
{
  GGA_Sentence new_GGA_Sentence;
  return new_GGA_Sentence;
}

result<GSA_Sentence> GSA_Sentence::create()
{
  GSA_Sentence new_GSA_Sentence;
  return new_GSA_Sentence;
}

result<GSV_Sentence> GSV_Sentence::create()
{
  GSV_Sentence new_GSV_Sentence;
  return new_GSV_Sentence;
}

result<RMC_Sentence> RMC_Sentence::create()
{
  RMC_Sentence new_RMC_Sentence;
  return new_RMC_Sentence;
}

hal::result<std::span<hal::byte>> nmea_parser::read()
{
  auto bytes_read_array = HAL_CHECK(m_serial->read(m_gps_buffer)).data;
  return hal::result<std::span<hal::byte>>(bytes_read_array);
}

hal::result<GGA_Sentence::gga_data_t> GGA_Sentence::parse(
  std::span<hal::byte>& bytes_read_array)
{
  using namespace std::literals;

  auto start_of_line_finder =
    hal::stream::find(hal::as_bytes(gga_start_of_line));
  auto end_of_line_finder = hal::stream::find(hal::as_bytes(end_of_line));

  auto start_of_line_found = bytes_read_array | start_of_line_finder;
  auto end_of_line_found = start_of_line_found | end_of_line_finder;

  std::string_view gga_data(
    reinterpret_cast<const char*>(start_of_line_found.data()),
    end_of_line_found.data() - start_of_line_found.data());

  int ret = sscanf(gga_data.data(),
                   GGA_FORMAT,
                   &m_gga_data.time,
                   &m_gga_data.latitude,
                   &m_gga_data.latitude_direction,
                   &m_gga_data.longitude,
                   &m_gga_data.longitude_direction,
                   &m_gga_data.fix_status,
                   &m_gga_data.satellites_used,
                   &m_gga_data.hdop,
                   &m_gga_data.altitude,
                   &m_gga_data.altitude_units,
                   &m_gga_data.height_of_geoid,
                   &m_gga_data.height_of_geoid_units,
                   &m_gga_data.time_since_last_dgps_update,
                   &m_gga_data.dgps_station_id_checksum);

  m_gga_data.is_locked = (ret < 10) ? false : true;

  return hal::result<GGA_Sentence::gga_data_t>(m_gga_data);
}

hal::result<GGA_Sentence::gga_data_t> GGA_Sentence::calculate_lon_lat(
  const GGA_Sentence::gga_data_t& p_gps_data)
{

  GGA_Sentence::gga_data_t modified_data = p_gps_data;
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

  return hal::result<GGA_Sentence::gga_data_t>(modified_data);
}

hal::result<GGA_Sentence::gga_data_t> GGA_Sentence::read(
  std::span<hal::byte>& bytes_read_array)
{
  auto gga_data = HAL_CHECK(parse(bytes_read_array));
  auto data = HAL_CHECK(calculate_lon_lat(gga_data));
  return hal::result<GGA_Sentence::gga_data_t>(data);
}

hal::result<GSA_Sentence::gsa_data_t> GSA_Sentence::parse(
  std::span<hal::byte>& bytes_read_array)
{
  using namespace std::literals;

  auto start_of_line_finder =
    hal::stream::find(hal::as_bytes(gsa_start_of_line));
  auto end_of_line_finder = hal::stream::find(hal::as_bytes(end_of_line));

  auto start_of_line_found = bytes_read_array | start_of_line_finder;
  auto end_of_line_found = start_of_line_found | end_of_line_finder;

  std::string_view gsa_data(
    reinterpret_cast<const char*>(start_of_line_found.data()),
    end_of_line_found.data() - start_of_line_found.data());

  int ret = sscanf(gsa_data.data(),
                   GSA_FORMAT,
                   &m_gsa_data.mode,
                   &m_gsa_data.fix_type,
                   &m_gsa_data.satellite_ids[0],
                   &m_gsa_data.satellite_ids[1],
                   &m_gsa_data.satellite_ids[2],
                   &m_gsa_data.satellite_ids[3],
                   &m_gsa_data.satellite_ids[4],
                   &m_gsa_data.satellite_ids[5],
                   &m_gsa_data.satellite_ids[6],
                   &m_gsa_data.satellite_ids[7],
                   &m_gsa_data.satellite_ids[8],
                   &m_gsa_data.satellite_ids[9],
                   &m_gsa_data.satellite_ids[10],
                   &m_gsa_data.satellite_ids[11],
                   &m_gsa_data.pdop,
                   &m_gsa_data.hdop,
                   &m_gsa_data.vdop);

  return hal::result<GSA_Sentence::gsa_data_t>(m_gsa_data);
}

hal::result<GSA_Sentence::gsa_data_t> GSA_Sentence::read(
  std::span<hal::byte>& bytes_read_array)
{
  auto data = HAL_CHECK(parse(bytes_read_array));
  return hal::result<GSA_Sentence::gsa_data_t>(data);
}

hal::result<GSV_Sentence::satellite_data_t> GSV_Sentence::parse(
  std::span<hal::byte>& bytes_read_array)
{
  using namespace std::literals;

  auto start_of_line_finder =
    hal::stream::find(hal::as_bytes(gsv_start_of_line));
  auto end_of_line_finder = hal::stream::find(hal::as_bytes(end_of_line));

  auto start_of_line_found = bytes_read_array | start_of_line_finder;
  auto end_of_line_found = start_of_line_found | end_of_line_finder;

  std::string_view gsv_data(
    reinterpret_cast<const char*>(start_of_line_found.data()),
    end_of_line_found.data() - start_of_line_found.data());

  int ret = sscanf(gsv_data.data(),
                   GSV_FORMAT,
                   &m_satellite_data.number_of_messages,
                   &m_satellite_data.message_number,
                   &m_satellite_data.satellites_in_view,
                   &m_satellite_data.id,
                   &m_satellite_data.elevation,
                   &m_satellite_data.azimuth,
                   &m_satellite_data.snr);

  return hal::result<GSV_Sentence::satellite_data_t>(m_satellite_data);
}

hal::result<GSV_Sentence::satellite_data_t> GSV_Sentence::read(
  std::span<hal::byte>& bytes_read_array)
{
  auto data = HAL_CHECK(parse(bytes_read_array));
  return hal::result<GSV_Sentence::satellite_data_t>(data);
}

hal::result<RMC_Sentence::rmc_data_t> RMC_Sentence::parse(
  std::span<hal::byte>& bytes_read_array)
{
  using namespace std::literals;

  auto start_of_line_finder =
    hal::stream::find(hal::as_bytes(rmc_start_of_line));
  auto end_of_line_finder = hal::stream::find(hal::as_bytes(end_of_line));

  auto start_of_line_found = bytes_read_array | start_of_line_finder;
  auto end_of_line_found = start_of_line_found | end_of_line_finder;

  std::string_view rmc_data(
    reinterpret_cast<const char*>(start_of_line_found.data()),
    end_of_line_found.data() - start_of_line_found.data());

  int ret = sscanf(rmc_data.data(),
                   RMC_FORMAT,
                   &m_rmc_data.time,
                   &m_rmc_data.status,
                   &m_rmc_data.latitude,
                   &m_rmc_data.latitude_direction,
                   &m_rmc_data.longitude,
                   &m_rmc_data.longitude_direction,
                   &m_rmc_data.speed,
                   &m_rmc_data.track_angle,
                   &m_rmc_data.date,
                   &m_rmc_data.magnetic_variation,
                   &m_rmc_data.magnetic_direction);

  return hal::result<RMC_Sentence::rmc_data_t>(m_rmc_data);
}

hal::result<RMC_Sentence::rmc_data_t> RMC_Sentence::read(
  std::span<hal::byte>& bytes_read_array)
{
  auto data = HAL_CHECK(parse(bytes_read_array));
  return hal::result<RMC_Sentence::rmc_data_t>(data);
}

}  // namespace hal::neo