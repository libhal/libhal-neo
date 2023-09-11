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

#include <libhal-neo/nmea_parser.hpp>
#include <libhal-util/as_bytes.hpp>
#include <libhal-util/streams.hpp>
#include <libhal/functional.hpp>
#include <libhal/serial.hpp>

namespace hal::neo {

struct nmea_parse_t
{
  std::span<const hal::byte> remaining;
  bool end_of_token_found;
};

nmea_parse_t parse(std::span<nmea_parser*> p_parsers,
                   std::span<const hal::byte> p_data);

class GGA_Sentence : public nmea_parser
{
public:
  GGA_Sentence();
  struct gga_data_t
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
  std::span<const hal::byte> parse(std::span<const hal::byte> p_data) override;
  state_t state() override;
  void reset() override;
  gga_data_t read();
  gga_data_t calculate_lon_lat(const gga_data_t& p_gps_data);

private:
  gga_data_t m_gga_data;
  nmea_parser::state_t m_state;
};

class GSA_Sentence : public nmea_parser
{
public:
  GSA_Sentence();
  struct gsa_data_t
  {
    char mode;
    int fix_type;
    std::array<int, 12> satellite_ids{};  // assuming up to 12 satellites
    float pdop;
    float hdop;
    float vdop;
  };
  std::span<const hal::byte> parse(std::span<const hal::byte> p_data) override;
  state_t state() override;
  void reset() override;
  gsa_data_t read();

private:
  gsa_data_t m_gsa_data;
  nmea_parser::state_t m_state;
};

class GSV_Sentence : public nmea_parser
{
public:
  GSV_Sentence();
  struct satellite_data_t
  {
    int number_of_messages;
    int message_number;
    int satellites_in_view;
    int id;
    int elevation;
    int azimuth;
    int snr;
  };
  std::span<const hal::byte> parse(std::span<const hal::byte> p_data) override;
  state_t state() override;
  void reset() override;
  satellite_data_t read();

private:
  satellite_data_t m_satellite_data;
  nmea_parser::state_t m_state;
};

class RMC_Sentence : public nmea_parser
{
public:
  RMC_Sentence();
  struct rmc_data_t
  {

    int reading_status = 0;

    float time;
    char status;
    float latitude;
    char latitude_direction;
    float longitude;
    char longitude_direction;
    float speed;
    float track_angle;
    int date;
    float magnetic_variation;
    char magnetic_direction;
  };
  std::span<const hal::byte> parse(std::span<const hal::byte> p_data) override;
  state_t state() override;
  void reset() override;
  rmc_data_t read();

private:
  rmc_data_t m_rmc_data;
  nmea_parser::state_t m_state;
};

}  // namespace hal::neo