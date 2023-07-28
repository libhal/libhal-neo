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

class GGA_Sentence
{
public:
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

  [[nodiscard]] static result<GGA_Sentence> create();
  hal::result<gga_data_t> parse(std::span<hal::byte>& bytes_read_array);
  hal::result<gga_data_t> read(std::span<hal::byte>& bytes_read_array);
  hal::result<gga_data_t> calculate_lon_lat(const gga_data_t& p_gps_data);

private:
  GGA_Sentence();
  gga_data_t m_gga_data;
};

class GSA_Sentence
{
public:
  struct gsa_data_t
  {
    char mode;
    int fix_type;
    std::array<int, 12> satellite_ids{};  // assuming up to 12 satellites
    float pdop;
    float hdop;
    float vdop;
  };

  [[nodiscard]] static result<GSA_Sentence> create();
  hal::result<gsa_data_t> parse(std::span<hal::byte>& bytes_read_array);
  hal::result<gsa_data_t> read(std::span<hal::byte>& bytes_read_array);

private:
  GSA_Sentence();
  gsa_data_t m_gsa_data;
};

class GSV_Sentence
{
public:
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

  [[nodiscard]] static result<GSV_Sentence> create();
  hal::result<satellite_data_t> parse(std::span<hal::byte>& bytes_read_array);
  hal::result<satellite_data_t> read(std::span<hal::byte>& bytes_read_array);

private:
  GSV_Sentence();
  satellite_data_t m_satellite_data;
};

class RMC_Sentence
{
public:
  struct rmc_data_t
  {
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

  [[nodiscard]] static result<RMC_Sentence> create();
  hal::result<rmc_data_t> parse(std::span<hal::byte>& bytes_read_array);
  hal::result<rmc_data_t> read(std::span<hal::byte>& bytes_read_array);

private:
  RMC_Sentence();
  rmc_data_t m_rmc_data;
};

class nmea_parser
{
public:
  [[nodiscard]] static result<nmea_parser> create(hal::serial& p_serial);
  hal::result<std::span<hal::byte>> read();

private:
  nmea_parser(hal::serial& p_serial);
  hal::serial* m_serial;
  std::array<hal::byte, 512> m_gps_buffer{};
};

}  // namespace hal::neo