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
    std::string time;
    std::string latitude;
    std::string latitude_dir;
    std::string longitude;
    std::string longitude_dir;
    int fix_quality;
    int num_of_satellites;
    float horizontal_dilution;
    float altitude;
    std::string altitude_units;
    float geoidal_separation;
    std::string separation_units;
    std::string age;
    std::string checksum;
  };

  [[nodiscard]] static result<neo_GPS> create(hal::serial& p_serial);

  hal::result<std::string_view> read_coordinates();

private:
  neo_GPS(hal::serial& p_serial);
  hal::serial* m_serial;
  std::array<hal::byte, 512> m_gps_buffer;
  // gps_parsed_t gpsCoordinate;
  hal::stream::find start_of_line_finder;
  hal::stream::fill_upto end_of_line_finder;
};
}  // namespace hal::neo