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
#include <libhal-util/timeout.hpp>

namespace hal::neo {

// constructor
neo_GPS::neo_GPS(hal::serial& p_serial)
  : m_serial(&p_serial)
{
}

result<neo_GPS> neo_GPS::create(hal::serial& p_serial)
{
  neo_GPS new_neo(p_serial);
  return new_neo;
}

hal::result<std::string_view> neo_GPS::read_coordinates()
{
  // Setup
  using namespace std::literals;

  auto bytes_read_array = HAL_CHECK(m_serial->read(m_gps_buffer)).data;

  auto start_of_line = hal::as_bytes("$GPGGA,"sv);
  hal::stream::find start_of_line_finder(start_of_line);

  auto end_of_line = hal::as_bytes("\n"sv);
  hal::stream::fill_upto end_of_line_finder(end_of_line, m_gps_buffer);

  auto remaining = bytes_read_array | start_of_line_finder | end_of_line_finder;

  std::string_view remaining_str(
    reinterpret_cast<const char*>(remaining.data()), remaining.size());

  return remaining_str;
}

}  // namespace hal::neo