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

hal::result<std::span<hal::byte>> neo_GPS::read_raw_gps()
{
  auto bytes_read_array = HAL_CHECK(m_serial->read(gps_buffer)).data;
  return bytes_read_array;
}

}  // namespace hal::neo