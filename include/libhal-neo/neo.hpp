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

#include <libhal/functional.hpp>
#include <libhal/serial.hpp>

namespace hal::neo {
class neo_GPS
{
public:

  [[nodiscard]] static result<neo_GPS> create(hal::serial& p_serial);

  hal::result<serial::read_t> read_raw_gps();

    private : neo_GPS(hal::serial& p_serial);
  hal::serial* m_serial;
  static constexpr uint8_t gps_buffer_size = 512;
};
}  // namespace hal::neo