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

#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal-neo/neo.hpp>
#include "../hardware_map.hpp"

hal::status application(hardware_map& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock;
  auto& console = *p_map.console;
  auto& gps = *p_map.gps;

  std::array<hal::byte, 512> buffer{};

  hal::print(console, "Initializing GPS...\n");
  auto neoGPS = HAL_CHECK(hal::neo::neo_GPS::create(gps));
  hal::print(console, "GPS created! \n");

  while (true) {
    // Wait 1 second before reading response back
    hal::delay(clock, 1000ms);
    // Read response back from serial port
    auto received = HAL_CHECK(neoGPS.read_raw_gps()).data;

    hal::print(console, "\n=================== GPS RESPONSE ===================n\n");
    hal::print(console, received);

    // // Echo back anything received
    // hal::print(console, "\n=================== Data I Should Get ===================n\n");
    // std::array<hal::byte, 512> read_buffer;
    // auto data = HAL_CHECK(gps.read(read_buffer)).data;
    // hal::print(console, data);

  }

  return hal::success();
}
