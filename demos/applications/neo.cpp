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

  hal::print(console, "Initializing GPS...\n");
  auto neoGPS = HAL_CHECK(hal::neo::neo_GPS::create(gps));
  hal::print(console, "GPS created! \n");
  hal::print(console, "***You may need to wait a few minutes before having a full GPS lock***\n");
  

  while (true) {
    hal::delay(clock, 1000ms);

    auto coordinate_data = HAL_CHECK(neoGPS.read_coordinates());
    hal::print(console, "\n=================== GPS Coordinate Data ===================\n");
    hal::print(console, coordinate_data);



  }

  return hal::success();
}
