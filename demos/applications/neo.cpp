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

#include "../hardware_map.hpp"
#include <libhal-neo/neo.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

hal::status application(hardware_map& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock;
  auto& console = *p_map.console;
  auto& gps = *p_map.gps;

  hal::print(console, "Initializing GPS...\n");
  auto neoGPS = HAL_CHECK(hal::neo::nmea_parser::create(gps));
  auto gga_parser = HAL_CHECK(hal::neo::GGA_Sentence::create());
  auto gsa_parser = HAL_CHECK(hal::neo::GSA_Sentence::create());
  auto gsv_parser = HAL_CHECK(hal::neo::GSV_Sentence::create());
  auto rmc_parser = HAL_CHECK(hal::neo::RMC_Sentence::create());

  hal::print(console, "GPS created! \n");
  hal::print(
    console,
    "***You may need to wait a few minutes before having a full GPS lock***\n");

  while (true) {
    hal::delay(clock, 1000ms);
    auto nmea_reader = neoGPS.read();
    auto gga_data = gga_parser.read(nmea_reader.value());
    auto gsa_data = gsa_parser.read(nmea_reader.value());
    auto gsv_data = gsv_parser.read(nmea_reader.value());
    auto rmc_data = rmc_parser.read(nmea_reader.value());

    auto GGA = gga_data.value();
    auto GSA = gsa_data.value();
    auto GSV = gsv_data.value();
    auto RMC = rmc_data.value();
    hal::print(
      console,
      "\n=================== GPS Coordinate Data ===================\n");
    hal::print<128>(console,
                    "Time: %f\nLatitude: %f\nLongitude: %f\nNumber of "
                    "satellites seen: %d\nAltitude: %f meters",
                    GGA.time,
                    GGA.latitude,
                    GGA.longitude,
                    GGA.satellites_used,
                    GGA.altitude);

    hal::print(console,
               "\n=================== GPS Status Data ===================\n");
    hal::print<128>(console,
                    "Fix type: %d\nFix mode: %c\nPDOP: %f\nHDOP: %f\nVDOP: "
                    "%f\n",
                    GSA.fix_type,
                    GSA.mode,
                    GSA.pdop,
                    GSA.hdop,
                    GSA.vdop);

    hal::print(
      console,
      "\n=================== GPS Satellite Data ===================\n");
    hal::print<128>(console,
                    "Satellites in view: %d\nElevation: %d\nAzimuth: %d\nSNR: "
                    "%d\n",
                    GSV.satellites_in_view,
                    GSV.elevation,
                    GSV.azimuth,
                    GSV.snr);

    hal::print(console,
               "\n=================== GPS Speed Data ===================\n");
    hal::print<128>(console,
                    "Speed: %f\nTrack angle: %f\nMagnetic direction: %c\n",
                    RMC.speed,
                    RMC.track_angle,
                    RMC.magnetic_direction);
  }

  return hal::success();
}
