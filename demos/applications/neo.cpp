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
  using namespace std::literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock;
  auto& console = *p_map.console;
  auto& gps = *p_map.gps;

  hal::neo::GGA_Sentence gga_sentence;
  hal::neo::GSA_Sentence gsa_sentence;
  hal::neo::GSV_Sentence gsv_sentence;
  hal::neo::RMC_Sentence rmc_sentence;

  std::array<hal::neo::nmea_parser*, 4> parsers = {
    &rmc_sentence, &gsa_sentence, &gsv_sentence, &gga_sentence
  };

  hal::print(console, "Initializing GPS...\n");
  hal::print(console, "GPS created! \n");
  hal::print(
    console,
    "***You may need to wait a few minutes before having a full GPS lock***\n");

  while (true) {
    hal::delay(clock, 1000ms);


    std::array<hal::byte, 512> m_gps_buffer;
    auto bytes_read_array = HAL_CHECK(gps.read(m_gps_buffer)).data;
    auto end_token = hal::as_bytes("\r\n"sv);

    hal::print(console, "============BYTES READ============\n");
    hal::print(console, hal::as_bytes(bytes_read_array));
    hal::print(console, "\n");

    auto data = hal::neo::parse(parsers, bytes_read_array);

    hal::print(console, "==================Remaining data==================\n");
    hal::print(console, data.remaining);
    hal::print(console, "\n");






    // std::span<const hal::byte> remaining_data = data.remaining;

    // while (!remaining_data.empty()) {
    //   for (auto* parser : parsers) {
    //     if (parser->state() == hal::neo::nmea_parser::state_t::active) {
    //       remaining_data = parser->parse(remaining_data);
    //       hal::print(console, remaining_data);
    //       if (remaining_data.empty()) {
    //         parser->reset();
    //       } else {
    //         break;
    //       }
    //     }
    //   }
    // }

    auto GGA = gga_sentence.read();
    auto GSA = gsa_sentence.read();
    auto GSV = gsv_sentence.read();
    auto RMC = rmc_sentence.read();

    if (GGA.is_locked == 0) {
      hal::print(console, "No fix yet, need more satellite locks\n");
      hal::print<128>(console, "Number of satellites seen: %d\n", GGA.satellites_used);
    }else{

    hal::print(
      console,
      "\n=================== GPS Coordinate Data (GGA) ===================\n");
    hal::print<128>(console,
                    "Time: %f\nLatitude: %f\nLongitude: %f\nNumber of "
                    "satellites seen: %d\nAltitude: %f meters",
                    GGA.time,
                    GGA.latitude,
                    GGA.longitude,
                    GGA.satellites_used,
                    GGA.altitude);

    hal::print(console,
               "\n=================== GPS Status Data (GSA) ===================\n");
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
      "\n=================== GPS Satellite Data (GSV) ===================\n");
    hal::print<128>(console,
                    "Satellites in view: %d\nElevation: %d\nAzimuth: %d\nSNR: "
                    "%d\n",
                    GSV.satellites_in_view,
                    GSV.elevation,
                    GSV.azimuth,
                    GSV.snr);

    hal::print(console,
               "\n=================== GPS Speed Data (RMC) ===================\n");
    hal::print<128>(console,
                    "Time: %f\nSpeed: %f\nTrack angle: %f\nMagnetic direction: %c\n",
                    RMC.time,
                    RMC.speed,
                    RMC.track_angle,
                    RMC.magnetic_direction);

                    hal::print<128>(console, "Reading Status RMC: %d\n", RMC.reading_status);
    }
  }

  return hal::success();
}
