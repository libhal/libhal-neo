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
  struct read_t
  {
    // The buffer containing the bytes read from the server
    std::span<hal::byte> data;
  };

  struct write_t
  {
    // The buffer that was written to the server
    std::span<const hal::byte> data;
  };

  [[nodiscard]] static result<neo_GPS> create(hal::serial& p_serial);

  hal::result<neo_GPS::read_t> read_gps(std::span<hal::byte> p_buffer);

private:
  class packet_manager
  {
  public:
    packet_manager();
    void find(hal::serial& p_serial);
    bool is_complete_header();
    std::uint16_t packet_length();
    hal::result<std::span<hal::byte>> read_packet(
      hal::serial& p_serial,
      std::span<hal::byte> p_buffer);
    void reset();
    void set_state(std::uint8_t p_state);

  private:
    void update_state(hal::byte p_byte);
    std::uint8_t m_state;
    std::uint16_t m_length;
  };

  neo_GPS(hal::serial& p_serial);
  hal::serial* m_serial;
  packet_manager m_packet_manager;
};
}  // namespace hal::neo