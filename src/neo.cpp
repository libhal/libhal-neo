#include <libhal-neo/neo.hpp>

#include <algorithm>
#include <array>
#include <span>

#include <libhal-util/serial.hpp>
#include <libhal-util/streams.hpp>
#include <libhal-util/timeout.hpp>

// #include "util.hpp"

namespace hal::neo {

// constructor
neo_GPS::neo_GPS(hal::serial& p_serial)
  : m_serial(&p_serial)
  , m_packet_manager{}
{
}

result<neo_GPS> neo_GPS::create(hal::serial& p_serial)
{
  neo_GPS new_neo(p_serial);
  return new_neo;
}

namespace _packet_manager {
enum state : std::uint8_t
{
  expect_plus,
  expect_i,
  expect_p,
  expect_d,
  expect_comma,
  expect_digit1,
  expect_digit2,
  expect_digit3,
  expect_digit4,
  expect_colon,
  header_complete
};
}

neo_GPS::packet_manager::packet_manager()
  : m_state(_packet_manager::state::expect_plus)
  , m_length(0)
{
}

void neo_GPS::packet_manager::find(hal::serial& p_serial)
{
  if (is_complete_header()) {
    return;
  }

  std::array<hal::byte, 1> byte;
  auto result = p_serial.read(byte);
  while (result.has_value() && result.value().data.size() != 0) {
    update_state(byte[0]);
    if (is_complete_header()) {
      return;
    }
    result = p_serial.read(byte);
  }
}

void neo_GPS::packet_manager::set_state(std::uint8_t p_state)
{
  m_state = p_state;
}

void neo_GPS::packet_manager::update_state(hal::byte p_byte)
{
  char c = static_cast<char>(p_byte);
  switch (m_state) {
    case _packet_manager::state::expect_plus:
      if (c == '+') {
        m_state = _packet_manager::state::expect_i;
      }
      break;
    case _packet_manager::state::expect_i:
      if (c == 'I') {
        m_state = _packet_manager::state::expect_p;
      } else {
        m_state = _packet_manager::state::expect_plus;
      }
      break;
    case _packet_manager::state::expect_p:
      if (c == 'P') {
        m_state = _packet_manager::state::expect_d;
      } else {
        m_state = _packet_manager::state::expect_plus;
      }
      break;
    case _packet_manager::state::expect_d:
      if (c == 'D') {
        m_state = _packet_manager::state::expect_comma;
      } else {
        m_state = _packet_manager::state::expect_plus;
      }
      break;
    case _packet_manager::state::expect_comma:
      if (c == ',') {
        m_state = _packet_manager::state::expect_digit1;
        m_length = 0;  // Reset the length because we're about to parse it
      } else {
        m_state = _packet_manager::state::expect_plus;
      }
      break;
    case _packet_manager::state::expect_digit1:
    case _packet_manager::state::expect_digit2:
    case _packet_manager::state::expect_digit3:
    case _packet_manager::state::expect_digit4:
      if (isdigit(c)) {
        m_length = m_length * 10 + (c - '0');  // Accumulate the length
        m_state += 1;
      } else if (c == ':') {
        m_state = _packet_manager::state::header_complete;
      } else {
        // It's not a digit or a ':', so this is an error
        m_state = _packet_manager::state::expect_plus;
      }
      break;
    case _packet_manager::state::expect_colon:
      if (c == ':') {
        m_state = _packet_manager::state::header_complete;
      } else {
        m_state = _packet_manager::state::expect_plus;
      }
      break;
    default:
      m_state = _packet_manager::state::expect_plus;
  }
}

bool neo_GPS::packet_manager::is_complete_header()
{
  return m_state == _packet_manager::state::header_complete;
}

std::uint16_t neo_GPS::packet_manager::packet_length()
{
  return is_complete_header() ? m_length : 0;
}


 void neo_GPS::packet_manager::reset()
{
  m_state = _packet_manager::state::expect_plus;
  m_length = 0;
}


hal::result<std::span<hal::byte>> neo_GPS::packet_manager::read_packet(
  hal::serial& p_serial,
  std::span<hal::byte> p_buffer)
{
  if (!is_complete_header()) {
    return p_buffer.first(0);
  }

  auto buffer_size = static_cast<std::uint16_t>(p_buffer.size());
  auto bytes_capable_of_reading = std::min(m_length, buffer_size);
  auto subspan = p_buffer.first(bytes_capable_of_reading);
  auto bytes_read_array = HAL_CHECK(p_serial.read(subspan)).data;

  m_length = m_length - bytes_read_array.size();

  if (m_length == 0) {
    reset();
  }

  return bytes_read_array;
}


hal::result<neo_GPS::read_t> neo_GPS::read_gps(std::span<hal::byte> p_buffer)
{
  size_t bytes_read = 0;
  auto buffer = p_buffer;
  auto read = std::span<hal::byte>();

  do {
    m_packet_manager.find(*m_serial);
    read = HAL_CHECK(m_packet_manager.read_packet(*m_serial, buffer));
    bytes_read += read.size();
    buffer = buffer.subspan(read.size());
  } while (read.size() != 0 && buffer.size() != 0);

  return read_t{ .data = p_buffer.first(bytes_read) };
}

}  // namespace hal::neo