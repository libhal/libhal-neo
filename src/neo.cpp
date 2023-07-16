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
{
}

result<neo_GPS> neo_GPS::create(hal::serial& p_serial)
{
  neo_GPS new_neo(p_serial);
  return new_neo;
}

hal::result<serial::read_t> neo_GPS::read_raw_gps()
{
    std::array<hal::byte, gps_buffer_size> read_buffer;
    hal::result<serial::read_t> gps_data = m_serial->read(read_buffer);
  return gps_data;
}

}  // namespace hal::neo