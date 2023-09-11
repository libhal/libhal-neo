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

namespace hal::neo {

using namespace std::literals;

constexpr auto gga_start_of_line = "GGA,"sv;
constexpr auto gsa_start_of_line = "GSA,"sv;
constexpr auto gsv_start_of_line = "GSV,"sv;
constexpr auto rmc_start_of_line = "RMC,"sv;

constexpr auto end_of_line = "\r\n"sv;

const char* GGA_FORMAT = ",%f,%f,%c,%f,%c,%d,%d,%f,%f,%c,%f,%c,,%s,";
// $GPGGA,175323.00,4332.45922,N,00127.35291,E,1,05,3.85,276.8,M,48.6,M,,*51

const char* GSA_FORMAT = ",%c,%d,%d,%d,%d,%d,%d,%d,%d,%d,,,,,,,,,%f,%f,%f*";
// $GPGSA,A,3,05,11,30,13,09,06,20,,,,,,,2.08,1.12,1.75*00

const char* GSV_FORMAT = ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,*";
// $GPGSV,3,1,12,04,03,076,,05,32,309,37,06,23,195,42,07,65,076,23*7C

const char* RMC_FORMAT = ",%f,%c,%f,%c,%f,%c,%f,,%d,,,%c";
// ,173808.00,A,4332.46125,N,00127.36287,E,0.110,,010823,,,A*73
// ,175323.00,A,4332.45922,N,00127.35291,E,0.116,,010823,,,A*79

}  // namespace hal::neo