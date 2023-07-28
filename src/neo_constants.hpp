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
const char* GSA_FORMAT = ",%c,%d,%d,%d,%d,%d,%d,%d,%d,%d,,,,,,,,%f,%f,%f,*";
const char* GSV_FORMAT = ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,*";
const char* RMC_FORMAT = ",%f,%c,%f,%c,%f,%c,%f,%f,%d,,,,%c*";

}  // namespace hal::neo