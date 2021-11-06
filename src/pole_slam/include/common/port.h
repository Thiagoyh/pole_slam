/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <cinttypes>
#include <cmath>
#include <string>

namespace config_lua {

using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;
using uint8 = uint8_t;
using uint16 = uint16_t;  // 2个字节的无符号整型
using uint32 = uint32_t;
using uint64 = uint64_t;  // 8个字节的无符号整型

namespace common {

// c++11: std::lround 返回最接近x的long int整数 eg: lround(15.2) -> 15, lround(15.8) -> 16
// 返回最接近x的整数
int RoundToInt(const float x);

int RoundToInt(const double x);

int64 RoundToInt64(const float x);

int64 RoundToInt64(const double x);

/**
 * @brief 将字符串进行压缩
 *
 * @param[in] uncompressed 压缩前的string
 * @param[out] compressed 压缩后的string
 */
void FastGzipString(const std::string &uncompressed,
                           std::string *compressed);

/**
 * @brief 将字符串进行解压
 *
 * @param[in] compressed 压缩的string
 * @param[out] decompressed 解压后的string
 */
void FastGunzipString(const std::string &compressed,
                             std::string *decompressed);

}
}

