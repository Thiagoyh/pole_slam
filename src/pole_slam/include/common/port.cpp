#include "port.h"

namespace config_lua {

namespace common {

// c++11: std::lround 返回最接近x的long int整数 eg: lround(15.2) -> 15, lround(15.8) -> 16
// 返回最接近x的整数
int RoundToInt(const float x) { return std::lround(x); }

int RoundToInt(const double x) { return std::lround(x); }

int64 RoundToInt64(const float x) { return std::lround(x); }

int64 RoundToInt64(const double x) { return std::lround(x); }

/**
 * @brief 将字符串进行压缩
 *
 * @param[in] uncompressed 压缩前的string
 * @param[out] compressed 压缩后的string
 */
void FastGzipString(const std::string& uncompressed,
                           std::string* compressed) {
  boost::iostreams::filtering_ostream out;
  // 按gzip压缩
  out.push(
      boost::iostreams::gzip_compressor(boost::iostreams::zlib::best_speed));
  out.push(boost::iostreams::back_inserter(*compressed));
  boost::iostreams::write(out,
                          reinterpret_cast<const char*>(uncompressed.data()),
                          uncompressed.size());
}

/**
 * @brief 将字符串进行解压
 *
 * @param[in] compressed 压缩的string
 * @param[out] decompressed 解压后的string
 */
void FastGunzipString(const std::string& compressed,
                             std::string* decompressed) {
  boost::iostreams::filtering_ostream out;
  // 按gzip解压
  out.push(boost::iostreams::gzip_decompressor());
  out.push(boost::iostreams::back_inserter(*decompressed));
  boost::iostreams::write(out, reinterpret_cast<const char*>(compressed.data()),
                          compressed.size());
}

}
}