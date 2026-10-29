#pragma once
#include <string>
namespace ec {
// gzip (RFC 1952) or zlib (RFC 1950) or raw deflate data -> bytes. False when it is not valid.
bool inflate_any(const std::string &in, std::string &out);
}
