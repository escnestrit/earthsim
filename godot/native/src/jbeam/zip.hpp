// Zip archives (stored / deflate, zip64) read from disk; BeamNG vehicles come as zips.
#pragma once
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace ec {

struct ZipEntry {
    std::string name;
    uint16_t method = 0;
    uint32_t crc = 0;
    uint64_t csize = 0, size = 0, offset = 0;
};

class Zip {
public:
    ~Zip();
    bool open(const std::string &path, std::string &err);
    const ZipEntry *find(const std::string &name) const;
    bool read(const ZipEntry &e, std::string &out, std::string &err);
    // first n bytes of a stored entry (whole entry when compressed)
    bool read_prefix(const ZipEntry &e, uint64_t n, std::string &out, std::string &err);
    std::vector<ZipEntry> entries;
    std::string path;

private:
    FILE *f_ = nullptr;
};

uint32_t crc32(const std::string &data);

}  // namespace ec
