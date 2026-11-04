#pragma once

// Minimal SHA-256 (FIPS 180-4), self-contained: no OpenSSL dependency.

#include <string>

namespace real_replay {

/// Lower-case hex digest of `data`.
std::string sha256Hex(const std::string& data);

/// Lower-case hex digest of the file's bytes. Throws std::runtime_error if it cannot be read.
std::string sha256FileHex(const std::string& path);

}  // namespace real_replay
