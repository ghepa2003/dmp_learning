#pragma once

// Shared gate for the long, empirical probe tests (no assertions on values; they only print
// measurements). They are skipped unless GRASP_RUN_PROBES is set to something other than "" or "0".

#include <cstdlib>
#include <string>

#include <gtest/gtest.h>

namespace probe_gate {

/// Single place with the rule: true iff GRASP_RUN_PROBES is set and is neither "" nor "0".
inline bool probesEnabled() {
    const char* v = std::getenv("GRASP_RUN_PROBES");
    return v != nullptr && std::string(v) != "" && std::string(v) != "0";
}

}  // namespace probe_gate

/// First statement of every probe test body (GTEST_SKIP needs a void function, hence a macro).
#define SKIP_UNLESS_PROBES_ENABLED()                                                     \
    do {                                                                                 \
        if (!::probe_gate::probesEnabled()) {                                            \
            GTEST_SKIP() << "probe lungo: imposta GRASP_RUN_PROBES=1 per eseguirlo";     \
        }                                                                                \
    } while (0)
