#include "haptic_dmp_learning/core/demo_params.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace haptic_dmp_learning {
namespace core {
namespace demo_params {

namespace {

// ---------------------------------------------------------------- SHA-256 (self-contained)

constexpr std::uint32_t kK[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

inline std::uint32_t rotr(std::uint32_t x, unsigned n) { return (x >> n) | (x << (32 - n)); }

void sha256Block(std::uint32_t h[8], const unsigned char* p) {
    std::uint32_t w[64];
    for (int i = 0; i < 16; ++i) {
        w[i] = (static_cast<std::uint32_t>(p[4 * i]) << 24) | (static_cast<std::uint32_t>(p[4 * i + 1]) << 16) |
               (static_cast<std::uint32_t>(p[4 * i + 2]) << 8) | static_cast<std::uint32_t>(p[4 * i + 3]);
    }
    for (int i = 16; i < 64; ++i) {
        const std::uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
        const std::uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    std::uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
    for (int i = 0; i < 64; ++i) {
        const std::uint32_t S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
        const std::uint32_t ch = (e & f) ^ (~e & g);
        const std::uint32_t t1 = hh + S1 + ch + kK[i] + w[i];
        const std::uint32_t S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
        const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        const std::uint32_t t2 = S0 + maj;
        hh = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
}

// ---------------------------------------------------------------- writing helpers

std::string num(double v) {
    std::ostringstream o;
    o << std::setprecision(17) << v;
    return o.str();
}

std::string vec(const Eigen::Vector3d& v) {
    return "[" + num(v.x()) + ", " + num(v.y()) + ", " + num(v.z()) + "]";
}

std::string joints(const std::array<double, 7>& q) {
    std::string s = "[";
    for (std::size_t i = 0; i < q.size(); ++i) s += (i ? ", " : "") + num(q[i]);
    return s + "]";
}

const char* graspPointName(GraspPointId k) {
    switch (k) {
        case GraspPointId::kP0: return "kP0";
        case GraspPointId::kP90: return "kP90";
        case GraspPointId::kP180: return "kP180";
        default: return "kP270";
    }
}

// ---------------------------------------------------------------- reading helpers

[[noreturn]] void fail(const std::string& what, const std::string& path) {
    throw std::runtime_error("demo_params::read(" + path + "): " + what);
}

/// The child must be DEFINED (null counts as defined); absent keys are errors naming the key.
YAML::Node need(const YAML::Node& parent, const std::string& key, const std::string& full,
                const std::string& path) {
    if (!parent.IsMap()) fail("expected a map while looking for key '" + full + "'", path);
    const YAML::Node n = parent[key];
    if (!n.IsDefined()) fail("missing key '" + full + "'", path);
    return n;
}

double asDouble(const YAML::Node& n, const std::string& full, const std::string& path) {
    if (!n.IsScalar()) fail("invalid value for '" + full + "' (expected a number)", path);
    double v;
    try {
        v = n.as<double>();
    } catch (const YAML::Exception&) {
        fail("invalid value for '" + full + "' (not a number)", path);
    }
    if (!std::isfinite(v)) fail("non-finite value for '" + full + "'", path);
    return v;
}

std::vector<double> asDoubles(const YAML::Node& n, std::size_t count, const std::string& full,
                              const std::string& path) {
    if (!n.IsSequence() || n.size() != count) {
        fail("'" + full + "' must be a sequence of exactly " + std::to_string(count) + " values", path);
    }
    std::vector<double> out;
    for (std::size_t i = 0; i < n.size(); ++i) {
        out.push_back(asDouble(n[i], full + "[" + std::to_string(i) + "]", path));
    }
    return out;
}

Eigen::Vector3d asVec3(const YAML::Node& n, const std::string& full, const std::string& path) {
    const auto v = asDoubles(n, 3, full, path);
    return Eigen::Vector3d(v[0], v[1], v[2]);
}

std::array<double, 7> asJoints(const YAML::Node& n, const std::string& full, const std::string& path) {
    const auto v = asDoubles(n, 7, full, path);
    std::array<double, 7> q{};
    for (std::size_t i = 0; i < 7; ++i) q[i] = v[i];
    return q;
}

}  // namespace

std::string sha256Hex(const std::string& bytes) {
    std::uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                          0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    std::string msg = bytes;
    const std::uint64_t bit_len = static_cast<std::uint64_t>(bytes.size()) * 8ull;
    msg.push_back(static_cast<char>(0x80));
    while (msg.size() % 64 != 56) msg.push_back('\0');
    for (int i = 7; i >= 0; --i) msg.push_back(static_cast<char>((bit_len >> (8 * i)) & 0xff));
    for (std::size_t off = 0; off < msg.size(); off += 64) {
        sha256Block(h, reinterpret_cast<const unsigned char*>(msg.data()) + off);
    }
    std::ostringstream o;
    o << std::hex << std::setfill('0');
    for (int i = 0; i < 8; ++i) o << std::setw(8) << h[i];
    return o.str();
}

std::string sha256FileHex(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) throw std::runtime_error("demo_params::sha256FileHex: cannot open " + path);
    std::ostringstream buf;
    buf << f.rdbuf();
    return sha256Hex(buf.str());
}

void write(const std::string& path, const DemoParams& p) {
    std::ofstream out(path);
    if (!out.is_open()) throw std::runtime_error("demo_params::write: cannot open file for writing: " + path);
    out << "format_version: " << p.format_version << "\n";
    out << "weights_file: " << p.weights_file << "\n";
    out << "weights_sha256: " << p.weights_sha256 << "\n";
    out << "demo:\n";
    out << "  tau_s: " << num(p.tau_s) << "\n";
    out << "  t_contact_s: " << (p.t_contact_s ? num(*p.t_contact_s) : "null") << "\n";
    out << "  delta_g_demo_m: " << vec(p.delta_g_demo_m) << "\n";
    out << "  contact_to_end_offset_m: " << vec(p.contact_to_end_offset_m) << "\n";
    out << "  start_position_demo_frame_m: " << vec(p.start_position_demo_frame_m) << "\n";
    out << "  start_joints_rad: " << (p.start_joints_rad ? joints(*p.start_joints_rad) : "null") << "\n";
    out << "  end_joints_rad: " << (p.end_joints_rad ? joints(*p.end_joints_rad) : "null") << "\n";
    if (p.satellite_at_demo) {
        const SatelliteAtDemo& s = *p.satellite_at_demo;
        out << "satellite_at_demo:\n";
        out << "  center_m: " << vec(s.center_m) << "\n";
        out << "  axis: " << vec(s.axis) << "\n";
        out << "  omega_rad_s: " << num(s.omega_rad_s) << "\n";
        out << "  grasp_point: " << graspPointName(s.grasp_point) << "\n";
        out << "  phase_at_contact_rad: " << num(s.phase_at_contact_rad) << "\n";
    } else {
        out << "satellite_at_demo: null\n";
    }
    out << "fit:\n";
    out << "  num_basis: " << p.fit.num_basis << "\n";
    out << "  ridge_lambda: " << num(p.fit.ridge_lambda) << "\n";
    out << "  position_filter_window_s: " << num(p.fit.position_filter_window_s) << "\n";
    out << "  fix_goal_to_demo_endpoint: " << (p.fit.fix_goal_to_demo_endpoint ? "true" : "false") << "\n";
}

DemoParams read(const std::string& path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        fail(std::string("cannot load file: ") + e.what(), path);
    }
    DemoParams p;

    const YAML::Node ver = need(root, "format_version", "format_version", path);
    try {
        p.format_version = ver.as<int>();
    } catch (const YAML::Exception&) {
        fail("invalid value for 'format_version'", path);
    }
    if (p.format_version != 1) {
        fail("unsupported 'format_version' " + std::to_string(p.format_version) + " (expected 1)", path);
    }

    const YAML::Node wf = need(root, "weights_file", "weights_file", path);
    if (!wf.IsScalar() || wf.Scalar().empty()) fail("invalid value for 'weights_file'", path);
    p.weights_file = wf.Scalar();

    const YAML::Node wh = need(root, "weights_sha256", "weights_sha256", path);
    if (!wh.IsScalar()) fail("invalid value for 'weights_sha256'", path);
    p.weights_sha256 = wh.Scalar();
    if (p.weights_sha256.size() != 64) fail("'weights_sha256' must have 64 characters", path);
    for (char c : p.weights_sha256) {
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) {
            fail("'weights_sha256' must be lowercase hexadecimal", path);
        }
    }

    const YAML::Node demo = need(root, "demo", "demo", path);
    p.tau_s = asDouble(need(demo, "tau_s", "demo.tau_s", path), "demo.tau_s", path);
    if (!(p.tau_s > 0.0)) fail("'demo.tau_s' must be > 0", path);
    const YAML::Node tc = need(demo, "t_contact_s", "demo.t_contact_s", path);
    if (!tc.IsNull()) {
        const double t = asDouble(tc, "demo.t_contact_s", path);
        if (t < 0.0 || t > p.tau_s) fail("'demo.t_contact_s' must be within [0, tau_s]", path);
        p.t_contact_s = t;
    }
    p.delta_g_demo_m = asVec3(need(demo, "delta_g_demo_m", "demo.delta_g_demo_m", path), "demo.delta_g_demo_m", path);
    p.contact_to_end_offset_m = asVec3(need(demo, "contact_to_end_offset_m", "demo.contact_to_end_offset_m", path),
                                       "demo.contact_to_end_offset_m", path);
    p.start_position_demo_frame_m =
        asVec3(need(demo, "start_position_demo_frame_m", "demo.start_position_demo_frame_m", path),
               "demo.start_position_demo_frame_m", path);
    const YAML::Node sj = need(demo, "start_joints_rad", "demo.start_joints_rad", path);
    if (!sj.IsNull()) p.start_joints_rad = asJoints(sj, "demo.start_joints_rad", path);
    const YAML::Node ej = need(demo, "end_joints_rad", "demo.end_joints_rad", path);
    if (!ej.IsNull()) p.end_joints_rad = asJoints(ej, "demo.end_joints_rad", path);

    const YAML::Node sat = need(root, "satellite_at_demo", "satellite_at_demo", path);
    if (!sat.IsNull()) {
        SatelliteAtDemo s;
        s.center_m = asVec3(need(sat, "center_m", "satellite_at_demo.center_m", path), "satellite_at_demo.center_m", path);
        s.axis = asVec3(need(sat, "axis", "satellite_at_demo.axis", path), "satellite_at_demo.axis", path);
        if (!(s.axis.norm() > 1e-9)) fail("'satellite_at_demo.axis' has ~zero norm", path);
        s.omega_rad_s = asDouble(need(sat, "omega_rad_s", "satellite_at_demo.omega_rad_s", path),
                                 "satellite_at_demo.omega_rad_s", path);
        if (s.omega_rad_s == 0.0) fail("'satellite_at_demo.omega_rad_s' must be != 0", path);
        const YAML::Node gp = need(sat, "grasp_point", "satellite_at_demo.grasp_point", path);
        const std::string name = gp.IsScalar() ? gp.Scalar() : "";
        if (name == "kP0") s.grasp_point = GraspPointId::kP0;
        else if (name == "kP90") s.grasp_point = GraspPointId::kP90;
        else if (name == "kP180") s.grasp_point = GraspPointId::kP180;
        else if (name == "kP270") s.grasp_point = GraspPointId::kP270;
        else fail("invalid 'satellite_at_demo.grasp_point' '" + name + "' (expected kP0/kP90/kP180/kP270)", path);
        s.phase_at_contact_rad = asDouble(need(sat, "phase_at_contact_rad", "satellite_at_demo.phase_at_contact_rad", path),
                                          "satellite_at_demo.phase_at_contact_rad", path);
        p.satellite_at_demo = s;
    }

    const YAML::Node fit = need(root, "fit", "fit", path);
    const YAML::Node nb = need(fit, "num_basis", "fit.num_basis", path);
    try {
        p.fit.num_basis = nb.as<int>();
    } catch (const YAML::Exception&) {
        fail("invalid value for 'fit.num_basis'", path);
    }
    if (p.fit.num_basis <= 0) fail("'fit.num_basis' must be > 0", path);
    p.fit.ridge_lambda = asDouble(need(fit, "ridge_lambda", "fit.ridge_lambda", path), "fit.ridge_lambda", path);
    p.fit.position_filter_window_s = asDouble(
        need(fit, "position_filter_window_s", "fit.position_filter_window_s", path),
        "fit.position_filter_window_s", path);
    const YAML::Node fg = need(fit, "fix_goal_to_demo_endpoint", "fit.fix_goal_to_demo_endpoint", path);
    try {
        p.fit.fix_goal_to_demo_endpoint = fg.as<bool>();
    } catch (const YAML::Exception&) {
        fail("invalid value for 'fit.fix_goal_to_demo_endpoint'", path);
    }
    return p;
}

DemoParams buildFromDemo(const std::vector<Sample>& demo,
                         const std::optional<std::vector<joint_states_csv_io::JointStateSample>>& joint_states,
                         const FitInfo& fit, const std::string& weights_path,
                         const std::optional<SatelliteAtDemo>& satellite) {
    if (demo.size() < 2) throw std::invalid_argument("demo_params::buildFromDemo: need at least 2 samples.");
    for (std::size_t i = 1; i < demo.size(); ++i) {
        if (!(demo[i].t > demo[i - 1].t)) {
            throw NonIncreasingTimestampsError("demo_params::buildFromDemo: non-increasing timestamps at sample " +
                                               std::to_string(i));
        }
    }
    DemoParams p;
    const Sample& first = demo.front();
    const Sample& last = demo.back();
    p.tau_s = last.t - first.t;
    p.delta_g_demo_m = last.position - first.position;
    p.start_position_demo_frame_m = first.position;
    p.contact_to_end_offset_m = Eigen::Vector3d::Zero();  // no trigger: contact assumed at the end
    for (const Sample& s : demo) {
        if (s.gripper_trigger) {
            p.t_contact_s = s.t - first.t;
            p.contact_to_end_offset_m = last.position - s.position;
            break;
        }
    }
    if (joint_states) {
        if (joint_states->empty()) {
            throw std::invalid_argument("demo_params::buildFromDemo: joint_states is empty.");
        }
        p.start_joints_rad = joint_states->front().q;
        p.end_joints_rad = joint_states->back().q;
    }
    const std::size_t slash = weights_path.find_last_of('/');
    p.weights_file = (slash == std::string::npos) ? weights_path : weights_path.substr(slash + 1);
    p.weights_sha256 = sha256FileHex(weights_path);
    p.satellite_at_demo = satellite;
    p.fit = fit;
    return p;
}

GraspPointId parseGraspPoint(const std::string& name) {
    if (name == "kP0") return GraspPointId::kP0;
    if (name == "kP90") return GraspPointId::kP90;
    if (name == "kP180") return GraspPointId::kP180;
    if (name == "kP270") return GraspPointId::kP270;
    throw std::invalid_argument("grasp point must be one of kP0, kP90, kP180, kP270 (got '" + name + "')");
}

std::string writeForWeights(const std::string& demo_csv_path, const std::vector<Sample>& demo,
                            const FitInfo& fit, const std::string& weights_path,
                            const std::optional<SatelliteAtDemo>& satellite, std::ostream& log,
                            DemoParams* out) {
    std::optional<std::vector<joint_states_csv_io::JointStateSample>> joints;
    const std::string joints_path = joint_states_csv_io::deriveJointStatesCsvPath(demo_csv_path);
    if (std::ifstream(joints_path).good()) {
        joints = joint_states_csv_io::readJointStatesCsv(joints_path);
        log << "  demo_params: joint states from " << joints_path << " (" << joints->size() << " rows)\n";
    } else {
        log << "  demo_params: no joint-states CSV at " << joints_path << " -> start/end joints = null\n";
    }
    const DemoParams params = buildFromDemo(demo, joints, fit, weights_path, satellite);
    const std::string path = deriveDemoParamsPath(weights_path);
    write(path, params);
    try {
        const DemoParams back_check = read(path);  // fail loud if what we wrote does not validate
        verifyWeightsAlignment(path, back_check);
    } catch (...) {
        std::remove(path.c_str());  // never leave a demo-params file that does not validate
        throw;
    }
    const DemoParams back = read(path);
    log << "  demo_params: wrote " << path << " (tau_s=" << params.tau_s << ", t_contact_s="
        << (params.t_contact_s ? std::to_string(*params.t_contact_s) : std::string("null"))
        << ", |delta_g_demo|=" << params.delta_g_demo_m.norm() << " m, sha256=" << params.weights_sha256
        << ")\n";
    if (out) *out = back;
    return path;
}

void verifyWeightsAlignment(const std::string& demo_params_path, const DemoParams& params) {
    const std::size_t slash = demo_params_path.find_last_of('/');
    const std::string dir = (slash == std::string::npos) ? std::string() : demo_params_path.substr(0, slash + 1);
    const std::string weights_path = dir + params.weights_file;
    std::ifstream probe(weights_path, std::ios::binary);
    if (!probe.is_open()) {
        throw std::runtime_error("demo_params::verifyWeightsAlignment: weights file not found: " + weights_path);
    }
    probe.close();
    const std::string actual = sha256FileHex(weights_path);
    if (actual != params.weights_sha256) {
        throw std::runtime_error("demo_params::verifyWeightsAlignment: hash mismatch for " + weights_path +
                                 ": file has " + actual + ", demo params expect " + params.weights_sha256);
    }
}

std::string deriveDemoParamsPath(const std::string& weights_path) {
    const std::size_t slash = weights_path.find_last_of('/');
    const std::string dir = (slash == std::string::npos) ? std::string() : weights_path.substr(0, slash + 1);
    const std::string file = (slash == std::string::npos) ? weights_path : weights_path.substr(slash + 1);
    const std::size_t dot = file.find_last_of('.');
    const std::string stem = (dot == std::string::npos || dot == 0) ? file : file.substr(0, dot);
    return dir + stem + "_demo_params.yaml";
}

}  // namespace demo_params
}  // namespace core
}  // namespace haptic_dmp_learning
