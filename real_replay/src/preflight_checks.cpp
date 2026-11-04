#include "real_replay/preflight_checks.hpp"

#include <cmath>
#include <cstdio>
#include <regex>
#include <stdexcept>

namespace real_replay {

MeasuredGravityResult checkGravityVsMeasured(const Array7& g_franka, const Array7& tau_j, const Array7& dq,
                                             double tol_nm, double still_max_dq_rad_s) {
  MeasuredGravityResult r;
  for (std::size_t i = 0; i < 7; ++i) {
    const double d = std::fabs(g_franka[i] - tau_j[i]);
    if (d > r.max_abs_diff) {
      r.max_abs_diff = d;
      r.worst_joint = static_cast<int>(i);
    }
    r.max_abs_dq = std::max(r.max_abs_dq, std::fabs(dq[i]));
  }
  r.moving = r.max_abs_dq > still_max_dq_rad_s;
  if (r.max_abs_diff <= tol_nm) {
    r.verdict = MeasuredGravityResult::Verdict::kPass;
  } else {
    r.verdict = r.moving ? MeasuredGravityResult::Verdict::kWarnMoving : MeasuredGravityResult::Verdict::kFail;
  }
  return r;
}

UrdfGravityResult checkGravityVsUrdf(const Array7& g_franka, const Array7& g_urdf, double warn_nm) {
  UrdfGravityResult r;
  for (std::size_t i = 0; i < 7; ++i) {
    const double d = std::fabs(g_franka[i] - g_urdf[i]);
    if (d > r.max_abs_diff) {
      r.max_abs_diff = d;
      r.worst_joint = static_cast<int>(i);
    }
  }
  r.warn = r.max_abs_diff > warn_nm;
  return r;
}

namespace {
std::string num(double v) {
  char b[40];
  std::snprintf(b, sizeof(b), "%.17g", v);
  return b;
}

// Multiplies the numeric attributes `attrs` of the first element `<tag ... />` found in `block`.
void scaleAttrs(std::string& block, const std::string& tag, const std::vector<std::string>& attrs, double f,
                const std::string& link) {
  const std::regex el("<" + tag + "\\b[^>]*>");
  std::smatch m;
  if (!std::regex_search(block, m, el)) throw std::runtime_error("scaleLinkMasses: no <" + tag + "> in link " + link);
  std::string e = m.str();
  for (const std::string& a : attrs) {
    const std::regex at("(\\b" + a + "=\")([^\"]+)(\")");
    std::smatch am;
    if (!std::regex_search(e, am, at)) throw std::runtime_error("scaleLinkMasses: no " + a + " in <" + tag + "> of " + link);
    const double v = std::stod(am[2].str());
    e.replace(static_cast<std::size_t>(am.position(0)), static_cast<std::size_t>(am.length(0)),
              am[1].str() + num(v * f) + am[3].str());
  }
  block.replace(static_cast<std::size_t>(m.position(0)), static_cast<std::size_t>(m.length(0)), e);
}
}  // namespace

std::string scaleLinkMasses(const std::string& urdf_xml, const std::vector<std::string>& links, double factor) {
  std::string out = urdf_xml;
  for (const std::string& link : links) {
    const std::string open = "<link name=\"" + link + "\">";
    const std::size_t b = out.find(open);
    if (b == std::string::npos) throw std::runtime_error("scaleLinkMasses: link '" + link + "' not found");
    const std::size_t e = out.find("</link>", b);
    if (e == std::string::npos) throw std::runtime_error("scaleLinkMasses: unterminated link '" + link + "'");
    std::string block = out.substr(b, e - b);
    scaleAttrs(block, "mass", {"value"}, factor, link);
    scaleAttrs(block, "inertia", {"ixx", "ixy", "ixz", "iyy", "iyz", "izz"}, factor, link);
    out.replace(b, e - b, block);
  }
  return out;
}

}  // namespace real_replay
