// Pure tests of designDamping() (no model, no robot).
//   test_damping_design <config.yaml>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <random>
#include <vector>

#include "real_replay/config.hpp"
#include "real_replay/controller_core.hpp"

using namespace real_replay;

static int g_fail = 0;
static int g_checks = 0;
#define CHECK(cond, ...)                                              \
  do {                                                                \
    ++g_checks;                                                       \
    if (!(cond)) {                                                    \
      ++g_fail;                                                       \
      std::printf("  CHECK FAILED (line %d): %s  ", __LINE__, #cond); \
      std::printf(__VA_ARGS__);                                       \
      std::printf("\n");                                              \
    }                                                                 \
  } while (0)

namespace {
/// Independent check: asymptotic decay rate of the state recursion (x, v, queue of `delay` torques),
/// normalised Lambda = dt = 1, D = a, K = b. The torque computed now is applied `delay` cycles later.
double recursionRate(double a, double b, int delay = 1) {
  std::vector<double> q(static_cast<std::size_t>(delay), 0.0);  // q[0] applied next
  q.back() = -0.7;
  double x = 1.0, v = 0.3;
  double log_sum = 0.0;
  const int n = 6000;
  for (int k = 0; k < n; ++k) {
    const double applied = q.front();
    const double fresh = -b * x - a * v;
    const double xn = x + v, vn = v + applied;
    q.erase(q.begin());
    q.push_back(fresh);
    x = xn; v = vn;
    double m2 = x * x + v * v;
    for (double e : q) m2 += e * e;
    const double m = std::sqrt(m2);
    log_sum += std::log(m);
    x /= m; v /= m;
    for (double& e : q) e /= m;
  }
  return std::exp(log_sum / n);
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: %s <config.yaml>\n", argv[0]);
    return 2;
  }
  try {
    Config cfg = loadConfig(argv[1]);
    cfg.damping.override_d.clear();
    const double dt = kNominalCycleS;
    const double cap = cfg.damping.max_damping_per_sample;
    const double scale = cfg.safety.stop_damping_scale;

    std::printf("[delayedSpectralRadius: z^3 - 2z^2 + (1+a)z + (b-a), against the 3-state recursion]\n");
    {
      struct Case { double a, b, expect, tol; };
      const Case cases[] = {{0.5, 0.167, 0.945, 0.001}, {0.47, 0.0556, 0.831, 0.001}, {0.447, 0.05, 0.840, 0.005}};
      for (const Case& c : cases) {
        const double rho = delayedSpectralRadius(c.a, c.b);
        const double rec = recursionRate(c.a, c.b);
        std::printf("  a=%.3f b=%.4f: max|z| = %.6f, recursion rate = %.6f (expected ~%.3f)\n", c.a, c.b, rho, rec, c.expect);
        CHECK(std::fabs(rho - c.expect) < c.tol, "max|z| %.6f vs %.3f", rho, c.expect);
        CHECK(std::fabs(rho - rec) < 2e-3, "roots %.6f vs recursion %.6f", rho, rec);
      }
      // random (a,b): roots against the recursion, and the same sign of (rho<1)
      std::mt19937 rng(7);
      std::uniform_real_distribution<double> ua(0.0, 1.0), ub(0.0, 0.4);
      double worst = 0.0;
      for (int n = 0; n < 200; ++n) {
        const double a = ua(rng), b = ub(rng);
        const double rho = delayedSpectralRadius(a, b);
        if (rho > 0.98) continue;  // too slow to measure by iteration
        worst = std::max(worst, std::fabs(rho - recursionRate(a, b)));
      }
      std::printf("  200 random (a,b): worst |max|z| - recursion rate| = %.3e\n", worst);
      CHECK(worst < 5e-3, "worst %.3e", worst);
      CHECK(delayedSpectralRadius(0.0, 0.0) >= 1.0 - 1e-6, "a=b=0 (no damping, no spring): marginal");
    }

    std::printf("[gain margin g*: bisection vs closed form (a-2b)/(a-b)^2; roots at g*; two cycles of delay]\n");
    {
      struct Case { double a, b, expect, tol; };
      const Case cases[] = {{0.5, 0.167, 1.50, 0.01}, {0.447, 0.05, 2.19, 0.02}};
      for (const Case& c : cases) {
        const double g = gainMargin(c.a, c.b, 1);
        const double cf = gainMarginClosedForm1(c.a, c.b);
        std::printf("  a=%.3f b=%.4f: g* bisection %.6f, closed form %.6f (expected ~%.2f)\n", c.a, c.b, g, cf, c.expect);
        CHECK(std::fabs(g - c.expect) < c.tol, "g* %.6f vs %.2f", g, c.expect);
        CHECK(std::fabs(g - cf) < 1e-6, "bisection %.9f vs closed form %.9f", g, cf);
      }
      // random: closed form == bisection for a > 2b; at g* a pair on the unit circle and a real root g*(a-b)
      std::mt19937 rng(11);
      std::uniform_real_distribution<double> ua(0.01, 0.5), ur(0.01, 0.45);
      double worst = 0.0, worst_real = 0.0;
      int used = 0;
      for (int n = 0; n < 300; ++n) {
        const double a = ua(rng), b = a * ur(rng);  // b < 0.45 a  (a > 2b)
        const double g = gainMargin(a, b, 1);
        const double cf = gainMarginClosedForm1(a, b);
        if (cf <= 1.0) {  // unstable (or marginal) already at g = 1: the margin is 0 by definition
          CHECK(g == 0.0, "a=%.4f b=%.4f closed form %.4f <= 1 -> bisection must give 0, got %.4f", a, b, cf, g);
          continue;
        }
        worst = std::max(worst, std::fabs(g - cf) / cf);
        // roots at g*: z^3 - 2z^2 + (1+g a) z + g(b-a)
        Eigen::Matrix3d C;
        C << 2.0, -(1.0 + cf * a), -(cf * (b - a)), 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
        const Eigen::EigenSolver<Eigen::Matrix3d> es(C);
        int on_circle = 0;
        double real_root = 0.0;
        for (int i = 0; i < 3; ++i) {
          const std::complex<double> z = es.eigenvalues()(i);
          if (std::fabs(std::abs(z) - 1.0) < 1e-9 && std::fabs(z.imag()) > 1e-9) ++on_circle;
          else if (std::fabs(z.imag()) < 1e-9) real_root = z.real();
        }
        CHECK(on_circle == 2, "a=%.4f b=%.4f: %d complex roots on the unit circle", a, b, on_circle);
        worst_real = std::max(worst_real, std::fabs(real_root - cf * (a - b)));
        ++used;
      }
      std::printf("  %d random (a,b) with a>2b: worst relative |bisection - closed form| %.3e; |real root - g*(a-b)| %.3e\n",
                  used, worst, worst_real);
      CHECK(worst < 1e-7, "worst %.3e", worst);
      CHECK(worst_real < 1e-9, "real root %.3e", worst_real);
      CHECK(std::isnan(gainMarginClosedForm1(0.1, 0.06)), "closed form invalid for a <= 2b");
      CHECK(gainMargin(0.1, 0.06, 1) == 0.0, "a <= 2b: no stable point, margin 0");
      CHECK(gainMargin(2.5, 1.0, 1) == 0.0, "already unstable -> 0");

      // two cycles: polynomial z^2 (z-1)^2 + a(z-1) + b against the 4-state recursion
      double w2 = 0.0;
      std::uniform_real_distribution<double> ua2(0.0, 0.3), ub2(0.0, 0.05);
      for (int n = 0; n < 100; ++n) {
        const double a = ua2(rng), b = ub2(rng);
        const double rho = delayedSpectralRadius(a, b, 2);
        if (rho > 0.99 || rho < 0.0) continue;
        w2 = std::max(w2, std::fabs(rho - recursionRate(a, b, 2)));
      }
      std::printf("  two cycles: worst |max|z| - 4-state recursion rate| = %.3e\n", w2);
      CHECK(w2 < 5e-3, "two-cycle polynomial vs recursion %.3e", w2);
      const double g2 = gainMargin(0.5, 0.167, 2);
      std::printf("  a=0.5 b=0.167: g*(2 cycles) = %.4f (1 cycle %.4f)\n", g2, gainMargin(0.5, 0.167, 1));
      CHECK(g2 == 0.0, "a=0.5 b=0.167 is already unstable with two cycles of delay (max|z|=%.4f)", delayedSpectralRadius(0.5, 0.167, 2));
      CHECK(gainMargin(0.447, 0.05, 2) > 1.0 && gainMargin(0.447, 0.05, 2) < gainMargin(0.447, 0.05, 1), "a=0.447 b=0.05: stable but with less margin");
    }

    std::printf("[design: zeta regime]\n");
    {
      Array6 lam = {9.0, 3.0, 4.0, 0.04, 0.14, 0.05};
      const DampingDesign d = designDamping(cfg.gains, cfg.damping, scale, dt, lam, lam, lam, false);
      for (std::size_t i = 0; i < 6; ++i) {
        const double k = i < 3 ? cfg.gains.translational_stiffness : cfg.gains.rotational_stiffness;
        const double z = i < 3 ? cfg.damping.zeta_trans : cfg.damping.zeta_rot;
        CHECK(!d.capped[i], "axis %zu should not be capped", i);
        CHECK(std::fabs(d.d[i] - 2.0 * z * std::sqrt(k * lam[i])) < 1e-12, "axis %zu: D = 2 zeta sqrt(K Lambda)", i);
        CHECK(std::fabs(d.zeta[i] - z) < 1e-12, "axis %zu zeta", i);
        CHECK(d.a[i] <= cap + 1e-12, "axis %zu a = %.4f", i, d.a[i]);
        CHECK(std::fabs(d.zeta_lmax[i] - d.d[i] / (2.0 * std::sqrt(k * lam[i]))) < 1e-12, "axis %zu zeta@Lmax", i);
        CHECK(std::fabs(d.b[i] - k * dt * dt / lam[i]) < 1e-15, "axis %zu b", i);
        CHECK(std::fabs(d.rho[i] - delayedSpectralRadius(d.a[i], d.b[i])) < 1e-15, "axis %zu rho", i);
        if (i != 5) CHECK(d.g1[i] >= 10.0, "axis %zu g*(1)=%.2f must be >= 10", i, d.g1[i]);
      }
    }

    std::printf("[design: the cap holds, with a very small Lambda on one axis]\n");
    {
      Array6 lam = {9.0, 3.0, 4.0, 0.04, 0.14, 1e-6};  // rz tiny
      const DampingDesign d = designDamping(cfg.gains, cfg.damping, scale, dt, lam, lam, lam, false);
      CHECK(d.capped[5], "rz must be capped");
      CHECK(d.a[5] <= cap + 1e-12 && d.a[5] > cap - 1e-9, "rz: a = %.12f, cap %.3f", d.a[5], cap);
      CHECK(std::fabs(d.d[5] - cap * lam[5] / dt) < 1e-15, "rz: D = cap*Lambda/dt");
      CHECK(d.zeta[5] < cfg.damping.zeta_rot, "capped axis has zeta below the target (%.4f)", d.zeta[5]);
      for (std::size_t i = 0; i < 5; ++i) CHECK(!d.capped[i], "axis %zu must not be capped", i);
      CHECK(d.d_stop[5] * dt / lam[5] <= cap + 1e-12, "stop phase: cap re-applied");
    }

    std::printf("[design: random Lambda over 8 decades, a_i <= cap on every axis, also in the stop phase]\n");
    {
      std::mt19937 rng(2024);
      std::uniform_real_distribution<double> e(-7.0, 1.0);
      double worst_a = 0.0, worst_stop = 0.0;
      for (int n = 0; n < 3000; ++n) {
        Array6 act{}, mn{};
        for (std::size_t i = 0; i < 6; ++i) {
          act[i] = std::pow(10.0, e(rng));
          mn[i] = act[i] * std::pow(10.0, -2.0 * std::uniform_real_distribution<double>(0.0, 1.0)(rng));  // <= act
        }
        const DampingDesign d = designDamping(cfg.gains, cfg.damping, scale, dt, act, mn, act, true);
        for (std::size_t i = 0; i < 6; ++i) {
          worst_a = std::max(worst_a, d.a[i]);
          worst_stop = std::max(worst_stop, d.d_stop[i] * dt / d.lambda_min[i]);
          CHECK(d.a[i] <= cap * (1.0 + 1e-12), "a=%.6f", d.a[i]);
          CHECK(d.d_stop[i] * dt / d.lambda_min[i] <= cap * (1.0 + 1e-12), "stop a=%.6f", d.d_stop[i] * dt / d.lambda_min[i]);
          CHECK(d.lambda_min[i] <= act[i], "lambda_min must not exceed lambda_act");
        }
      }
      std::printf("  worst a_i %.9f, worst a_i in the stop phase %.9f (cap %.3f)\n", worst_a, worst_stop, cap);
    }

    std::printf("[design: Lambda_min above Lambda_act is clamped; override; invalid input]\n");
    {
      Array6 act = {9.0, 3.0, 4.0, 0.04, 0.14, 0.05};
      Array6 big = {90.0, 30.0, 40.0, 4.0, 14.0, 5.0};
      const DampingDesign d = designDamping(cfg.gains, cfg.damping, scale, dt, act, big, big, true);
      for (std::size_t i = 0; i < 6; ++i) CHECK(d.lambda_min[i] == act[i], "effective minimum is min(lambda_min, lambda_act)");

      DampingConfig ov = cfg.damping;
      ov.override_d = {20.0, 20.0, 20.0, 8.8, 8.8, 8.8};
      const DampingDesign o = designDamping(cfg.gains, ov, scale, dt, act, act, act, false);
      CHECK(o.overridden, "override flag");
      for (std::size_t i = 0; i < 6; ++i) {
        CHECK(o.d[i] == ov.override_d[i] && std::fabs(o.d_stop[i] - scale * ov.override_d[i]) < 1e-12, "override values");
      }
      bool threw = false;
      Array6 bad = act;
      bad[2] = 0.0;
      try {
        designDamping(cfg.gains, cfg.damping, scale, dt, bad, bad, bad, false);
      } catch (const std::invalid_argument&) {
        threw = true;
      }
      CHECK(threw, "Lambda = 0 must throw");
    }
  } catch (const std::exception& e) {
    std::printf("EXCEPTION: %s\n", e.what());
    return 2;
  }
  std::printf("\ntest_damping_design: %d check(s), %d failure(s)\n", g_checks, g_fail);
  return g_fail == 0 ? 0 : 1;
}
