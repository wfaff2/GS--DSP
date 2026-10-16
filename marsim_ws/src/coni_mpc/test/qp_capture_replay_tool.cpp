#include "acado_common.h"
#include "acado_qpoases3_interface.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {
constexpr std::uint64_t kMagic = UINT64_C(0x3150514d4f434343);
constexpr std::size_t kNvar = 140;
constexpr std::size_t kNcon = 200;

struct Record {
  std::uint64_t index = 0;
  std::int32_t status = 0;
  std::int32_t nwsr = 0;
  double objective = 0.0;
  double kkt = 0.0;
  std::vector<double> x, u, x0, od, y, yN, W, WN;
  std::vector<double> lbValues, ubValues, lbAValues, ubAValues;
  std::vector<double> H, g, A, lb, ub, lbA, ubA, solution, dual;
};

template <typename T>
bool read(std::istream& in, T* data, std::size_t n) {
  in.read(reinterpret_cast<char*>(data), static_cast<std::streamsize>(n * sizeof(T)));
  return static_cast<bool>(in);
}

bool readRecord(std::istream& in, Record& r) {
  if (!read(in, &r.index, 1)) return false;
  if (!read(in, &r.status, 1) || !read(in, &r.nwsr, 1) ||
      !read(in, &r.objective, 1) || !read(in, &r.kkt, 1)) return false;
  auto get = [&](std::vector<double>& v, std::size_t n) {
    v.resize(n); return read(in, v.data(), n);
  };
  return get(r.x,147) && get(r.u,140) && get(r.x0,7) && get(r.od,1827) &&
      get(r.y,280) && get(r.yN,7) && get(r.W,3920) && get(r.WN,49) &&
      get(r.lbValues,140) && get(r.ubValues,140) && get(r.lbAValues,200) &&
      get(r.ubAValues,200) && get(r.H,19600) && get(r.g,140) &&
      get(r.A,28000) && get(r.lb,140) && get(r.ub,140) && get(r.lbA,200) &&
      get(r.ubA,200) && get(r.solution,140) && get(r.dual,340);
}

template <typename T, std::size_t N>
void copyTo(T (&dst)[N], const std::vector<double>& src) {
  std::copy(src.begin(), src.end(), dst);
}

void loadVariables(const Record& r) {
  copyTo(acadoVariables.x, r.x); copyTo(acadoVariables.u, r.u);
  copyTo(acadoVariables.x0, r.x0); copyTo(acadoVariables.od, r.od);
  copyTo(acadoVariables.y, r.y); copyTo(acadoVariables.yN, r.yN);
  copyTo(acadoVariables.W, r.W); copyTo(acadoVariables.WN, r.WN);
  copyTo(acadoVariables.lbValues, r.lbValues);
  copyTo(acadoVariables.ubValues, r.ubValues);
  copyTo(acadoVariables.lbAValues, r.lbAValues);
  copyTo(acadoVariables.ubAValues, r.ubAValues);
}

double maxDiff(const double* a, const std::vector<double>& b, std::size_t n) {
  double result = 0.0;
  for (std::size_t i=0; i<n; ++i) result = std::max(result, std::abs(a[i]-b[i]));
  return result;
}

double objectiveFromWorkspace() {
  double value = 0.0;
  for (std::size_t i=0; i<kNvar; ++i) {
    value += acadoWorkspace.g[i] * acadoWorkspace.x[i];
    for (std::size_t j=0; j<kNvar; ++j)
      value += 0.5 * acadoWorkspace.x[i] *
          acadoWorkspace.H[i*kNvar+j] * acadoWorkspace.x[j];
  }
  return value;
}

int run(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: qp_capture_replay_tool CAPTURE.bin [MAX_RECORDS]\n";
    return 2;
  }
  std::ifstream in(argv[1], std::ios::binary);
  if (!in) { std::cerr << "cannot open capture\n"; return 2; }
  std::uint64_t magic=0; std::uint32_t version=0,nv=0,nc=0,nx=0,nu=0,nod=0;
  if (!read(in,&magic,1) || !read(in,&version,1) || !read(in,&nv,1) ||
      !read(in,&nc,1) || !read(in,&nx,1) || !read(in,&nu,1) || !read(in,&nod,1) ||
      magic!=kMagic || version!=1 || nv!=140 || nc!=200 || nx!=147 || nu!=140 || nod!=1827) {
    std::cerr << "invalid capture header\n"; return 3;
  }
  const std::size_t limit = argc > 2 ? std::stoull(argv[2]) : std::numeric_limits<std::size_t>::max();
  if (acado_initializeSolver() != 0) { std::cerr << "acado_initializeSolver failed\n"; return 4; }
  std::size_t count=0, ok=0; double max_h=0,max_a=0,max_b=0,max_x=0,max_y=0,max_obj=0,max_kkt=0;
  while (count < limit) {
    Record r; if (!readRecord(in,r)) break;
    loadVariables(r);
    const int prep = acado_preparationStep();
    if (prep != 0) { std::cerr << "prep failed at record " << r.index << " status=" << prep << "\n"; return 5; }
    const double prep_h_error = maxDiff(acadoWorkspace.H,r.H,19600);
    const double prep_a_error = maxDiff(acadoWorkspace.A,r.A,28000);
    max_h=std::max(max_h,prep_h_error);
    max_a=std::max(max_a,prep_a_error);
    const int status = acado_feedbackStep();
    const double qp_vector_error = std::max({maxDiff(acadoWorkspace.g,r.g,140),
        maxDiff(acadoWorkspace.lb,r.lb,140), maxDiff(acadoWorkspace.ub,r.ub,140),
        maxDiff(acadoWorkspace.lbA,r.lbA,200), maxDiff(acadoWorkspace.ubA,r.ubA,200)});
    max_b=std::max(max_b,qp_vector_error);
    max_x=std::max(max_x,maxDiff(acadoWorkspace.x,r.solution,140));
    max_y=std::max(max_y,maxDiff(acadoWorkspace.y,r.dual,340));
    max_obj=std::max(max_obj,std::abs(objectiveFromWorkspace()-r.objective));
    max_kkt=std::max(max_kkt,std::abs(acado_getKKT()-r.kkt));
    if (status==r.status && acado_getNWSR()==r.nwsr &&
        prep_h_error <= 1e-10 && prep_a_error <= 1e-10 &&
        qp_vector_error <= 1e-10 &&
        maxDiff(acadoWorkspace.x,r.solution,140) <= 1e-8) ++ok;
    ++count;
  }
  std::cout << std::setprecision(17)
            << "records=" << count << " exact_status_nwsr_and_solution=" << ok
            << " max_H_error=" << max_h << " max_A_error=" << max_a
            << " max_bounds_g_error=" << max_b << " max_primal_error=" << max_x
            << " max_dual_error=" << max_y << " max_objective_error=" << max_obj
            << " max_kkt_error=" << max_kkt << '\n';
  return count == 0 ? 6 : 0;
}

}  // namespace

int main(int argc, char** argv) {
  return run(argc, argv);
}
