#include "acado_common.h"
#include "acado_qpoases3_interface.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {
constexpr std::uint64_t kMagic = UINT64_C(0x3150514d4f434343);
constexpr std::size_t kNvar = ACADO_QP_NV;
constexpr std::size_t kNcon = ACADO_NPAC * ACADO_N;
struct Record {
  std::uint64_t index=0; std::int32_t status=0,nwsr=0; double objective=0,kkt=0;
  std::vector<double> x,u,x0,od,y,yN,W,WN,lbValues,ubValues,lbAValues,ubAValues;
  std::vector<double> H,g,A,lb,ub,lbA,ubA,solution,dual;
};
template<typename T> bool read(std::istream& in,T* p,std::size_t n){in.read(reinterpret_cast<char*>(p),static_cast<std::streamsize>(n*sizeof(T)));return static_cast<bool>(in);}
bool readRecord(std::istream& in,Record& r){
  if(!read(in,&r.index,1))return false; if(!read(in,&r.status,1)||!read(in,&r.nwsr,1)||!read(in,&r.objective,1)||!read(in,&r.kkt,1))return false;
  auto get=[&](std::vector<double>&v,std::size_t n){v.resize(n);return read(in,v.data(),n);};
  return get(r.x,(ACADO_NX * (ACADO_N + 1)))&&get(r.u,kNvar)&&get(r.x0,7)&&get(r.od,(ACADO_NOD * (ACADO_N + 1)))&&get(r.y,(ACADO_NY * ACADO_N))&&get(r.yN,7)&&get(r.W,(ACADO_NY * ACADO_NY * ACADO_N))&&get(r.WN,(ACADO_NYN * ACADO_NYN))&&get(r.lbValues,kNvar)&&get(r.ubValues,kNvar)&&get(r.lbAValues,kNcon)&&get(r.ubAValues,kNcon)&&get(r.H,(kNvar * kNvar))&&get(r.g,kNvar)&&get(r.A,(kNvar * kNcon))&&get(r.lb,kNvar)&&get(r.ub,kNvar)&&get(r.lbA,kNcon)&&get(r.ubA,kNcon)&&get(r.solution,kNvar)&&get(r.dual,(kNvar + kNcon));
}
template<typename T,std::size_t N> void copyTo(T(&dst)[N],const std::vector<double>&src){std::copy(src.begin(),src.end(),dst);}
void load(const Record&r){copyTo(acadoVariables.x,r.x);copyTo(acadoVariables.u,r.u);copyTo(acadoVariables.x0,r.x0);copyTo(acadoVariables.od,r.od);copyTo(acadoVariables.y,r.y);copyTo(acadoVariables.yN,r.yN);copyTo(acadoVariables.W,r.W);copyTo(acadoVariables.WN,r.WN);copyTo(acadoVariables.lbValues,r.lbValues);copyTo(acadoVariables.ubValues,r.ubValues);copyTo(acadoVariables.lbAValues,r.lbAValues);copyTo(acadoVariables.ubAValues,r.ubAValues);}
struct Profile {double ax=0,ay=0,az=0,b=0; int active=0;};
using Profiles=std::map<std::tuple<std::uint64_t,std::string,int>,Profile>;
static std::vector<std::string> split(const std::string&s){std::vector<std::string>v;std::string x;std::stringstream q(s);while(std::getline(q,x,','))v.push_back(x);return v;}
 Profiles readProfiles(const char*path){Profiles out;std::ifstream in(path);if(!in)return out;std::string line;std::getline(in,line);while(std::getline(in,line)){auto v=split(line);if(v.size()<17)continue;try{const auto step=std::stoull(v[1]);const int stage=std::stoi(v[3]);Profile p{std::stod(v[12]),std::stod(v[13]),std::stod(v[14]),std::stod(v[16]),std::stoi(v[8])};out[{step,v[4],stage}]=p;}catch(...){}}return out;}
double objective(){double z=0;for(int i=0;i<kNvar;++i){z+=acadoWorkspace.g[i]*acadoWorkspace.x[i];for(int j=0;j<kNvar;++j)z+=.5*acadoWorkspace.x[i]*acadoWorkspace.H[i*kNvar+j]*acadoWorkspace.x[j];}return z;}
double maxResidual(){double m=0;for(int i=0;i<kNvar;++i){m=std::max(m,acadoWorkspace.lb[i]-acadoWorkspace.x[i]);m=std::max(m,acadoWorkspace.x[i]-acadoWorkspace.ub[i]);}for(int i=0;i<kNcon;++i){double v=0;for(int j=0;j<kNvar;++j)v+=acadoWorkspace.A[i*kNvar+j]*acadoWorkspace.x[j];if(v<acadoWorkspace.lbA[i])m=std::max(m,acadoWorkspace.lbA[i]-v);if(v>acadoWorkspace.ubA[i])m=std::max(m,v-acadoWorkspace.ubA[i]);}return m;}
double maxDiff(const double* a,const std::vector<double>& b){double m=0;for(std::size_t i=0;i<b.size();++i)m=std::max(m,std::abs(a[i]-b[i]));return m;}
int run(int argc,char**argv){
  if(argc<3){std::cerr<<"usage: qp_counterfactual_tool CAPTURE.bin profiles.csv [step ...]\n";return 2;}
  std::ifstream in(argv[1],std::ios::binary);if(!in)return 2;auto profiles=readProfiles(argv[2]);
  std::uint64_t magic=0;std::uint32_t version=0,nv=0,nc=0,nx=0,nu=0,nod=0;if(!read(in,&magic,1)||!read(in,&version,1)||!read(in,&nv,1)||!read(in,&nc,1)||!read(in,&nx,1)||!read(in,&nu,1)||!read(in,&nod,1)||magic!=kMagic||version!=1||nv!=kNvar||nc!=kNcon||nx!=ACADO_NX*(ACADO_N+1)||nu!=ACADO_NU*ACADO_N||nod!=ACADO_NOD*(ACADO_N+1)){std::cerr<<"capture contract differs from linked solver\n";return 3;}
  std::vector<std::uint64_t>want;for(int i=3;i<argc;++i)want.push_back(std::stoull(argv[i]));
  if(acado_initializeSolver()!=0)return 4;std::cout<<"step,variant,valid,status,nwsr,objective,kkt,residual,dH,dA,dg,dlb,dub,dlbA,dubA,changed_A_rows,changed_lbA_rows,changed_ubA_rows,u0,u1,u2,u3,delta_u0_from_A,delta_u1_from_A,delta_u2_from_A,delta_u3_from_A\n";
  std::size_t seen=0; bool have_previous=false; Record previous;
  while(true){Record r;if(!readRecord(in,r))break;bool selected=want.empty()||std::find(want.begin(),want.end(),r.index)!=want.end();if(!selected){previous=std::move(r);have_previous=true;++seen;continue;}
    const std::vector<std::string> vars={"A_original","B_fixed_snapshot","C_fixed_slice","D_fixed_ids_current_state","E_previous_nominal_reference"}; double base_u[4]={0,0,0,0}; std::vector<double> baseH((kNvar * kNvar)),baseA((kNvar * kNcon)),baseg(kNvar),baseLb(kNvar),baseUb(kNvar),baseLbA(kNcon),baseUbA(kNcon);
    for(const auto&variant:vars){load(r);if(variant=="E_previous_nominal_reference"){if(!have_previous)continue;copyTo(acadoVariables.x,previous.x);copyTo(acadoVariables.u,previous.u);copyTo(acadoVariables.y,previous.y);copyTo(acadoVariables.yN,previous.yN);copyTo(acadoVariables.W,previous.W);copyTo(acadoVariables.WN,previous.WN);}else if(variant!="A_original"){bool missing=false;for(int stage=0;stage<21;++stage){auto it=profiles.find({r.index,variant,stage});if(it==profiles.end()){missing=true;break;}const auto&p=it->second;const int off=stage*ACADO_NOD+80;acadoVariables.od[off+0]=p.ax;acadoVariables.od[off+1]=p.ay;acadoVariables.od[off+2]=p.az;acadoVariables.od[off+3]=p.b;acadoVariables.od[off+4]=static_cast<double>(p.active);}if(missing){std::cerr<<"Missing profile step "<<r.index<<" variant "<<variant<<"\n";return 6;}}
      const int prep=acado_preparationStep();if(prep!=0){std::cerr<<"prep failure step "<<r.index<<" variant "<<variant<<"\n";return 5;}const int status=acado_feedbackStep();
      const bool valid=status==0 && maxResidual()<=1e-7;
      if(variant=="A_original")for(int j=0;j<4;++j)base_u[j]=acadoVariables.u[j];
      if(variant=="A_original"){std::copy(acadoWorkspace.H,acadoWorkspace.H+(kNvar * kNvar),baseH.begin());std::copy(acadoWorkspace.A,acadoWorkspace.A+(kNvar * kNcon),baseA.begin());std::copy(acadoWorkspace.g,acadoWorkspace.g+kNvar,baseg.begin());}
      double dlb = 0.0, dub = 0.0, dlbA = 0.0, dubA = 0.0;
      std::ostringstream changedA, changedLbA, changedUbA;
      if (variant == "A_original") {
        std::copy(acadoWorkspace.lb, acadoWorkspace.lb + kNvar, baseLb.begin());
        std::copy(acadoWorkspace.ub, acadoWorkspace.ub + kNvar, baseUb.begin());
        std::copy(acadoWorkspace.lbA, acadoWorkspace.lbA + kNcon, baseLbA.begin());
        std::copy(acadoWorkspace.ubA, acadoWorkspace.ubA + kNcon, baseUbA.begin());
      } else {
        for (int i = 0; i < kNvar; ++i) {
          dlb = std::max(dlb, std::abs(acadoWorkspace.lb[i] - baseLb[i]));
          dub = std::max(dub, std::abs(acadoWorkspace.ub[i] - baseUb[i]));
        }
        for (int i = 0; i < kNcon; ++i) {
          const double dl = std::abs(acadoWorkspace.lbA[i] - baseLbA[i]);
          const double du = std::abs(acadoWorkspace.ubA[i] - baseUbA[i]);
          dlbA = std::max(dlbA, dl); dubA = std::max(dubA, du);
          bool row_changed = false;
          for (int j = 0; j < kNvar; ++j)
            row_changed = row_changed || std::abs(acadoWorkspace.A[i * kNvar + j] - baseA[i * kNvar + j]) > 1e-12;
          if (row_changed) { if (changedA.tellp() > 0) changedA << ';'; changedA << i; }
          if (dl > 1e-12) { if (changedLbA.tellp() > 0) changedLbA << ';'; changedLbA << i; }
          if (du > 1e-12) { if (changedUbA.tellp() > 0) changedUbA << ';'; changedUbA << i; }
        }
      }
      std::cout<<r.index<<','<<variant<<','<<valid<<','<<status<<','<<acado_getNWSR()<<','<<std::setprecision(17)<<objective()<<','<<acado_getKKT()<<','<<maxResidual()<<','<<maxDiff(acadoWorkspace.H,baseH)<<','<<maxDiff(acadoWorkspace.A,baseA)<<','<<maxDiff(acadoWorkspace.g,baseg)<<','<<dlb<<','<<dub<<','<<dlbA<<','<<dubA<<','<<changedA.str()<<','<<changedLbA.str()<<','<<changedUbA.str()<<','<<acadoVariables.u[0]<<','<<acadoVariables.u[1]<<','<<acadoVariables.u[2]<<','<<acadoVariables.u[3]<<','<<acadoVariables.u[0]-base_u[0]<<','<<acadoVariables.u[1]-base_u[1]<<','<<acadoVariables.u[2]-base_u[2]<<','<<acadoVariables.u[3]-base_u[3]<<'\n';
    } previous=std::move(r);have_previous=true;++seen;
  }
  return 0;
}
}
int main(int argc,char**argv){return run(argc,argv);}
