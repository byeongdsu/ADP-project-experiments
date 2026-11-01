// Diagnostic only: no ROS nodes, vehicle commands or changes to the solver library.
#include <omp.h>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <cmath>
#include "mppi_controller/prior_samples_with_costs.hpp"
using mppi::cpu::PriorSamplesWithCosts;
uint64_t digest(const PriorSamplesWithCosts& sampler) {
 uint64_t hash=14695981039346656037ULL;
 for(const auto& seq:sampler.noise_seq_samples_) for(Eigen::Index i=0;i<seq.size();++i) {
  uint64_t bits;double value=seq.data()[i];std::memcpy(&bits,&value,sizeof(bits));hash^=bits;hash*=1099511628211ULL;
 }
 return hash;
}
int main() {
 omp_set_dynamic(0);
 std::cout<<"{\"configuration\":{\"samples\":1000,\"prediction_states\":30,\"seed\":42,\"variance\":0.1,\"trials\":10,\"sampling_calls_per_trial\":5},\"tests\":[";
 bool comma=false;
 for(int threads:{1,4}) {
  int different_trials=0,different_calls=0;double max_difference=0;uint64_t first_digest=0;int nonfinite=0;
  for(int trial=0;trial<10;++trial) {
   PriorSamplesWithCosts a(1000,30,{.4189},{-.4189},.1,threads,42);
   PriorSamplesWithCosts b(1000,30,{.4189},{-.4189},.1,threads,42);
   auto mean=a.get_constant_control_seq({.08});auto cov=a.get_constant_control_seq_cov_matrices({.1});bool mismatch=false;
   for(int call=0;call<5;++call) {
    a.random_sampling(mean,cov);b.random_sampling(mean,cov);
    auto ha=digest(a),hb=digest(b);if(trial==0 && call==0) first_digest=ha;
    if(ha!=hb){mismatch=true;++different_calls;}
    for(size_t i=0;i<a.noise_seq_samples_.size();++i) {
     const auto& aa=a.noise_seq_samples_[i];const auto& bb=b.noise_seq_samples_[i];
     if(!aa.allFinite()||!bb.allFinite())++nonfinite;
     max_difference=std::max(max_difference,(aa-bb).cwiseAbs().maxCoeff());
    }
   }
   different_trials+=mismatch;
  }
  if(comma)std::cout<<",";comma=true;
  std::cout<<"{\"threads\":"<<threads<<",\"mismatched_trials\":"<<different_trials<<",\"mismatched_sampling_calls\":"<<different_calls<<",\"max_abs_noise_difference\":"<<std::setprecision(16)<<max_difference<<",\"first_digest\":\""<<std::hex<<first_digest<<std::dec<<"\",\"nonfinite_sequences\":"<<nonfinite<<"}";
 }
 std::cout<<"]}\n";
}
