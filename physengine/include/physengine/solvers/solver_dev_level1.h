#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL1_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL1_H

#include "../utils/type_conversion.h"
#include "../bits/types.h"
#include "../bits/concepts.h"
#include "../bits/solver_types.h"
#include "../mechanics/compute_trajectory.h"


namespace dte3607::physengine::solver_dev::level1
{
  template <typename Data_T, typename Params_T>
  void computeCache(Data_T& data, Params_T const& params)
  {
    auto const proc_kernel = [&params](auto& data) {
      auto const& [F, dt]             = params;
      auto& [pos, vel, out_a, out_ds] = data;


      std::tuple <types::Vector3, types::Vector3> outputs = std::tie(out_ds,out_a);
      outputs = mechanics::computeLinearTrajectory(vel,F,dt);
//      out_a                           = F * dt;
//      out_ds                          = vel * dt + 0.5 * F * std::pow(dt, 2.0);

      // Move to different process
       out_ds = std::get<0>(outputs);
       out_a = std::get<1>(outputs);
       pos += out_ds;
       vel += out_a;
    };
    std::ranges::for_each(data, proc_kernel);
  }

  template <concepts::SolverFixture Fixture_T>
  void solve(Fixture_T& scenario, types::Duration timestep)
  {
    solver_types::Params params;
    params.F = scenario.m_forces;
    params.timestep = timestep;
//    std::tuple<typename Fixture_T::Vector3, types::ValueType> params;
//    std::get<0>(params) = scenario.m_forces;
//    std::get<1>(params) = utils::toDt(timestep);
    computeCache(scenario.m_backend->m_cache_data, params);

    for (auto const& id : scenario.m_backend->m_rb_cache) {
      scenario.translateParent(
        id.second, scenario.m_backend->m_cache_data[id.first].out_ds);
      scenario.addAcceleration(
        id.second, scenario.m_backend->m_cache_data[id.first].out_a);
    }
  }

}   // namespace dte3607::physengine::solver_dev::level1


#endif   // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL1_H
