#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL0_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL0_H

#include "../utils/type_conversion.h"
#include "../bits/types.h"
#include "../api_concepts.h"

namespace dte3607::physengine::solver_dev::level0
{

  namespace example_impl
  {
    inline std::pair<types::Vector3, types::Vector3>
    computeLinearTrajectory(types::Vector3 const& velocity,
                            types::Vector3 const& external_forces,
                            types::Duration       timestep)
    {
      auto const F  = external_forces;
      auto const dt = utils::toDt(timestep);

      auto const v = velocity;
      auto const a = F * dt;

      auto const ds = (v + 0.5 * a) * dt;

      return {ds, a};
    }

  }   // namespace example_impl

  template <api::concepts::SolverFixtureLevel0 Fixture_T>
  void solve(Fixture_T& scenario, types::NanoSeconds timestep)
  {

    auto no_rbs = scenario.noRigidBodies();
    for( auto i = 0; i < no_rbs; ++i )
    {
      auto constexpr NonFixed = Fixture_T::RBMode::NonFixed;
      if (scenario.mode(i) not_eq NonFixed) continue;

      auto const [lin_traj, accel] = example_impl::computeLinearTrajectory(
        scenario.globalVelocity(i), scenario.externalForces(), timestep);

      scenario.translateParent(i,lin_traj);
      scenario.addAcceleration(i, accel);
    }
  }

}   // namespace dte3607::physengine::solver_dev::level0


#endif // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL0_H
