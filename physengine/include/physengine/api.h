#ifndef DTE3607_PHYSENGINE_FIXTURE_API_H
#define DTE3607_PHYSENGINE_FIXTURE_API_H


//#include "solvers/solver_dev_level0.h"
#include "solvers/solver_dev_level1.h"
#include "bits/types.h"
#include "bits/concepts.h"



namespace dte3607::physengine::api
{

  // SOLVER
  template <concepts::SolverFixture Fixture_T>
  void solve(Fixture_T& scenario, types::NanoSeconds timestep)
  {
    // Call another solver !_!
    solver_dev::level1::solve(scenario, timestep);
  }
}   // namespace dte3607::physengine::api


#endif   // DTE3607_PHYSENGINE_FIXTURE_API_H
