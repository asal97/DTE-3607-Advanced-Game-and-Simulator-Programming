#ifndef DTE3607_PHYSENGINE_SOLVER_TYPES_H
#define DTE3607_PHYSENGINE_SOLVER_TYPES_H

#include "types.h"

namespace dte3607::physengine::solver_types
{
  using Vector3 = types::Vector3;

  struct Params {
    Vector3 F;    // External forces(environment)
    types::Duration  timestep;   // Time step (system)
  };

  struct CacheProcDataBlock {
    Vector3 in_p;   // Position
    Vector3 in_v;   // Velocity

    Vector3 out_a;    // Acceleration
    Vector3 out_ds;   // Trajectory
  };

  struct SimProcDataBlock {
    Vector3 in_p;    // Position
    Vector3 in_ds;   // Trajectory

    Vector3 out_p;   // New Position
  };

  struct SphereGeomDataBlock{
      Vector3 p;
      double r;
      Vector3 ds;
  };

  struct InfPlaneGeomDataBlock{
      Vector3 p;
      double n;
  };

  struct IntsecStatusDataBlock{
      bool is_collision;
      double col_tp;
  };

  struct IntersectDetProcDataBlock{
      SphereGeomDataBlock sphere;
      InfPlaneGeomDataBlock plane;
      IntsecStatusDataBlock status;
  };

}   // namespace dte3607::physengine::solver_types


#endif   // DTE3607_PHYSENGINE_FIXTURES_H
