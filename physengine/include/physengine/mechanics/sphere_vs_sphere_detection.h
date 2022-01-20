#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H

#include "compute_trajectory.h"
#include "../bits/types.h"

// stl
#include <optional>

namespace dte3607::physengine::mechanics
{

  inline std::optional<types::ValueType> detectCollisionSphereSphere(
    [[maybe_unused]] types::HighResolutionTP const& s1_tc,
    [[maybe_unused]] types::Point3 const&           s1_p,
    [[maybe_unused]] types::ValueType               s1_r,
    [[maybe_unused]] types::Vector3 const&          s1_v,
    [[maybe_unused]] types::HighResolutionTP const& s2_tc,
    [[maybe_unused]] types::Point3 const&           s2_p,
    [[maybe_unused]] types::ValueType               s2_r,
    [[maybe_unused]] types::Vector3 const&          s2_v,
    [[maybe_unused]] types::Vector3 const&          external_forces,
    [[maybe_unused]] types::HighResolutionTP const& t_0,
    [[maybe_unused]] types::Duration                timestep)
  {
      auto const ds1 = mechanics::computeLinearTrajectory(s1_v,external_forces,timestep).first;
      auto const ds2 = mechanics::computeLinearTrajectory(s2_v,external_forces,timestep).first;
      auto const r = s1_r + s2_r;
      auto const Q = s2_p - s1_p;
      auto const R = ds2 - ds1;

      if (blaze::inner(R,R)==0)
          return std::nullopt;


      auto const innerRQ = blaze::inner(Q,R);

      if ((std::pow(innerRQ,2) - (blaze::inner(R,R)) *
              (blaze::inner(Q,Q) - std::pow(r,2))) < 0)
          return std::nullopt;

      auto const x = (-innerRQ - std::sqrt(std::pow(innerRQ,2) - (blaze::inner(R,R)) *
                                           (blaze::inner(Q,Q) - std::pow(r,2)))) /
              blaze::inner(R,R);

      if (x<=0 || x>1)
          return std::nullopt;

      auto const timeOfimpact = t_0 + (x * timestep);
      if (timeOfimpact > t_0 && timeOfimpact > s1_tc && timeOfimpact > s2_tc && timeOfimpact < t_0 + timestep)
          return x;
      else
          return std::nullopt;

  }


}   // namespace dte3607::physengine::mechanics



#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H
