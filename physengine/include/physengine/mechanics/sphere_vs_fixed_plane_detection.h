#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H

#include "../bits/types.h"
#include "../utils/type_conversion.h"
#include "../mechanics/compute_trajectory.h"

// stl
#include <optional>

namespace dte3607::physengine::mechanics
{

  inline std::optional<types::HighResolutionTP> detectCollisionSphereFixedPlane(
    [[maybe_unused]] types::HighResolutionTP const& sphere_tc,
    [[maybe_unused]] types::Point3 const&           sphere_p,
    [[maybe_unused]] types::ValueType               sphere_r,
    [[maybe_unused]] types::Vector3 const&          sphere_v,
    [[maybe_unused]] types::Point3 const&           fplane_q,
    [[maybe_unused]] types::Vector3 const&          fplane_n,
    [[maybe_unused]] types::Vector3 const&          external_forces,
    [[maybe_unused]] types::HighResolutionTP const& t_0,
    [[maybe_unused]] types::Duration                timestep)
  {
    auto const ds
      = mechanics::computeLinearTrajectory(sphere_v, external_forces, timestep)
          .first;

    auto const d         = (fplane_q + (sphere_r * fplane_n)) - sphere_p;
    auto       RWithoutN = blaze::inner(ds, fplane_n);
    auto       QWithoutN = blaze::inner(d, fplane_n);
    auto const x         = QWithoutN / RWithoutN;

    if (blaze::inner(ds, fplane_n) == 0) return std::nullopt;

    auto const timeOfimpact = t_0 + (x * timestep);
    if (timeOfimpact <= sphere_tc) return std::nullopt;

    if (x > 0
        && x <= 1 - (utils::toDt(sphere_tc - t_0) / utils::toDt(timestep)))
      return t_0
             + utils::toDuration(types::SecondsD(x * (utils::toDt(timestep))));

    else
      return std::nullopt;
  }

}   // namespace dte3607::physengine::mechanics



#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
