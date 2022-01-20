#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H

#include "../bits/types.h"

// stl
#include <utility>

namespace dte3607::physengine::mechanics
{

  inline
    std::pair<types::Vector3, types::Vector3>
    computeImpactResponseSphereSphere(
      [[maybe_unused]] types::Point3 const&  s1_p,
      [[maybe_unused]] types::Vector3 const& s1_v,
      [[maybe_unused]] types::ValueType      s1_mass,
      [[maybe_unused]] types::Point3 const&  s2_p,
      [[maybe_unused]] types::Vector3 const& s2_v,
      [[maybe_unused]] types::ValueType      s2_mass)
  {
      auto const minus = (s2_p - s1_p);
      auto const magnitude = std::sqrt(std::pow(minus[0],2) +std::pow(minus[1],2) +std::pow(minus[2],2) );
      auto const d = minus / magnitude;
      auto const V1d = blaze::inner(s1_v,d);
      auto const V2d = blaze::inner(s2_v,d);
      auto const V1n = s1_v - V1d * d;
      auto const V2n = s2_v - V2d * d;
      auto const Vprime1d = ((s1_mass-s2_mass)/(s1_mass + s2_mass)) * V1d + ((2*s2_mass)/(s1_mass+s2_mass)) * V2d;
      auto const Vprime2d = ((s2_mass-s1_mass)/(s1_mass + s2_mass)) * V2d + ((2*s1_mass)/(s1_mass+s2_mass)) * V1d;
      auto const Vprime1 = V1n + Vprime1d * d;
      auto const Vprime2 = V2n + Vprime2d * d;



    return std::make_pair(Vprime1,Vprime2);
  }


}   // namespace dte3607::physengine::mechanics


#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H
