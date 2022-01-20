#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H

#include "../bits/types.h"
#include "../bits/concepts.h"
#include "../bits/solver_types.h"
#include "../utils/type_conversion.h"

namespace dte3607::physengine::solver_dev::level2
{
//template <typename Params_T,typename Data_T/*,typename Plane_T *//*, typename Sphere_T*/>
//void computeCache(Data_T& data,/*Plane_T const& plane,*//* Sphere_T& sphere,*/ Params_T const& params)
//    {
//      auto const proc_kernel = [&params](auto& data) {
//        auto const& [F, dt]             = params;
//        auto& [pos, vel, out_a, out_ds] = data;
////        auto const& [planePos, n] = plane;
////        auto& [spherePos, r, ds] = sphere;


//      };
//      std::ranges::for_each(data, proc_kernel);
//    }
//template <typename Plane_T , typename Sphere_T>
//void detectingCollision(Plane_T const& planes, Sphere_T& spheres,types::Duration timestep)
//    {
//      auto const proc_kernel = [&sphere](auto& data) {
//        auto& [spherePos, r, ds] = data;

//      };
//      std::ranges::for_each(sphere, proc_kernel);

//                foreach (sphere, spheres) {
//            foreach (plane, planes) {

//            }

//        }
//    }


  template <concepts::SolverFixtureLevel2 Fixture_T>
  void solve([[maybe_unused]] Fixture_T&         scenario,
             [[maybe_unused]] types::NanoSeconds timestep)
  {
      solver_types::Params params;
      params.F = scenario.m_forces;
//      params.timestep = utils::toDuration(timestep);


//      solver_types::SphereGeomDataBlock sphere;
//      sphere.p = ;
//      sphere.r = ;
//      sphere.ds = ;

//      solver_types::InfPlaneGeomDataBlock plane;
//      plane.n = ;
//      plane.p = ;

//      computeCache(scenario.m_backend->m_cache_data, params);
//      for (auto const& id : scenario.m_backend->m_rb_cache) {
//        scenario.translateParent(
//          id.second, scenario.m_backend->m_cache_data[id.first].out_ds);
//        scenario.addAcceleration(
//          id.second, scenario.m_backend->m_cache_data[id.first].out_a);
//      }

  }


}   // namespace dte3607::physengine::solver_dev::level2


#endif // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H
