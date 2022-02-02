#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H

#include "../bits/types.h"
#include "../bits/concepts.h"
#include "../bits/solver_types.h"
#include "../utils/type_conversion.h"
#include "../mechanics/sphere_vs_fixed_plane_detection.h"
#include "../mechanics/sphere_vs_fixed_plane_response.h"
#include <set>

namespace dte3607::physengine::solver_dev::level2
{
  template <typename Sphere_T, typename Params_T>
  void simulateAll(Sphere_T& data, Params_T const& params)
  {
    auto const proc_kernel = [&params](auto& data) {
      auto& [pos, r, ds] = data;
      pos                = pos + ds;
    };
    std::ranges::for_each(data, proc_kernel);
  }
  template <typename Intersect_T, typename Plane_T, typename Sphere_T,
            typename Cache_T, typename Params_T>
  void detectingCollision(Intersect_T& data, Sphere_T& spheres, Plane_T& planes,
                          Cache_T& cache, Params_T& params)
  {
    // plane

    for (auto i = 0; i < spheres.size(); i++) {
      for (auto plane : planes) {

        auto detection = mechanics::detectCollisionSphereFixedPlane(
          cache[i].t_c, spheres[i].p, spheres[i].r, cache[i].in_v, plane.p,
          plane.n, params.F, params.t_0, params.timestep);
        if (detection.has_value()) {
          solver_types::IntersectDetProcDataBlock intersection;
          intersection.sphere              = spheres[i];
          intersection.plane               = plane;
          intersection.status.is_collision = true;
          intersection.status.col_tp       = *detection;
          cache[i].t_c                     = *detection;
          data.emplace_back(intersection);
        }
      }
    }
  }
  bool compare(solver_types::IntersectDetProcDataBlock int1,
               solver_types::IntersectDetProcDataBlock int2)
  {
    return int1.status.col_tp < int2.status.col_tp;
  }

  template <typename Intersect_T>
  std::set<solver_types::IntersectDetProcDataBlock, decltype(compare)*>
  sortAndReduce(Intersect_T& intersection)
  {
    std::set<solver_types::IntersectDetProcDataBlock, decltype(compare)*>
      sorted;

    for (auto intersect : intersection) {
      if (!sorted.contains(intersect)) sorted.insert(intersect);
    }

    return sorted;
  }
  template <typename Sphere_T>
  void simulateObjects(Sphere_T& sphere)
  {
    // do i need to use simProcDataBlock or sphere
    // do i need to
    auto p   = sphere.p;
    auto ds  = sphere.ds;
    sphere.p = p + ds;
  }

  template <typename Intersect_T, typename Cache_T, typename Sphere_T,
            typename Param_T>
  void HandleFirstCollision(Intersect_T& sorted, Cache_T& cache,
                            Sphere_T& spheres, Param_T& params)
  {
    int index = 0;
    for (auto iter_sort : sorted) {
      //    for (auto i = 0; i < sorted.size(); i++) {
      simulateObjects(iter_sort.sphere);
      //      std::vector<int>::iterator itr = std::find(v.begin(), v.end(),
      //      key);

      for (int i = 0; i < spheres.size(); i++) {
        if (iter_sort.sphere.p == spheres[i].p
            && iter_sort.sphere.r == spheres[i].r
            && iter_sort.sphere.ds == spheres[i].ds) {
          index = i;
          break;
        }
      }
      //      std::vector<solver_types::SphereGeomDataBlock>::iterator itr
      //        = std::find(spheres.begin(), spheres.end(), iter_sort.sphere);
      //      if (itr != spheres.end()) {
      //      auto index        = std::distance(spheres.begin(), itr);
      cache[index].in_v = mechanics::computeImpactResponseSphereFixedPlane(
        cache[index].in_v, iter_sort.plane.n);
      //////////////////
      auto detection = mechanics::detectCollisionSphereFixedPlane(
        cache[index].t_c, spheres[index].p, spheres[index].r, cache[index].in_v,
        iter_sort.plane.p, iter_sort.plane.n, params.F, params.t_0,
        params.timestep - (iter_sort.status.col_tp - params.t_0));

      if (detection.has_value()) {
        solver_types::IntersectDetProcDataBlock intersection;
        intersection.sphere              = spheres[index];
        intersection.plane               = iter_sort.plane;
        intersection.status.is_collision = true;
        intersection.status.col_tp       = *detection;
        cache[index].t_c                 = *detection;
        sorted.insert(intersection);
      }
    }
  }

  template <concepts::SolverFixtureLevel2 Fixture_T>
  void solve([[maybe_unused]] Fixture_T&         scenario,
             [[maybe_unused]] types::NanoSeconds timestep)
  {
    solver_types::Params params;
    params.F        = scenario.m_forces;
    params.timestep = timestep;

    //     now   = ;
    params.t_0 = types::HighResolutionClock::now();


    detectingCollision(
      scenario.m_backend.m_intersect_data, scenario.m_backend.m_sphere_data,
      scenario.m_backend.m_plane_data, scenario.m_backend.m_cache_data, params);

    auto sorted = sortAndReduce(scenario.m_backend.m_intersect_data);
    HandleFirstCollision(sorted, scenario.m_backend.m_cache_data,
                         scenario.m_backend.m_sphere_data, params);
    simulateAll(scenario.m_backend.m_sphere_data, params);
  }
}   // namespace dte3607::physengine::solver_dev::level2
    // namespace dte3607::physengine::solver_dev::level2


#endif   // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H
