#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H

#include "../bits/types.h"
#include "../bits/concepts.h"
#include "../bits/solver_types.h"
#include "../utils/type_conversion.h"
#include "../mechanics/compute_trajectory.h"
#include "../mechanics/sphere_vs_fixed_plane_detection.h"
#include "../mechanics/sphere_vs_fixed_plane_response.h"
#include <set>

namespace dte3607::physengine::solver_dev::level2
{
  template <typename Sphere_T, typename Params_T>
  void simulateAll(Sphere_T& data, Params_T const& params)
  {
    //    auto const proc_kernel = [&params](auto& data) {
    for (auto& sphere : data) {
      auto [ds, a] = mechanics::computeLinearTrajectory(
        sphere.v, params.F, params.timestep - (sphere.t_c - params.t_0));
      sphere.p += ds;
      sphere.v += a;
      sphere.ds += ds;
      sphere.a += a;
    }
    //    std::ranges::for_each(data, proc_kernel);
  }
  template <typename Intersect_T, typename Plane_T, typename Sphere_T,
            typename Params_T>
  void detectingCollision(Intersect_T& data, Sphere_T& spheres, Plane_T& planes,
                          Params_T const& params)
  {

    for (auto& sphere : spheres) {
      for (auto& plane : planes) {

        auto detection = mechanics::detectCollisionSphereFixedPlane(
          sphere.t_c, sphere.p, sphere.r, sphere.v, plane.p, plane.n, params.F,
          params.t_0, params.timestep);
        if (detection.has_value()) {
          //          solver_types::IntersectDetProcDataBlock intersection;
          //          intersection.sphere = sphere;
          //          intersection.plane  = plane;
          //          //                    intersection.status.is_collision =
          //          true; intersection.col_tp = *detection;
          //          //          sphere.t_c          = *detection;
          data.emplace_back(sphere, plane, detection.value());
        }
      }
    }
  }

  struct compare {
    bool operator()(const solver_types::IntersectDetProcDataBlock& int1,
                    const solver_types::IntersectDetProcDataBlock& int2) const
    {
      return int1.col_tp < int2.col_tp;
    }
  };


  template <typename Intersect_T>
  std::multiset<solver_types::IntersectDetProcDataBlock, compare>
  sortAndReduce(Intersect_T& intersection)
  {
    std::multiset<solver_types::IntersectDetProcDataBlock, compare> sorted;

    for (auto intersect : intersection) {
      sorted.insert(intersect);
    }


    return sorted;
  }
  template <typename Intersect_T, typename Param_T>
  void simulateObject(Intersect_T& intersection, Param_T const& params)
  {

    auto& sphere = intersection.sphere;
    auto [ds, a] = mechanics::computeLinearTrajectory(
      sphere.v, params.F, intersection.col_tp - sphere.t_c);


    sphere.p += ds;
    sphere.v += a;

    sphere.ds += ds;
    sphere.a += a;
  }

  template <typename SortIntersect_T, typename Param_T>
  void HandleFirstCollision(SortIntersect_T& sorted, Param_T const& params)
  {

    //    for (auto& intersection : sorted) {
    auto& intersection = *(sorted.begin());
    auto [ds, a]       = mechanics::computeLinearTrajectory(
      intersection.sphere.v, params.F,
      intersection.col_tp - intersection.sphere.t_c);

    simulateObject(intersection, params);

    auto VPrime = mechanics::computeImpactResponseSphereFixedPlane(
      intersection.sphere.v, intersection.plane.n);


    intersection.sphere.v   = VPrime;
    intersection.sphere.t_c = intersection.col_tp;
  }

  template <concepts::SolverFixtureLevel2 Fixture_T>
  void solve([[maybe_unused]] Fixture_T&         scenario,
             [[maybe_unused]] types::NanoSeconds timestep)
  {
    solver_types::Params params;
    params.F        = scenario.m_forces;
    params.timestep = timestep;
    auto now        = types::HighResolutionClock::now();
    params.t_0      = now;
    for (auto& spheres : scenario.m_backend.m_sphere_data) {
      spheres.t_c = now;
    }


    detectingCollision(scenario.m_backend.m_intersect_data,
                       scenario.m_backend.m_sphere_data,
                       scenario.m_backend.m_plane_data, params);

    auto sorted = sortAndReduce(scenario.m_backend.m_intersect_data);

    while (sorted.size() > 0) {
      HandleFirstCollision(sorted, params);
      scenario.m_backend.m_intersect_data.clear();
      detectingCollision(scenario.m_backend.m_intersect_data,
                         scenario.m_backend.m_sphere_data,
                         scenario.m_backend.m_plane_data, params);
      sorted = sortAndReduce(scenario.m_backend.m_intersect_data);
    }

    simulateAll(scenario.m_backend.m_sphere_data, params);

    for (auto const& [id, sphere] : scenario.m_backend.m_rb_sphere) {
      scenario.translateParent(id, scenario.m_backend.m_sphere_data[sphere].ds);
      scenario.addAcceleration(id, scenario.m_backend.m_sphere_data[sphere].a);
    }
  }
}   // namespace dte3607::physengine::solver_dev::level2
    // namespace dte3607::physengine::solver_dev::level2


#endif   // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H
