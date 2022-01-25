#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H

#include "../bits/types.h"
#include "../bits/concepts.h"
#include "../bits/solver_types.h"
#include "../utils/type_conversion.h"
#include "../mechanics/sphere_vs_fixed_plane_detection.h"

namespace dte3607::physengine::solver_dev::level2
{
template <typename Params_T,typename Data_T/*,typename Plane_T*/ /*, typename Sphere_T*/>
void computeCache(Data_T& data,/*Plane_T const& plane,*/ /*Sphere_T& sphere,*/ Params_T const& params)
    {
        auto const proc_kernel = [&params](auto& plane) {
        auto const& [F, dt,t_0]             = params;
        auto& [pos, vel, out_a, out_ds,t_c] = data;
        };
        std::ranges::for_each(plane, proc_kernel);
        std::tuple <types::Vector3, types::Vector3> outputs = std::tie(out_ds,out_a);
        outputs = mechanics::computeLinearTrajectory(vel,F,dt);

        // Move to different process
        out_ds = std::get<0>(outputs);
        out_a = std::get<1>(outputs);
        t_c = defineTime(params);
    }
template <typename Intersect_T,typename Plane_T , typename Sphere_T>
void detectingCollision(Intersect_T const& data,types::Duration timestep)
    {
            auto const proc_kernel = [&intersection](auto& data){

                auto const& [planePos, n] = plane;
                auto& [spherePos,vel, r, ds] = sphere;
                auto& [is_collision,col_tp] = status;

            };
                   std::ranges::for_each(plane, proc_kernel);
            foreach (sphere spheres) {
            foreach (plane, planes) {
                if(mechanics::detectCollisionSphereFixedPlane(t_c,spherePos,r,vel,planePos,n,a,t_0,timestep))
                {
                    solver_types::IntersectDetProcDataBlock intersect;
                    intersect.plane = plane;
                    intersect.sphere = sphere;
                    intersect.status.is_collision = true;
                    intersect.status.col_tp = mechanics::detectCollisionSphereFixedPlane(t_c,spherePos,r,vel,planePos,n,a,t_0,timestep);


                }
            }

        }

            }
template<typename Param_T,>
void defineTime(Param_T params)
    {
        auto now = types::HighResolutionClock.now();
        foreach (sphere, params) {
            sphere.t_c = now;
        }
    }

  template <concepts::SolverFixtureLevel2 Fixture_T>
  void solve([[maybe_unused]] Fixture_T&         scenario,
             [[maybe_unused]] types::NanoSeconds timestep)
  {
      solver_types::Params params;
      params.F = scenario.m_forces;
      params.timestep = utils::toDuration(timestep);

      computeCache(scenario.m_backend.m_cache_data, params,scenario.m_backend.m_sphere_data);
      detectingCollision(scenario.m_backend.IntersectDetProcData,timestep);
  }
}
            // namespace dte3607::physengine::solver_dev::level2


#endif // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H
