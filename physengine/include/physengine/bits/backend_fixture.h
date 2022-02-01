#ifndef DTE3607_PHYSENGINE_BACKEND_FIXTURES_H
#define DTE3607_PHYSENGINE_BACKEND_FIXTURES_H

#include <unordered_map>

// DTE-3607 PhysEngine
#include <physengine/bits/types.h>
#include <physengine/bits/solver_types.h>

namespace dte3607::physengine::backend
{
  namespace types        = dte3607::physengine::types;
  namespace solver_types = dte3607::physengine::solver_types;

  // Solver types
  using CacheProcDataBlock    = solver_types::CacheProcDataBlock;
  using SphereGeomDataBlock   = solver_types::SphereGeomDataBlock;
  using InfPlaneGeomDataBlock = solver_types::InfPlaneGeomDataBlock;
  using CacheProcData         = std::vector<CacheProcDataBlock>;
  using SphereGeomData        = std::vector<SphereGeomDataBlock>;
  using InfPlaneGeomData      = std::vector<InfPlaneGeomDataBlock>;
  using Vector3               = types::Vector3;

  using IntersectDetProcDataBlock = solver_types::IntersectDetProcDataBlock;
  using IntersectDetProcData      = std::vector<IntersectDetProcDataBlock>;

  struct BackendFixture {
    CacheProcData                      m_cache_data;
    SphereGeomData                     m_sphere_data;
    InfPlaneGeomData                   m_plane_data;
    IntersectDetProcData               m_intersect_data;
    std::unordered_map<size_t, size_t> m_rb_cache;
    std::unordered_map<size_t, size_t> m_rb_plane;
    std::unordered_map<size_t, size_t> m_rb_sphere;
  };


  void initSphere(BackendFixture& bf, Vector3 position, double radius,
                  Vector3 velocity, size_t rbi)
  {
    bf.m_cache_data.emplace_back(position, velocity, Vector3{0, 0, 0},
                                 Vector3{0, 0, 0},
                                 types::HighResolutionClock::now());
    bf.m_rb_cache.emplace(rbi, bf.m_cache_data.size() - 1);

    bf.m_sphere_data.emplace_back(position, radius, Vector3{0, 0, 0});
    bf.m_rb_sphere.emplace(rbi, bf.m_sphere_data.size() - 1);
  }


  void initPlane(BackendFixture& bf, Vector3 position, Vector3 normal,
                 size_t rbi)
  {
    bf.m_plane_data.emplace_back(position, normal);
    bf.m_rb_plane.emplace(rbi, bf.m_plane_data.size() - 1);
  }


}   // namespace dte3607::physengine::backend

#endif   // DTE3607_PHYSENGINE_BACKEND_FIXTURES_H
