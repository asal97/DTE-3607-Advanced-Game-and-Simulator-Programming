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
  using CacheProcDataBlock = solver_types::CacheProcDataBlock;
  using CacheProcData      = std::vector<CacheProcDataBlock>;
  using Vector3            = types::Vector3;

  struct BackendFixture {
    CacheProcData                      m_cache_data;
    std::unordered_map<size_t, size_t> m_rb_cache;
  };

  template <typename Fixture_T>
  void initComputationalWorld(Fixture_T const& f, BackendFixture& bf)
  {
    size_t id = 0ul;

    for (auto const& rb : f.m_rigid_bodies) {
      auto current_id         = id++;
      auto constexpr NonFixed = Fixture_T::RBMode::NonFixed;
      if (rb->mode() not_eq NonFixed) continue;

      bf.m_cache_data.emplace_back(rb->globalFramePosition(), rb->velocity(),
                                   Vector3{0, 0, 0}, Vector3{0, 0, 0});

      // [cd1, cd2, cd3]
      // [plane, sphere, sphere, shpere]
      bf.m_rb_cache.emplace(bf.m_cache_data.size() - 1, current_id);
    }
  }
}   // namespace dte3607::physengine::backend

#endif   // DTE3607_PHYSENGINE_BACKEND_FIXTURES_H
