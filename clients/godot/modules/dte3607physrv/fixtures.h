#ifndef FIXTURES_H
#define FIXTURES_H

#include "dte3607_types.h"

// DTE-3607 PhysEngine
#include <physengine/bits/types.h>
#include <physengine/bits/solver_types.h>
#include <physengine/bits/backend_fixture.h>

// stl
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>

namespace frb
{
  namespace types        = dte3607::physengine::types;
  namespace solver_types = dte3607::physengine::solver_types;
  namespace backend      = dte3607::physengine::backend;


  struct Fixture {

    /*** API concept required types ***/

    // Types
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;

    // Fixture types
    using Forces         = types::Vector3;
    using BackendFixture = backend::BackendFixture;

    // RigidBody types
    using RigidBody    = frb::RigidBody;
    using RigidBodyPtr = std::unique_ptr<RigidBody>;
    using RigidBodies  = std::vector<RigidBodyPtr>;

    using RBShape    = rb_shapes::Shape;
    using RBShapePtr = std::unique_ptr<RBShape>;
    using RBShapes   = std::vector<RBShapePtr>;

    using RBPart  = RigidBodyPart;
    using RBMode  = frb::RigidBody::Mode;
    using RBState = frb::RigidBody::State;

    /*** API concept required methods ***/

    // Global properties
    size_t noRigidBodies() const;
    Forces externalForces() const;

    void setGravity(Forces);

    // RB properties
    types::Point3  globalFramePosition(size_t rid) const;
    types::Vector3 globalVelocity(size_t rid) const;

    // Modes and states
    RigidBody::Mode mode(size_t rid) const;

    // Transform
    void translateParent(size_t rid, Vector3 lin_trajectory);
    void setVelocity(size_t rid, Vector3 velocity);
    void addAcceleration(size_t rid, Vector3 accel);

    // Construction methods
    size_t createSphere(ValueType radius = 1., Vector3 velocity = {0, 0, 0},
                        Vector3   translation   = {0, 0, 0},
                        ValueType friction_coef = 1.)
    {
      return {};
    }

    size_t createFixedInfPlane([[maybe_unused]] Vector3 normal      = {0, 1, 0},
                               [[maybe_unused]] Vector3 translation = {0, 0, 0})
    {
        return {};
    }
    /*** END API requirements ***/

    void updateComputationalWorld()
    {
      if (m_backend) m_backend.release();

      m_backend = std::make_unique<BackendFixture>();
      backend::initComputationalWorld(*this, *m_backend.get());
    }

    /*** persistent world ***/
    RigidBodies m_rigid_bodies;
    Forces      m_forces;
    RBShapes    m_rb_shapes;

    /*** computational world ***/
    std::unique_ptr<BackendFixture> m_backend{nullptr};
  };


}   // namespace frb



#endif   // FIXTURES_H
