#ifndef DTE3607_PHYSENGINE_FIXTURES_H
#define DTE3607_PHYSENGINE_FIXTURES_H

#include "types.h"
#include "solver_types.h"
#include <physengine/bits/backend_fixture.h>
// stl
#include <variant>
#include <unordered_map>

namespace dte3607::physengine::fixtures
{
  namespace backend = dte3607::physengine::backend;

  namespace rb_oop
  {

    struct SpaceObjectBase {

      // Concept requirements
      using ValueType   = types::Point3::ElementType;
      using Point3      = types::Point3;
      using Point3H     = types::Point3H;
      using Vector3     = types::Vector3;
      using Vector3H    = types::Vector3H;
      using SpaceObject = types::ProjectiveSpaceObject;
      using Timepoint   = types::HighResolutionTP;


      // Common properties
      SpaceObject m_object;

      // Lin. alg. prop. access
      virtual SpaceObject&       spaceObject();
      virtual SpaceObject const& spaceObject() const;

      typename SpaceObject::ASFrameH pFrame() const;

      typename SpaceObject::Frame vFrame() const;

      Point3 globalFramePosition() const;
    };


    namespace rb_shapes
    {
      enum class ShapeType { Sphere, InfPlane };

      struct Shape : SpaceObjectBase {
        Shape(ShapeType p_shape_type);
        ShapeType m_shape_type;
      };


      struct Sphere : Shape {
        Sphere(Vector3::ElementType const& radius = 1.);

        // Sphere prop. access
        Point3    point() const;
        ValueType radius() const;

      private:
        ValueType m_radius{1.0};
      };

      struct InfPlane : Shape {
        InfPlane(Vector3 const& n = {.0, 1., .0});

        // Plane prop. access
        Point3  point() const;
        Vector3 normal() const;

      private:
        Vector3 m_n{0, 1, 0};
      };

    }   // namespace rb_shapes



    struct RigidBodyPart : SpaceObjectBase {

      using Shape = rb_shapes::Shape;

      // Class type constants
      static constexpr ValueType frictionCoefMin{0.0};
      static constexpr ValueType frictionCoefMax{1.0};

      ValueType frictionCoef() const;
      void      setFrictionCoef(ValueType const& friction_coef);

      ValueType m_mass{1.0};

      Shape* shape();
      Shape* m_shape;

    private:
      ValueType m_friction_coef{0.0};   // 0 == no friction
    };

    struct RigidBody : SpaceObjectBase {

      RigidBody(const std::string& name = "");

      // Concept requirements
      using ValueType        = types::Point3::ElementType;
      using Point3           = types::Point3;
      using Point3H          = types::Point3H;
      using Vector3          = types::Vector3;
      using Vector3H         = types::Vector3H;
      using SpaceObjectFrame = types::ProjectiveSpaceObject;
      using Timepoint        = types::HighResolutionTP;
      using Mode             = types::RBMode;
      using State            = types::RBState;


      // Mechanics property access
      Vector3 velocity() const;

      /** a; in the "parent" spacial frame */
      void addAcceleration(types::Vector3 const& a);

      /** v; in the "parent" spacial frame */
      void setVelocity(types::Vector3 const& v);


      // Mode
      Mode mode() const;
      void setMode(Mode p_mode);

      // States
      State state() const;
      void  setState(State p_state);

    private:
      Timepoint m_timepoint;
      Vector3   m_velocity{0, 0, 0};
      Mode      m_mode{Mode::NonFixed};
      State     m_state{State::Free};


    public:
      std::string m_name;

      using Part    = RigidBodyPart;
      using PartPtr = std::unique_ptr<RigidBodyPart>;
      using Parts   = std::vector<PartPtr>;

      Parts m_parts;
    };

  }   // namespace rb_oop

  struct FixtureLevel1 {

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
    using RigidBody    = rb_oop::RigidBody;
    using RigidBodyPtr = std::unique_ptr<RigidBody>;
    using RigidBodies  = std::vector<RigidBodyPtr>;

    using RBShape    = rb_oop::rb_shapes::Shape;
    using RBShapePtr = std::unique_ptr<RBShape>;
    using RBShapes   = std::vector<RBShapePtr>;

    using RBPart  = rb_oop::RigidBodyPart;
    using RBMode  = RigidBody::Mode;
    using RBState = RigidBody::State;

    /*** API concept required methods ***/

    // Global properties
    size_t noRigidBodies() const { return m_rigid_bodies.size(); }
    void   setGravity([[maybe_unused]] Forces G) { m_forces = G; }
    void   translateParent(size_t rid, Vector3 lin_trajectory)
    {
      m_rigid_bodies[rid]->m_object.translateParent(lin_trajectory);
    }
    void addAcceleration(size_t rid, Vector3 a)
    {
      m_rigid_bodies[rid]->addAcceleration(a);
    }


    // RB properties
    types::Point3 globalFramePosition([[maybe_unused]] size_t rid) const
    {
      return m_rigid_bodies[rid]->globalFramePosition();
    }

    types::Vector3 globalVelocity(size_t rid) const;

    // Modes and states
    RigidBody::Mode mode(size_t rid) const
    {
      return m_rigid_bodies[rid]->mode();
    }

    /*** Fixture unit-test setup API ***/
    size_t createSphere([[maybe_unused]] ValueType radius,
                        [[maybe_unused]] Vector3   velocity,
                        [[maybe_unused]] Vector3   translation)
    {
      // Create Rigid body
      m_rigid_bodies.emplace_back(std::make_unique<RigidBody>());
      auto&      rb  = m_rigid_bodies.back();
      auto const rbi = m_rigid_bodies.size() - 1;
      rb->m_object.translateParent(translation);
      rb->setMode(RigidBody::Mode::NonFixed);
      rb->setVelocity(velocity);

      // Create Shape
      m_rb_shapes.emplace_back(
        std::make_unique<rb_oop::rb_shapes::Sphere>(radius));
      auto* sphere = m_rb_shapes.back().get();

      // Create RB body part and add sphere as shape to body part
      rb->m_parts.emplace_back(std::make_unique<RBPart>());
      auto& rbp    = rb->m_parts.back();
      rbp->m_shape = sphere;

      updateComputationalWorld();

      return rbi;
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



  struct FixtureLevel2 {

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
    using Forces = types::Vector3;


    /*** API concept required methods ***/

    // Environment
    void setGravity(Forces) {}

    // RBs
    size_t              noRigidBodies() const { return {}; }
    std::vector<size_t> nonFixedSphereRBs() const { return {}; }
    std::vector<size_t> fixedInfPlaneRBs() const { return {}; }

    ValueType rbSphereRadius([[maybe_unused]] size_t s_rid) const { return {}; }
    Vector3   rbPlaneNormal([[maybe_unused]] size_t p_rid) const { return {}; }

    // RB properties
    types::Point3 globalFramePosition([[maybe_unused]] size_t rid) const
    {
      return {};
    }


    /*** Fixture unit-test setup API ***/

    size_t createSphere([[maybe_unused]] ValueType radius      = 1.,
                        [[maybe_unused]] Vector3   velocity    = {0, 0, 0},
                        [[maybe_unused]] Vector3   translation = {0, 0, 0})
    {
      return {};
    }

    size_t createFixedInfPlane([[maybe_unused]] Vector3 normal      = {0, 1, 0},
                               [[maybe_unused]] Vector3 translation = {0, 0, 0})
    {
      return {};
    }

    /*** END API requirements ***/
  };


  struct FixtureLevel3 {

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
    using Forces = types::Vector3;

    // RigidBody types
    using RBState = types::RBState;


    /*** API concept required methods ***/

    // Environment
    void setGravity(Forces) {}

    ValueType rbPlaneMaxFrictionCoef() const { return {}; }
    ValueType rbSphereMaxFrictionCoef() const { return {}; }

    RBState rbState([[maybe_unused]] size_t rid) const { return {}; }


    /*** Fixture unit-test setup API ***/

    size_t createSphere([[maybe_unused]] ValueType radius      = 1.,
                        [[maybe_unused]] Vector3   velocity    = {0, 0, 0},
                        [[maybe_unused]] Vector3   translation = {0, 0, 0},
                        [[maybe_unused]] RBState initial_state = RBState::Free,
                        [[maybe_unused]] ValueType friction_coef = 0.,
                        [[maybe_unused]] ValueType mass          = 1.)
    {
      return {};
    }

    size_t createFixedInfPlane([[maybe_unused]] Vector3 normal      = {0, 1, 0},
                               [[maybe_unused]] Vector3 translation = {0, 0, 0},
                               [[maybe_unused]] ValueType friction_coef = 0.)
    {
      return {};
    }

    /*** END API requirements ***/
  };


  struct FixtureLevel4 {

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
    using Forces = types::Vector3;

    // RigidBody types
    using RBState = types::RBState;



    /*** END API requirements ***/
  };

  /****
   * Implementations
   */




  namespace rb_oop
  {

    inline SpaceObjectBase::SpaceObject& SpaceObjectBase::spaceObject()
    {
      return m_object;
    }

    inline const SpaceObjectBase::SpaceObject&
    SpaceObjectBase::spaceObject() const
    {
      return m_object;
    }

    inline SpaceObjectBase::SpaceObject::ASFrameH
    SpaceObjectBase::pFrame() const
    {
      return spaceObject().pSpaceFrameParent();
    }

    inline SpaceObjectBase::SpaceObject::Frame SpaceObjectBase::vFrame() const
    {
      return spaceObject().vSpaceFrameParent();
    }

    inline SpaceObjectBase::Point3 SpaceObjectBase::globalFramePosition() const
    {
      return m_object.frameOriginParent();
    }

    inline SpaceObjectBase::ValueType RigidBodyPart::frictionCoef() const
    {
      return m_friction_coef;
    }

    inline void RigidBodyPart::setFrictionCoef(const ValueType& friction_coef)
    {
      m_friction_coef
        = std::clamp(friction_coef, frictionCoefMin, frictionCoefMax);
    }

    inline RigidBodyPart::Shape* RigidBodyPart::shape() { return m_shape; }

    inline RigidBody::RigidBody(const std::string& name) : m_name{name} {}

    inline RigidBody::Vector3 RigidBody::velocity() const { return m_velocity; }

    inline void RigidBody::addAcceleration(const types::Vector3& a)
    {
      m_velocity += a;
    }

    inline void RigidBody::setVelocity(const types::Vector3& v)
    {
      m_velocity = v;
    }

    inline RigidBody::Mode RigidBody::mode() const { return m_mode; }

    inline void RigidBody::setMode(Mode p_mode) { m_mode = p_mode; }

    inline RigidBody::State RigidBody::state() const { return m_state; }

    inline void RigidBody::setState(State p_state) { m_state = p_state; }


    namespace rb_shapes
    {
      inline Shape::Shape(ShapeType p_shape_type) : m_shape_type{p_shape_type}
      {
      }

      inline Sphere::Sphere(const Vector3::ElementType& radius)
        : Shape(ShapeType::Sphere), m_radius{radius}
      {
      }

      inline SpaceObjectBase::Point3 Sphere::point() const
      {
        return spaceObject().frameOriginParent();
      }

      inline SpaceObjectBase::ValueType Sphere::radius() const
      {
        return m_radius;
      }

      inline InfPlane::InfPlane(const Vector3& n)
        : Shape(ShapeType::InfPlane), m_n{n}
      {
      }

      inline SpaceObjectBase::Point3 InfPlane::point() const
      {
        return spaceObject().frameOriginParent();
      }

      inline SpaceObjectBase::Vector3 InfPlane::normal() const
      {
        return spaceObject().vSpaceFrameParent() * m_n;
      }

    }   // namespace rb_shapes



  }   // namespace rb_oop

}   // namespace dte3607::physengine::fixtures


#endif   // DTE3607_PHYSENGINE_FIXTURES_H
