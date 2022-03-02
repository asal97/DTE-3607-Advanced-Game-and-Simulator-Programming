// google benchmark
#include <benchmark/benchmark.h>

// stl
#include <algorithm>
#include <limits>
#include <memory>
#include <numeric>
#include <random>

#include <physengine/solvers/solver_dev_level2.h>
#include <physengine/bits/fixtures.h>
#include <physengine/bits/types.h>

using namespace std::chrono_literals;

namespace dte3607::benchmarking::predef
{
  size_t Loops = 10;
  struct GoldDummyBenchmarkF : benchmark::Fixture {

    using benchmark::Fixture::Fixture;
    std::unique_ptr<dte3607::physengine::fixtures::FixtureLevel2> m_scenario;
    ~GoldDummyBenchmarkF() override {}

    void SetUp(benchmark::State const&) final
    {
      // Create Fixture
      m_scenario
        = std::make_unique<dte3607::physengine::fixtures::FixtureLevel2>();


      // Set external forces
      m_scenario->setGravity({0, 0, 0});


      // make plane
      m_scenario->createFixedInfPlane({-1, 0, 0}, {10, 0, 0});

      m_scenario->createFixedInfPlane({1, 0, 0}, {-10, 0, 0});


      // make sphere
      for (int i = 0; i < Loops; i++) {
        m_scenario->createSphere(1.0, {100, 0, 0}, {2, 0, i * 2.0});
        m_scenario->createSphere(1.0, {-100, 0, 0}, {-2, 0, i * 2.0});
        m_scenario->createSphere(1.0, {100, 0, 0}, {1, 0, i * 2.0 + 2});
        m_scenario->createSphere(1.0, {-100, 0, 0}, {-1, 0, i * 2.0 + 2});
      }
    }
    void TearDown(benchmark::State const&) final { m_scenario.release(); }
  };


}   // namespace dte3607::benchmarking::predef




// Qualify predefined fixtures
using namespace dte3607::benchmarking::predef;

// Dummy benchmarks
BENCHMARK_DEFINE_F(GoldDummyBenchmarkF, ms16)
(benchmark::State& st)
{
  for ([[maybe_unused]] auto const& _ : st)
    dte3607::physengine::solver_dev::level2::solve(*m_scenario, 16ms);
}

// Dummy benchmarks
BENCHMARK_DEFINE_F(GoldDummyBenchmarkF, ms30)
(benchmark::State& st)
{
  for ([[maybe_unused]] auto const& _ : st)
    dte3607::physengine::solver_dev::level2::solve(*m_scenario, 30ms);
}


BENCHMARK_REGISTER_F(GoldDummyBenchmarkF, ms16);

BENCHMARK_REGISTER_F(GoldDummyBenchmarkF, ms30);

BENCHMARK_MAIN();
