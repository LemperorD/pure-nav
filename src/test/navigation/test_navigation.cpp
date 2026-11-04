#include "pure_nav/navigation.hpp"
#include "pure_nav/simulator.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace pure_nav;
TEST(Grid, BoundsAndFootprint) {
    Grid grid(20, 20, .1);
    grid.mark(10, 10);
    EXPECT_TRUE(grid.blocked(-.01, 1));
    EXPECT_TRUE(grid.blocked(2, 1));
    auto inflated = grid.inflated(.2);
    EXPECT_TRUE(inflated.blocked(.9, 1));
    EXPECT_FALSE(grid.segment({.5, .5, 0}, {1.5, 1.5, 0}));
    EXPECT_TRUE(grid.segment({.2, .2, 0}, {.2, 1.8, 0}));
    EXPECT_THROW(Grid(0, 2, .1), std::invalid_argument);
}
TEST(Planner, WallAndInvalidInputs) {
    auto map = demoWorld().inflated(.22);
    auto path = plan(map, {1.5, 1.5, 0}, {8.5, 6.5, 0});
    ASSERT_FALSE(path.empty());
    for (std::size_t i = 1; i < path.size(); ++i)
        EXPECT_TRUE(map.segment(path[i - 1], path[i]));
    EXPECT_TRUE(plan(demoWorld(true), {1.5, 1.5, 0}, {8.5, 6.5, 0}).empty());
    EXPECT_TRUE(plan(map, {-1, 1, 0}, {8, 6, 0}).empty());
    EXPECT_TRUE(plan(map, {1, 1, 0}, {std::numeric_limits<double>::quiet_NaN(), 1, 0}).empty());
}
TEST(Localization, CorrectsOffsetWithoutTruthInput) {
    auto map = demoWorld();
    Simulator sim(map, {1.5, 1.5, .1});
    auto all = sim.scan();
    PointCloud scan;
    for (auto p : all.points)
        if (p.z > .1)
            scan.push_back(p);
    ScanLocalizer localization({1.58, 1.44, .12});
    ASSERT_TRUE(localization.correct(scan, map));
    EXPECT_LT(distance(localization.pose(), sim.truth()), .1);
    EXPECT_FALSE(localization.correct({}, map));
    EXPECT_FALSE(localization.healthy());
}
TEST(Terrain, GroundRejectionAndObstacleHeight) {
    PerFrameHeightConfig config;
    config.min_block_point_num = 3;
    terrain_analysis::PerFrameHeightMap terrain(config);
    PointCloud cloud;
    for (int x = -5; x <= 5; ++x)
        for (int y = -5; y <= 5; ++y)
            cloud.push_back({x * .02F, y * .02F, 0, 0});
    cloud.push_back({0, 0, .3F, 0});
    cloud.push_back({0, 0, 1.F, 0});
    terrain.compute(cloud, {0, 0, .23});
    ASSERT_EQ(terrain.obstacleCloud().size(), 1u);
    EXPECT_NEAR(terrain.obstacleCloud().points.front().intensity, .3, 1e-5);
    PersistentVoxelConfig vc;
    vc.decay_time = .5;
    terrain_analysis::PersistentVoxelMap voxels(vc);
    voxels.receiveFrame(cloud, {0, 0, .23}, 0);
    voxels.update();
    PointCloud out;
    voxels.collectCloud(out);
    EXPECT_FALSE(out.empty());
    voxels.receiveFrame({}, {0, 0, .23}, 1);
    voxels.update();
    voxels.collectCloud(out);
    EXPECT_TRUE(out.empty());
}
TEST(Controller, EmptyPathAndCollisionStop) {
    Pursuit controller;
    auto map = demoWorld();
    auto v = controller.command({1, 1, 0}, {}, map);
    EXPECT_EQ(v.x, 0);
    EXPECT_EQ(v.y, 0);
    map.mark(11, 10);
    v = controller.command({1.05, 1.05, 0}, {{1.5, 1.05, 0}}, map);
    EXPECT_EQ(v.x, 0);
    EXPECT_THROW(Pursuit(0), std::invalid_argument);
}
TEST(Navigation, GameGateStaleAndMissingScan) {
    auto map = demoWorld();
    simple_decision::ContextConfig cfg;
    cfg.default_x = 8;
    cfg.default_y = 6;
    cfg.require_game_running = true;
    cfg.start_delay_sec = 0;
    Navigation nav(map, {1.5, 1.5, 0}, cfg);
    Simulator sim(map, {1.5, 1.5, 0});
    simple_decision::RobotStatus status;
    status.current_hp = 400;
    status.projectile_allowance_17mm = 100;
    auto result = nav.tick(sim.scan(), {}, 0, status);
    EXPECT_EQ(result.command.x, 0);
    EXPECT_FALSE(result.planned);
    simple_decision::GameStatus game;
    game.game_progress = game.RUNNING;
    nav.onGameStatus(game, 0);
    result = nav.tick(sim.scan(), {}, .05, status);
    EXPECT_TRUE(result.planned);
    result = nav.tick(sim.scan(), {}, .05, status);
    EXPECT_EQ(result.command.x, 0);
    EXPECT_FALSE(result.planned);
    result = nav.tick({}, {}, .1, status);
    EXPECT_EQ(result.command.x, 0);
    EXPECT_FALSE(result.localized);
    result = nav.tick(sim.scan(), {}, 1, status);
    EXPECT_EQ(result.command.x, 0);
    EXPECT_FALSE(result.planned);
}

#include "pure_nav/map_io.hpp"
#include <filesystem>
#include <fstream>
TEST(MapIO, OriginalMapAndOrigin) {
    auto map = loadMap(std::string(PURE_NAV_SOURCE_DIR) + "/assets/maps/rmul_2025.yaml");
    EXPECT_NEAR(map.resolution, .05, 1e-9);
    EXPECT_NEAR(map.origin_x, -1.25, 1e-9);
    EXPECT_NEAR(map.origin_y, -7.55, 1e-9);
    const auto center = map.center(3, 7);
    EXPECT_EQ(map.cellX(center.x), 3);
    EXPECT_EQ(map.cellY(center.y), 7);
    EXPECT_TRUE(map.blocked(map.origin_x - .01, map.origin_y));
}
TEST(MapIO, PixelOrientationUnknownAndTruncation) {
    auto dir = std::filesystem::temp_directory_path() / "pure_nav_map_test";
    std::filesystem::create_directories(dir);
    {
        std::ofstream f(dir / "map.yaml");
        f << "image: map.pgm\nmode: trinary\nresolution: 1\norigin: [-2, -3, 0]\nnegate: 0\noccupied_thresh: "
             "0.65\nfree_thresh: 0.25\n";
    }
    {
        std::ofstream f(dir / "map.pgm");
        f << "P2\n# fixture\n2 2\n255\n0 255\n128 255\n";
    }
    auto map = loadMap((dir / "map.yaml").string());
    EXPECT_EQ(map.at(0, 1), 254);
    EXPECT_EQ(map.at(0, 0), 255);
    EXPECT_EQ(map.at(1, 0), 0);
    {
        std::ofstream f(dir / "map.pgm");
        f << "P5\n2 2\n255\n";
    }
    EXPECT_THROW(loadMap((dir / "map.yaml").string()), std::runtime_error);
    std::filesystem::remove_all(dir);
}

#include "jps_planner/jps_algorithm.hpp"
TEST(Planner, NullCostmapIsRejected) {
    jps_planner::JPSState state;
    state.size_x = 10;
    state.size_y = 10;
    std::vector<std::pair<double, double>> path;
    EXPECT_FALSE(jps_planner::JPSAlgorithm::generatePath({}, state, 1, 1, 2, 2, path));
}
TEST(Navigation, AttackExpiryAndSupplyPriority) {
    auto map = demoWorld();
    Simulator sim(map, {1.5, 1.5, 0});
    simple_decision::ContextConfig cfg;
    cfg.default_x = 8;
    cfg.default_y = 6;
    Navigation nav(map, {1.5, 1.5, 0}, cfg);
    simple_decision::RobotStatus status;
    status.current_hp = 400;
    status.projectile_allowance_17mm = 100;
    nav.onTarget({2, 2, 0}, 0);
    auto out = nav.tick(sim.scan(), {}, 0, status);
    EXPECT_EQ(out.state, simple_decision::State::ATTACK);
    status.current_hp = 80;
    out = nav.tick(sim.scan(), {}, .05, status);
    EXPECT_EQ(out.state, simple_decision::State::SUPPLY);
    status.current_hp = 400;
    for (int i = 2; i <= 12; ++i)
        out = nav.tick(sim.scan(), {}, i * .05, status);
    EXPECT_EQ(out.state, simple_decision::State::DEFAULT);
}
