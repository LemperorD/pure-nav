#include "pure_nav/map_io.hpp"
#include "pure_nav/navigation.hpp"
#include "pure_nav/simulator.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
int runSimulation(int argc, char **argv) {
    try {
        std::string scenario = "navigation", output = "trajectory.csv", map_path;
        pure_nav::Pose initial{1.5, 1.5, 0}, goal{8.5, 6.5, 0};
        bool plan_only = false;
        auto parse_pose = [](const std::string &value) {
            std::istringstream input(value);
            pure_nav::Pose pose;
            char comma1{}, comma2{};
            if (!(input >> pose.x >> comma1 >> pose.y >> comma2 >> pose.yaw) || comma1 != ',' ||
                comma2 != ',' || !pure_nav::finite(pose))
                throw std::invalid_argument("pose must be x,y,yaw (metres/radians)");
            input >> std::ws;
            if (!input.eof())
                throw std::invalid_argument("trailing pose data");
            return pose;
        };
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--help") {
                std::cout << "pure_nav [--scenario navigation|supply|blocked|sensor-loss] [--output "
                             "file.csv] [--map map.yaml --start x,y,yaw --goal x,y,yaw --plan-only]\n";
                return 0;
            }
            if (arg == "--plan-only") {
                plan_only = true;
                continue;
            }
            if ((arg == "--scenario" || arg == "--output" || arg == "--map" || arg == "--start" ||
                 arg == "--goal") &&
                i + 1 < argc) {
                const std::string value = argv[++i];
                if (arg == "--scenario")
                    scenario = value;
                else if (arg == "--map")
                    map_path = value;
                else if (arg == "--start")
                    initial = parse_pose(value);
                else if (arg == "--goal")
                    goal = parse_pose(value);
                else
                    output = value;
            } else
                throw std::invalid_argument("unknown or incomplete option: " + arg);
        }
        if (scenario != "navigation" && scenario != "supply" && scenario != "blocked" &&
            scenario != "sensor-loss")
            throw std::invalid_argument("unknown scenario");
        auto world =
            map_path.empty() ? pure_nav::demoWorld(scenario == "blocked") : pure_nav::loadMap(map_path);
        if (plan_only) {
            const auto path = pure_nav::plan(world.inflated(.22), initial, goal);
            std::ofstream route(output);
            if (!route)
                throw std::runtime_error("cannot open route output");
            route << "x,y,yaw\n";
            for (const auto p : path)
                route << p.x << ',' << p.y << ',' << p.yaw << '\n';
            std::cout << "plan=" << (path.empty() ? "UNREACHABLE" : "PASS") << " points=" << path.size()
                      << " route=" << output << '\n';
            return path.empty() ? 1 : 0;
        }
        if (!map_path.empty())
            throw std::invalid_argument(
                "imported maps currently support --plan-only; closed-loop simulation uses demoWorld");
        pure_nav::Simulator sim(world, initial);
        simple_decision::ContextConfig config;
        config.default_x = goal.x;
        config.default_y = goal.y;
        config.default_yaw = goal.yaw;
        config.supply_x = 1.5;
        config.supply_y = 6.5;
        config.start_delay_sec = 0;
        pure_nav::Navigation nav(world, {initial.x + .03, initial.y - .02, initial.yaw + .01}, config);
        simple_decision::GameStatus game;
        game.game_progress = game.RUNNING;
        nav.onGameStatus(game, 0);
        simple_decision::RobotStatus status;
        status.current_hp = scenario == "supply" ? 80 : 400;
        status.projectile_allowance_17mm = 100;
        std::ofstream csv(output);
        if (!csv)
            throw std::runtime_error("cannot open output: " + output);
        csv << "time,true_x,true_y,estimate_x,estimate_y,vx,vy,localized,planned,state\n";
        bool arrived = false, stopped = true;
        double max_error = 0;
        int ticks = 0;
        for (int i = 0; i < 1600; ++i) {
            auto scan = sim.scan();
            if (scenario == "sensor-loss" && i >= 20)
                scan.clear();
            auto result = nav.tick(scan, sim.measured(), i * .05, status);
            if ((scenario == "blocked" || scenario == "sensor-loss") && (scenario == "blocked" || i >= 20))
                stopped = stopped && std::hypot(result.command.x, result.command.y) < 1e-10 &&
                          std::abs(result.command.yaw) < 1e-10;
            const auto truth = sim.truth();
            max_error = std::max(max_error, pure_nav::distance(truth, result.estimate));
            csv << i * .05 << ',' << truth.x << ',' << truth.y << ',' << result.estimate.x << ','
                << result.estimate.y << ',' << result.command.x << ',' << result.command.y << ','
                << result.localized << ',' << result.planned << ',' << static_cast<int>(result.state) << '\n';
            ++ticks;
            if (!sim.step(result.command, .05)) {
                std::cerr << "collision at tick " << i << '\n';
                return 2;
            }
            if (result.arrived) {
                arrived = true;
                break;
            }
            if ((scenario == "blocked" || scenario == "sensor-loss") && i >= 40)
                break;
        }
        const bool safety = scenario == "blocked" || scenario == "sensor-loss";
        const bool pass = (safety ? stopped : arrived) && max_error < .25;
        std::cout << "scenario=" << scenario << " result=" << (pass ? "PASS" : "FAIL")
                  << " arrived=" << arrived << " stopped=" << stopped << " ticks=" << ticks
                  << " max_localization_error=" << max_error << " trajectory=" << output << '\n';
        return pass ? 0 : 1;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 2;
    }
}
