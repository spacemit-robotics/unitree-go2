#include "controller.hpp"
#include "policy.hpp"
#include "rl_service.h"
#include "support.hpp"
#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <filesystem> // NOLINT(build/c++17): this package requires C++17.
#include <fstream>
#include <iostream>
#include <limits>

namespace fs = std::filesystem;

namespace {
struct TemporaryConfig {
    fs::path directory;
    TemporaryConfig() {
        auto pattern = (fs::temp_directory_path() / "go2-policy-XXXXXX").string();
        const auto path = mkdtemp(pattern.data());
        if (!path)
            throw std::runtime_error("cannot create temporary config directory");
        directory = path;
    }
    ~TemporaryConfig() {
        std::error_code error;
        fs::remove_all(directory, error);
    }
    std::string write(const YAML::Node &config) const {
        const auto path = directory / "policies.yaml";
        std::ofstream stream(path);
        stream << config;
        if (!stream)
            throw std::runtime_error("cannot write test configuration");
        return path.string();
    }
};
} // namespace

int main(int argc, char **argv) {
    assert(argc == 2);
    const auto models = fs::absolute(argv[1]);
    const auto source = fs::path(GO2_TEST_POLICY_CONFIG);
    const auto original = YAML::LoadFile(source.string());
    // Temporary configurations keep references to the application's resources.
    for (const auto &entry : original["go2_modes"]) {
        if (auto motion = entry.second["motion"])
            motion["path"] = (source.parent_path() / motion["path"].as<std::string>()).lexically_normal().string();
    }
    TemporaryConfig temp;


    const auto load = [&](const YAML::Node &document, const std::string &name) {
        return go2::LoadPolicyConfigFromYaml(temp.write(document), name, models.string());
    };
    auto flat = load(original, "walk");
    assert(flat.exec_cfg.runtime.provider == "cpu" && flat.exec_cfg.runtime.threads == 1);
    assert(flat.exec_cfg.model_path == (models / "1.go2_flat.onnx").string());
    assert(flat.exec_cfg.obs_segments.front().terms.size() == 6);
    assert(flat.exec_cfg.clip_actions.empty());

    const auto policies = [](YAML::Node document) { return document["rl_policy"]["onnx_infer"]["policies"]; };
    // Prove configuration changes reach both observation and motor command paths.
    {
        auto document = YAML::Clone(original);
        auto walk = policies(document)["walk"];
        walk["observation"]["segment_0_terms"] =
            YAML::Load("[command, gravity, ang_vel, dof_pos, dof_vel, last_action]");
        walk["observation"]["ang_vel_scale"] = .7;
        walk["observation"]["dof_pos_scale"] = 2.;
        walk["observation"]["dof_vel_scale"] = .3;
        walk["command"]["scale"] = YAML::Load("[3, 4, 5]");
        walk["policy_adapter"]["command_limits"] = YAML::Load("[-.2, .2, -.1, .1, -.3, .3]");
        walk["rl_default_pos"][3] = .4;
        walk["action_scale"] = .6;
        walk["kp"][0] = 33.;
        walk["kd"][0] = 1.7;
        const auto adapter = std::make_shared<go2::PolicyAdapter>(temp.write(document), models.string());
        auto state = fresh(2.02);
        state.imu.angular_velocity = {1, 2, 3};
        state.motors.joints[3].feedback.position_rad = .5;
        state.motors.joints[3].feedback.velocity_rad_s = 2.;
        const auto obs = adapter->build_observation(state, {1, 1, 1}, {}, go2::Mode::kWalk);
        near(obs[0], .6);
        near(obs[1], .4);
        near(obs[2], 1.5);
        near(obs[6], .7);
        near(obs[7], 1.4);
        near(obs[8], 2.1);
        near(obs[9], .2);
        near(obs[21], .6);
        go2::Action action{};
        action[0] = 1;
        const auto target = adapter->action_to_hardware_targets(action, go2::Mode::kWalk);
        near(target[3], 1.);
        near(adapter->policy_gains(go2::Mode::kWalk).kp[0], 33.);
        // No process-global mutation: the original adapter retains the original defaults.
        near(test_adapter().action_to_hardware_targets(action, go2::Mode::kWalk)[3], .35);
        auto config = test_controller_config();
        config.policy_adapter = adapter;
        go2::Controller controller(config);
        assert(controller.set_mode(go2::Mode::kStand, fresh(1), at(1)).accepted);
        controller.tick(fresh(2), at(2));
        assert(controller.set_mode(go2::Mode::kWalk, state, at(2.02)).accepted);
        controller.set_velocity({1, 1, 1}, at(2.02));
        const auto initial_request = controller.prepare_policy(state, at(2.02));
        assert(initial_request);
        near(initial_request->observation[0], 0.);
        const auto request = controller.prepare_policy(state, at(2.12));
        assert(request);
        near(request->observation[0], .6);
        controller.complete_policy(*request, action, at(2.12));
        const auto commands = controller.tick(state, at(2.12));
        near(commands[3].position_rad, 1.);
        near(commands[3].kp, 33.);
        near(commands[3].kd, 1.7);
    }
    for (const auto &[key, value] : std::vector<std::pair<std::string, YAML::Node>>{
            {"action_joint_index", YAML::Load("[0,0,2,3,4,5,6,7,8,9,10,11]")}, {"rl_default_pos", YAML::Load("[0,0]")},
            {"action_scale", YAML::Load("[.2,.3]")}, {"kp", YAML::Load("[-1,1,1,1,1,1,1,1,1,1,1,1]")},
            {"kd", YAML::Load("[.nan,1,1,1,1,1,1,1,1,1,1,1]")}}) {
        auto document = YAML::Clone(original);
        policies(document)["walk"][key] = value;
        rejects([&] { go2::PolicyAdapter adapter(temp.write(document), models.string()); });
    }
    for (const auto &[key, value] : std::vector<std::pair<std::string, YAML::Node>>{
            {"backend", YAML::Node("unsupported")}, {"backend", YAML::Node("mnn")},
            {"model_path", YAML::Node("missing.onnx")},
            {"runtime", YAML::Load("{threads: 0}")}, {"model_io", YAML::Load("{inputs: [], outputs: []}")}}) {
        auto document = YAML::Clone(original);
        policies(document)["walk"][key] = value;
        rejects([&] { (void)load(document, "walk"); });
    }
    {
        auto document = YAML::Clone(original);
        policies(document)["walk"]["observation"]["segment_0_terms"] =
            YAML::Load("[ang_vel, gravity, command, dof_pos, dof_vel]");
        const auto invalid = load(document, "walk");
        rl_policy::PolicyExecutor executor;
        rejects([&] { executor.Init(invalid.exec_cfg); });
    }
    {
        auto document = YAML::Clone(original);
        document["go2_modes"].remove("walk");
        rejects([&] { go2::Policy policy(models.string(), temp.write(document)); });
    }
    {
        auto document = YAML::Clone(original);
        document["rl_policy"]["rl_dt"] = .01;
        rejects([&] { go2::Policy policy(models.string(), temp.write(document)); });
    }

    {
        auto document = YAML::Clone(original);
        policies(document)["walk"]["model_path"] = "missing.onnx";
        rejects([&] { go2::Policy policy(models.string(), temp.write(document)); });
    }
    {
        auto document = YAML::Clone(original);
        document["go2_modes"]["walk"]["models"] = YAML::Load("[walk, walk]");
        rejects([&] { go2::Policy policy(models.string(), temp.write(document)); });
    }
    go2::Policy policy(models.string(), source.string());
    for (const auto &entry : original["go2_modes"]) {
        const auto mode = go2::parse_mode(entry.first.as<std::string>());
        assert(policy.has_model(mode));
        assert(fs::path(load(original, entry.first.as<std::string>()).exec_cfg.model_path).extension() == ".onnx");
    }
    std::cout << "PASS: YAML defaults, paths, dimensions, backends, variants and invalid inputs\n";
}
