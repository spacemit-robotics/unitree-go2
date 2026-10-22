#include "controller.hpp"
#include "support.hpp"
#include <iostream>

using go2::Mode;
namespace {
void start(go2::Controller &c, Mode mode) {
    assert(c.set_mode(mode, fresh(1), at(1)).accepted);
    assert(c.status().mode == Mode::kStand);
    assert(!c.prepare_policy(fresh(1.5), at(1.5)));
    c.tick(fresh(1.5), at(1.5));
    c.tick(fresh(2.01), at(2.01));
    assert(c.status().mode == mode);
}
void frame(go2::Controller &c, double time, bool tilted = false) {
    auto s = fresh(time);
    if (tilted)
        s.imu.six_axis_quaternion = {.70710678F, 0, .70710678F, 0};
    auto r = c.prepare_policy(s, at(time));
    assert(r);
    c.complete_policy(*r, {}, at(time));
    c.tick(s, at(time));
}
} // namespace
int main() {
    const auto sit_trajectory = go2::load_sit_reference(
        std::string(GO2_TEST_APP) + "/resources/motions/12.go2_sit.motion");
    go2::ControllerConfig config = test_controller_config();
    config.sit_reference = sit_trajectory;
    assert(go2::parse_mode("crouch") == Mode::kCrouch);
    assert(go2::parse_mode("sit") == Mode::kSit);
    assert(go2::parse_mode("forward_jump") == Mode::kForwardJump);
    assert(std::string(go2::mode_name(Mode::kCrouch)) == "crouch");
    {
        go2::Controller c(config);
        assert(c.set_mode(Mode::kCrouch, fresh(1), at(1)).accepted);
        assert(!c.prepare_policy(fresh(2), at(2)));
        assert(c.status().mode == Mode::kCrouch);
    }
    // A queued switch cannot interrupt the jump's 120 accepted control frames.
    {
        go2::Controller c(config);
        start(c, Mode::kForwardJump);
        for (unsigned n = 0; n < 120; ++n) {
            double t = 2.02 + n * .02;
            if (n == 10)
                assert(c.set_mode(Mode::kRun, fresh(t), at(t)).accepted);
            if (n == 60)
                assert(c.set_mode(Mode::kSit, fresh(t), at(t)).accepted);
            frame(c, t);
            assert(c.status().mode == Mode::kForwardJump);
        }
        assert(!c.prepare_policy(fresh(4.43), at(4.43)));
        assert(c.status().mode == Mode::kStand);
        c.tick(fresh(5.44), at(5.44));
        assert(c.status().mode == Mode::kSit);
        auto r = c.prepare_policy(fresh(5.45), at(5.45));
        assert(r && r->policy_frame == 0 && r->observation.size() == 56);
    }
    // Different normal destinations all wait for the sitting policy to rise.
    for (auto destination :
        {Mode::kWalk, Mode::kRun, Mode::kForwardJump, Mode::kStand, Mode::kCrouch, Mode::kRollover}) {
        go2::Controller c(config);
        start(c, Mode::kSit);
        for (unsigned n = 0; n < 200; ++n)
            frame(c, 2.02 + n * .02);
        assert(c.status().mode == Mode::kSit); // No automatic four-second stand.
        auto old = c.prepare_policy(fresh(6.02), at(6.02));
        assert(old);
        auto requested_state = fresh(6.02);
        if (destination == Mode::kRollover) {
            assert(!c.set_mode(destination, requested_state, at(6.02)).accepted);
            requested_state.imu.quaternion = {0.819152F, 0.573576F, 0.0F, 0.0F};
            requested_state.imu.six_axis_quaternion = requested_state.imu.quaternion;
        }
        assert(c.set_mode(destination, requested_state, at(6.02)).accepted);
        c.complete_policy(*old, {}, at(6.02));
        auto first = c.prepare_policy(fresh(6.02), at(6.02));
        assert(first && first->generation != old->generation && first->policy_frame == 200);
        assert(first->observation[55] < .001F);
        for (unsigned n = 0; n < 169; ++n) {
            frame(c, 6.02 + n * .02);
            assert(c.status().mode == Mode::kSit);
        }
        for (unsigned n = 169; n < 195 && c.status().mode == Mode::kSit; ++n)
            frame(c, 6.02 + n * .02);
        if (destination == Mode::kCrouch || destination == Mode::kRollover) {
            assert(c.status().mode == destination);
        } else {
            assert(c.status().mode == Mode::kStand);
            c.tick(fresh(11.2), at(11.2));
            assert(c.status().mode == destination);
        }
    }
    // Repeated selection replaces the destination without restarting recovery.
    {
        go2::Controller c(config);
        start(c, Mode::kSit);
        for (unsigned n = 0; n < 100; ++n)
            frame(c, 2.02 + n * .02);
        assert(c.set_mode(Mode::kRun, fresh(4.02), at(4.02)).accepted);
        for (unsigned n = 0; n < 50; ++n)
            frame(c, 4.02 + n * .02);
        auto before = c.prepare_policy(fresh(5.02), at(5.02));
        assert(before);
        assert(c.set_mode(Mode::kWalk, fresh(5.02), at(5.02)).accepted);
        auto after = c.prepare_policy(fresh(5.02), at(5.02));
        assert(after);
        near(before->observation[55], after->observation[55]);
        assert(after->observation[55] > .1F && before->generation == after->generation);
        for (unsigned n = 50; n < 195 && c.status().mode == Mode::kSit; ++n)
            frame(c, 4.02 + n * .02);
        assert(c.status().mode == Mode::kStand);
        c.tick(fresh(9.2), at(9.2));
        assert(c.status().mode == Mode::kWalk);
    }
    // Failed recovery must not start the requested running model.
    {
        go2::Controller c(config);
        start(c, Mode::kSit);
        frame(c, 2.02);
        assert(c.set_mode(Mode::kRun, fresh(2.04), at(2.04)).accepted);
        for (unsigned n = 0; n < 405 && !c.status().protective_crouch; ++n)
            frame(c, 2.04 + n * .02, true);
        assert(c.status().protective_crouch);
        assert(c.status().reason.find("target mode cancelled") != std::string::npos);
        c.tick(fresh(11.2), at(11.2));
        assert(c.status().mode == Mode::kPassive);
    }
    // Exit during descent finishes sitting before starting the learned recovery.
    {
        go2::Controller c(config);
        start(c, Mode::kSit);
        frame(c, 2.02);
        assert(c.set_mode(Mode::kRun, fresh(2.04), at(2.04)).accepted);
        for (unsigned n = 0; n < 268; ++n) {
            frame(c, 2.04 + n * .02);
            assert(c.status().mode == Mode::kSit);
        }
        for (unsigned n = 268; n < 295 && c.status().mode == Mode::kSit; ++n)
            frame(c, 2.04 + n * .02);
        assert(c.status().mode == Mode::kStand);
        c.tick(fresh(9.1), at(9.1));
        assert(c.status().mode == Mode::kRun);
    }
    // Normal sleep finishes existing sitting/jump recovery, then crouches and brakes.
    for (auto mode : {Mode::kSit, Mode::kForwardJump}) {
        go2::Controller c(config);
        start(c, mode);
        frame(c, 2.02);
        assert(c.set_mode(Mode::kPassive, fresh(2.04), at(2.04), true).accepted);
        assert(c.status().mode == mode && !c.status().protective_crouch);
        double time = 2.04;
        for (unsigned n = 0; n < 500 && c.status().mode == mode; ++n, time += .02) {
            if (n == 10)
                assert(c.set_mode(Mode::kPassive, fresh(time), at(time), true).accepted);
            if (auto r = c.prepare_policy(fresh(time), at(time)))
                c.complete_policy(*r, {}, at(time));
            c.tick(fresh(time), at(time));
        }
        assert(c.status().protective_crouch);
        brake(c.tick(fresh(time + 1.01), at(time + 1.01)));
        assert(c.status().mode == Mode::kPassive);
    }
    // Passive and hard motor faults retain priority over both skill exits.
    for (auto mode : {Mode::kSit, Mode::kForwardJump}) {
        go2::Controller c(config);
        start(c, mode);
        frame(c, 2.02);
        c.set_mode(Mode::kRun, fresh(2.04), at(2.04));
        assert(c.set_mode(Mode::kPassive, fresh(2.05), at(2.05)).accepted);
        assert(c.status().protective_crouch);
        c.tick(fresh(3.1), at(3.1));
        assert(c.status().mode == Mode::kPassive);
        go2::Controller fault(config);
        start(fault, mode);
        frame(fault, 2.02);
        auto bad = fresh(2.04);
        bad.motors.joints[0].feedback.error = 1;
        assert(fault.tick(bad, at(2.04))[0].protective_crouch);
        assert(fault.status().protective_crouch);
        brake(fault.tick(bad, at(3.05)));
        assert(fault.status().mode == Mode::kPassive);
    }
    std::cout << "PASS: skill transitions, exits, recovery timeouts\n";
}
