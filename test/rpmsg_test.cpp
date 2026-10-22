/* SPDX-License-Identifier: Apache-2.0 */
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif
#include <assert.h>
#include <pthread.h>
#include <sys/socket.h>
#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
static int transport_fd = -1, endpoint_created, wrong_address, created_count;
static int leg_fds[4] = {-1, -1, -1, -1};
/* Emulate kernel sysfs and endpoint creation while using a real packet socket
* for the complete production open/HELLO/exchange/STOP path. */
static int fake_open(const char *path, int flags, ...) {
    (void)flags;
    const char *value = NULL;
    if (!strcmp(path, "/dev/rpmsg_ctrl0"))
        return dup(transport_fd);
    if (endpoint_created && leg_fds[0] >= 0) {
        static const char *names[] = {"go2-motor-rl\n", "go2-motor-rr\n", "go2-motor-fl\n", "go2-motor-fr\n"};
        for (unsigned leg = 0; leg < 4; ++leg) {
            char expected[128];
            snprintf(expected, sizeof(expected), "/sys/class/rpmsg/rpmsg%u/name", leg + 7);
            if (!strcmp(path, expected))
                value = names[leg];
            snprintf(expected, sizeof(expected), "/sys/class/rpmsg/rpmsg%u/src", leg + 7);
            static const char *src[] = {"1203\n", "1202\n", "1201\n", "1200\n"};
            if (!strcmp(path, expected))
                value = src[leg];
            snprintf(expected, sizeof(expected), "/sys/class/rpmsg/rpmsg%u/dst", leg + 7);
            static const char *dst[] = {"1103\n", "1102\n", "1101\n", "1100\n"};
            if (!strcmp(path, expected))
                value = dst[leg];
            snprintf(expected, sizeof(expected), "/dev/rpmsg%u", leg + 7);
            if (!strcmp(path, expected))
                return dup(leg_fds[leg]);
        }
    } else if (endpoint_created) {
        if (!strcmp(path, "/sys/class/rpmsg/rpmsg7/name"))
            value = "go2-motor-fr\n";
        if (!strcmp(path, "/sys/class/rpmsg/rpmsg7/src"))
            value = wrong_address ? "999\n" : "1200\n";
        if (!strcmp(path, "/sys/class/rpmsg/rpmsg7/dst"))
            value = "1100\n";
        if (!strcmp(path, "/dev/rpmsg7"))
            return dup(transport_fd);
    }
    if (value) {
        FILE *f = tmpfile();
        assert(f);
        int fd = dup(fileno(f));
        fclose(f);
        assert(write(fd, value, strlen(value)) == (ssize_t)strlen(value));
        assert(lseek(fd, 0, SEEK_SET) == 0);
        return fd;
    }
    errno = ENOENT;
    return -1;
}
static int fake_ioctl(int fd, unsigned long request, ...) {
    (void)fd;
    struct info {
        char name[32];
        uint32_t src, dst;
    };
    va_list args;
    va_start(args, request);
    struct info *info = va_arg(args, struct info *);
    va_end(args);
    assert(request == _IOW(0xb5, 1, struct info));
    assert(!strcmp(info->name, "go2-motor-fr") && info->src == 1200 && info->dst == 1100);
    endpoint_created = 1;
    created_count++;
    return 0;
}
#include <sys/file.h>
#include <sys/random.h>
#include <poll.h>
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <array>
#include "imu.h"
#include "motor_core.h"
#define open fake_open
#define ioctl fake_ioctl
// Include the private client to exercise production transport without exporting a test API.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsubobject-linkage"
#include "../src/hardware.cpp"
#pragma GCC diagnostic pop
#undef open
#undef ioctl
using go2::MotorMode;
using go2::RotorCommand;
using go2::RpmsgSession;
using go2::RpmsgSnapshot;
using go2::rpmsg_close;
using go2::rpmsg_endpoint;
using go2::rpmsg_exchange;
using go2::rpmsg_get;
using go2::rpmsg_milliseconds;
using go2::rpmsg_open;
using go2::rpmsg_set;

namespace {
std::array<std::atomic<unsigned>, 4> test_foc{}, test_gets{}, test_sets{};
std::atomic<bool> test_stale{false}, test_freeze{false};
std::atomic<unsigned> test_feedback_age{0};
std::atomic<bool> test_drop_get_reply{false};
std::atomic<bool> test_motion{false};
std::mutex target_mutex;
std::array<std::array<go2::RotorCommand, 3>, 4> test_targets{};
int imu_token;
imu_callback_t test_imu_callback;
void *test_imu_context;
template <typename Predicate> void wait_for(Predicate predicate, unsigned line = __builtin_LINE()) {
    const auto deadline = go2::SteadyClock::now() + std::chrono::seconds(2);
    while (!predicate() && go2::SteadyClock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const bool ready = predicate();
    if (!ready)
        fprintf(stderr, "RPMsg wait timed out at test line %u; FOC counts=%u/%u/%u/%u\n", line,
            test_foc[0].load(), test_foc[1].load(), test_foc[2].load(), test_foc[3].load());
    assert(ready);
}
bool all_at_least(const std::array<std::atomic<unsigned>, 4> &values, unsigned expected) {
    for (const auto &value : values)
        if (value.load() < expected)
            return false;
    return true;
}
} // namespace
extern "C" {
struct imu_dev *imu_alloc_spi(const char *, const char *, uint32_t, void *) {
    return reinterpret_cast<struct imu_dev *>(&imu_token);
}
struct imu_dev *imu_alloc_uart(const char *, const char *, uint32_t, void *) {
    return reinterpret_cast<struct imu_dev *>(&imu_token);
}
int imu_init(struct imu_dev *, const struct imu_config *) { return 0; }
int imu_read(struct imu_dev *, struct imu_data *) { return -EAGAIN; }
void imu_set_callback(struct imu_dev *, imu_callback_t callback, void *context) {
    test_imu_callback = callback;
    test_imu_context = context;
}
void imu_free(struct imu_dev *) {}
}

static void test_iwt603_quaternion() {
    go2::HardwareConfig config;
    config.backend = go2::Backend::kSerial;
    config.motor_io_authorized = config.actuation_enabled = true;
    config.hardware_configuration_verified = config.motor_ids_verified = true;
    config.joint_calibration_verified = config.suspended_test_verified = true;
    config.imu_device = "/dev/serial/by-id/GO2_TEST_IMU";
    for (unsigned leg = 0; leg < 4; ++leg) {
        config.legs[leg].device = "/dev/serial/by-id/GO2_TEST_MOTOR-" + std::to_string(leg);
        // An unregistered motor driver keeps this IMU-only case from opening serial ports.
        config.legs[leg].driver = "GO2_TEST_NO_MOTOR";
        config.legs[leg].reconnect_delay = std::chrono::milliseconds(1);
        for (unsigned id = 0; id < 3; ++id)
            config.legs[leg].calibration[id].motor_id = id;
    }
    go2::Hardware hardware(config);
    hardware.start();
    assert(test_imu_callback);
    struct imu_data data{};
    data.acc[2] = 9.80665F;
    data.gyro[0] = 1;
    data.quat[0] = data.quat[2] = std::sqrt(.5F);
    for (unsigned sample = 0; sample < 3; ++sample) {
        data.timestamp_us = 1000000U + sample * 10000U;
        test_imu_callback(nullptr, &data, test_imu_context);
        const auto imu = hardware.read().imu;
        assert(imu.valid && imu.sequence == data.timestamp_us * 1000U);
        for (unsigned component = 0; component < 4; ++component) {
            assert(std::fabs(imu.quaternion[component] - data.quat[component]) < 1e-6F);
            assert(std::fabs(imu.six_axis_quaternion[component] - data.quat[component]) < 1e-6F);
        }
    }
    // Missing/invalid sensor attitude must not fall back to host fusion.
    for (float invalid : {0.F, NAN, INFINITY}) {
        memset(data.quat, 0, sizeof(data.quat));
        data.quat[0] = invalid;
        test_imu_callback(nullptr, &data, test_imu_context);
        assert(!hardware.read().imu.valid);
    }
    data.quat[0] = 1;
    data.timestamp_us = 0;
    test_imu_callback(nullptr, &data, test_imu_context);
    assert(!hardware.read().imu.valid);
    hardware.stop();
    assert(!test_imu_callback);
}

struct server {
    int fd, malformed;
    unsigned sets, gets, stops;
    bool integration{}, active{};
    unsigned leg{}, sample{1};
    go2::TimePoint sampled_at{go2::SteadyClock::now()};
    go2::TimePoint motion_started{};
    float position{};
};
static void *serve(void *arg) {
    auto *s = static_cast<struct server *>(arg);
    uint8_t q[GO2_RPC_SIZE], r[GO2_RPC_SIZE];
    for (;;) {
        ssize_t size = read(s->fd, q, sizeof(q));
        if (size == 0)
            break;
        assert(size == GO2_RPC_SIZE && go2_rpc_valid(q, size));
        if (s->integration)
            assert(q[6] == 3U - s->leg);
        go2_rpc_header(r, q[5] | 0x80, q[6], go2_get32(q + 8), go2_get64(q + 12));
        if (q[5] == GO2_RPC_HELLO)
            s->active = false;
        if (q[5] == GO2_RPC_SET) {
            ++s->sets;
            s->active = false;
            for (unsigned id = 0; id < 3; ++id) {
                const uint8_t *t = q + 32 + id * 40;
                if (s->integration) {
                    std::lock_guard<std::mutex> lock(target_mutex);
                    auto &target = test_targets[s->leg][id];
                    target.mode = go2_get32(t) ? MotorMode::kFoc : MotorMode::kBrake;
                    target.torque_nm = go2_get_float(t + 4);
                    target.position_rad = go2_get_float(t + 12);
                    target.kp = go2_get_float(t + 16);
                    s->active = s->active || target.mode == MotorMode::kFoc;
                    continue;
                }
                assert(go2_get32(t) == 1);
                assert(go2_get_float(t + 4) == (float)id + 1);
                assert(go2_get_float(t + 12) == (float)id + 10);
            }
        }
        if (s->integration && q[5] == GO2_RPC_SET) {
            ++test_sets[s->leg];
            if (s->active)
                ++test_foc[s->leg];
        }
        if (q[5] == GO2_RPC_GET) {
            ++s->gets;
            if (!test_freeze.load()) {
                ++s->sample;
                s->sampled_at = go2::SteadyClock::now();
                if (s->integration && s->leg == 0 && test_motion.load()) {
                    if (s->motion_started == go2::TimePoint{})
                        s->motion_started = s->sampled_at;
                    s->position = go2::seconds(s->sampled_at - s->motion_started) * 72.78F;
                }
            }
            if (s->integration) {
                ++test_gets[s->leg];
                go2_put32(r + 20, s->active ? 0 : GO2_RPC_STOPPED);
            }
            for (unsigned id = 0; id < 3; ++id) {
                uint8_t *f = r + 32 + id * 40;
                f[0] = 1;
                f[2] = id;
                const bool moving = s->integration && s->leg == 0 && id == 2 && test_motion.load();
                go2_put_float(f + 8, moving ? 72.78F : 0.F);
                go2_put_float(f + 12, s->integration ? (moving ? s->position : 0.F) : (float)id + 10);
                const auto age =
                    std::chrono::duration_cast<std::chrono::milliseconds>(go2::SteadyClock::now() - s->sampled_at)
                        .count();
                go2_put32(f + 24, s->integration ? (test_stale.load() ? 400 :
                    std::max(static_cast<unsigned>(age), test_feedback_age.load())) : 20);
                go2_put32(f + 28, s->integration ? s->sample : 7);
            }
            if (s->malformed) {
                if (s->malformed == 1)
                    r[4] = 1;
                else if (s->malformed == 2)
                    go2_put_float(r + 44, NAN);
                else
                    r[34] = 9;
            }
        }
        if (q[5] == GO2_RPC_STOP) {
            ++s->stops;
            s->active = false;
        }
        // STOP acceptance alone is not completion: require another snapshot.
        if (s->stops && !s->active && q[5] == GO2_RPC_GET)
            go2_put32(r + 20, GO2_RPC_STOPPED);
        if (s->integration && s->leg == 0 && q[5] == GO2_RPC_GET && test_drop_get_reply.exchange(false))
            continue;
        uint8_t stale[GO2_RPC_SIZE];
        memcpy(stale, r, sizeof(r));
        go2_put64(stale + 12, 99);
        assert(write(s->fd, stale, sizeof(stale)) == sizeof(stale));
        assert(write(s->fd, r, sizeof(r)) == sizeof(r));
        if (!s->integration && s->stops && q[5] == GO2_RPC_GET)
            break;
    }
    return NULL;
}

static void test_workers() {
    go2::HardwareConfig new_config;
    new_config.backend = go2::Backend::kSerial;
    new_config.board_profile = go2::BoardProfile::kNewSmallcore;
    new_config.motor_io_authorized = new_config.actuation_enabled = true;
    new_config.hardware_configuration_verified = new_config.motor_ids_verified = true;
    new_config.joint_calibration_verified = new_config.suspended_test_verified = true;
    new_config.imu_driver = "drv_spi_bmi270";
    new_config.imu_device = "/dev/go2/imu";
    const char *names[] = {"go2-motor-rl", "go2-motor-rr", "go2-motor-fl", "go2-motor-fr"};
    struct server servers[4]{};
    pthread_t threads[4];
    endpoint_created = 1;
    for (unsigned leg = 0; leg < 4; ++leg) {
        int fd[2];
        assert(!socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0, fd));
        assert(!fcntl(fd[1], F_SETFL, 0));
        leg_fds[leg] = fd[0];
        servers[leg].fd = fd[1];
        servers[leg].leg = leg;
        servers[leg].integration = true;
        new_config.legs[leg].driver = "go2_smallcore";
        new_config.legs[leg].device = names[leg];
        for (unsigned id = 0; id < 3; ++id)
            new_config.legs[leg].calibration[id].motor_id = id;
        assert(!pthread_create(&threads[leg], NULL, serve, &servers[leg]));
    }
    {
        go2::Hardware rpmsg(new_config);
        rpmsg.start();
        wait_for([&] {
            for (const auto &joint : rpmsg.read().motors.joints)
                if (!joint.valid || !joint.feedback.position_anchored)
                    return false;
            return all_at_least(test_sets, 1);
        });
        go2::Commands batch{};
        for (auto &command : batch) {
            command.mode = go2::MotorMode::kFoc;
            command.torque_nm = 6.33F;
            command.position_rad = 0.1F;
            command.kp = 6.33F * 6.33F;
        }
        rpmsg.write(batch);
        wait_for([&] { return all_at_least(test_foc, 1); });
        {
            std::lock_guard<std::mutex> lock(target_mutex);
            for (const auto &leg : test_targets)
                for (const auto &target : leg) {
                    assert(std::fabs(target.torque_nm - 1) < 1e-5);
                    assert(std::fabs(target.position_rad - .633F) < 1e-5);
                    assert(std::fabs(target.kp - 1) < 1e-5);
                }
        }
        test_freeze = true;
        const auto initial_gets = test_gets[0].load();
        wait_for([&] { return test_gets[0].load() >= initial_gets + 2; });
        const auto first = rpmsg.read().motors.joints[0].received_at;
        wait_for([&] { return test_gets[0].load() >= initial_gets + 5; });
        assert(rpmsg.read().motors.joints[0].received_at == first);
        for (const auto &count : test_foc)
            assert(count.load() == 1); // GET must not resend FOC.
        test_freeze = false;
        wait_for([&] {
            std::lock_guard<std::mutex> lock(target_mutex);
            for (const auto &leg : test_targets)
                for (const auto &target : leg)
                    if (target.mode != MotorMode::kBrake)
                        return false;
            return true;
        }); // Host watchdog brakes after controller stops publishing.
        rpmsg.write(batch);
        wait_for([&] { return all_at_least(test_foc, 2); });
        test_feedback_age = 150;
        const auto before_loss = test_gets[0].load();
        wait_for([&] { return test_gets[0].load() >= before_loss + 3; });
        assert(!rpmsg.read().motors.session_anchor_faulted);
        for (const auto &joint : rpmsg.read().motors.joints)
            assert(joint.valid && joint.feedback.position_anchored);
        test_feedback_age = 0;
        test_drop_get_reply = true;
        const auto before_rpc_loss = test_gets[0].load();
        wait_for([&] { return test_gets[0].load() >= before_rpc_loss + 3; });
        assert(!rpmsg.read().motors.session_anchor_faulted);
        test_motion = true;
        const auto before_motion = test_gets[0].load();
        wait_for([&] { return test_gets[0].load() >= before_motion + 3; });
        test_freeze = true;
        wait_for([&] { return test_gets[0].load() >= before_motion + 5; });
        const auto before_gap = rpmsg.read().motors.joints[2].feedback.rotor_position_rad;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        test_freeze = false;
        wait_for([&] {
            return rpmsg.read().motors.joints[2].feedback.rotor_position_rad - before_gap > go2::kTwoPi / 2;
        });
        assert(!rpmsg.read().motors.session_anchor_faulted);
        assert(rpmsg.read().motors.joints[2].feedback.position_anchored);
        test_drop_get_reply = true;
        const auto before_moving_rpc_loss = test_gets[0].load();
        wait_for([&] { return test_gets[0].load() >= before_moving_rpc_loss + 3; });
        assert(!rpmsg.read().motors.session_anchor_faulted);
        test_stale = true;
        wait_for([&] { return rpmsg.read().motors.session_anchor_faulted; });
        test_stale = false;
        rpmsg.write(batch);
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        for (const auto &count : test_foc)
            assert(count.load() == 2); // Reconnect cannot re-arm anchors.
        for (auto& command : batch)
            command.protective_crouch = true;
        wait_for([&] {
            rpmsg.write(batch); // Match the running controller through reconnect.
            return all_at_least(test_foc, 3);
        });
        assert(rpmsg.read().motors.session_anchor_faulted);
        wait_for([&] {
            std::lock_guard<std::mutex> lock(target_mutex);
            for (const auto& leg : test_targets)
                for (const auto& target : leg)
                    if (target.mode != MotorMode::kBrake)
                        return false;
            return true;
        }); // Even a protective command expires when publishing stops.
        rpmsg.stop();
    }

    for (unsigned leg = 0; leg < 4; ++leg) {
        shutdown(leg_fds[leg], SHUT_RDWR);
        assert(!pthread_join(threads[leg], NULL));
        close(leg_fds[leg]);
        close(servers[leg].fd);
        leg_fds[leg] = -1;
    }
    test_motion = false;
}

static void test_protective_transmission_clock() {
    go2::JointTransmission sent;
    go2::JointCommand command;
    command.mode = MotorMode::kFoc;
    command.protective_crouch = true;
    command.protective_crouch_id = 1;
    const auto start = go2::TimePoint{} + std::chrono::seconds(10);
    go2::record_transmission(sent, command, start);
    assert(sent.protective_elapsed_sec == 0);
    go2::record_transmission(sent, command, start + std::chrono::milliseconds(50));
    assert(std::fabs(sent.protective_elapsed_sec - .05) < 1e-9);
    // 1.44s without sends cannot consume the remaining crouch time.
    go2::record_transmission(sent, command, start + std::chrono::milliseconds(1490));
    assert(std::fabs(sent.protective_elapsed_sec - .05) < 1e-9);
    go2::record_transmission(sent, command, start + std::chrono::milliseconds(1510));
    assert(std::fabs(sent.protective_elapsed_sec - .07) < 1e-9);
    command.protective_crouch_final = true;
    go2::record_transmission(sent, command, start + std::chrono::milliseconds(1530));
    assert(sent.protective_final);
    // Re-reading the previous transaction cannot credit a different target.
    command = {};
    go2::record_transmission(sent, command, sent.sent_at);
    assert(sent.protective_final && sent.protective_crouch_id == 1);
    go2::record_transmission(sent, command, start + std::chrono::milliseconds(1550));
    assert(!sent.protective_final && !sent.protective_active && sent.protective_crouch_id == 1);
    assert(std::fabs(sent.protective_elapsed_sec - .09) < 1e-9);
    command.mode = MotorMode::kFoc;
    command.protective_crouch = true;
    command.protective_crouch_id = 1;
    go2::record_transmission(sent, command, start + std::chrono::milliseconds(1560));
    assert(sent.protective_active && std::fabs(sent.protective_elapsed_sec - .09) < 1e-9);
    command.protective_crouch_id = 2;
    go2::record_transmission(sent, command, start + std::chrono::milliseconds(1570));
    assert(sent.protective_crouch_id == 2 && sent.protective_elapsed_sec == 0);
}

// A stalled leg remains observable while the other legs transact independently.
static void test_native_uart_diagnostics() {
    struct State {
        unsigned leg;
        std::atomic<bool> *release;
    };
    std::atomic<bool> release{false};
    std::array<go2::NativeUartProgress, 4> progress;
    std::array<std::thread, 4> workers;
    std::array<std::atomic<bool>, 4> completed{};
    std::array<State, 4> states{};
    std::array<struct motor_dev, 4> devices{};
    struct motor_ops ops{};
    ops.set_cmd = [](struct motor_dev *dev, const struct motor_cmd *) {
        auto &state = *static_cast<State *>(dev->priv_data);
        if (state.leg == 2) {
            while (!state.release->load())
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return -ETIMEDOUT;
    };
    auto start = [&](unsigned leg) {
        states[leg] = {leg, &release};
        devices[leg].ops = &ops;
        devices[leg].priv_data = &states[leg];
        workers[leg] = std::thread([&, leg] {
            try {
                go2::transact(&devices[leg], go2::brake_command(), {}, nullptr, true, &progress[leg]);
            } catch (const go2::MotorTransportError &) {
                completed[leg] = true;
            }
        });
    };
    start(2);
    wait_for([&] { return !strcmp(progress[2].snapshot().stage, "set_cmd_send_recv"); });
    for (unsigned leg : {0U, 1U, 3U})
        start(leg);
    const auto deadline = go2::SteadyClock::now() + std::chrono::seconds(2);
    while (!(completed[0] && completed[1] && completed[3]) && go2::SteadyClock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const bool independent = completed[0] && completed[1] && completed[3] && !completed[2];
    const auto stalled = progress[2].snapshot();
    release = true;
    for (auto &worker : workers)
        worker.join();
    assert(independent);
    assert(!strcmp(stalled.stage, "set_cmd_send_recv") && stalled.command_mode == 0);
    for (const auto &trace : progress) {
        assert(trace.snapshot().transactions == 1 && trace.snapshot().failures == 1);
        assert(trace.snapshot().last_result == -ETIMEDOUT);
    }
}

static void test_failed_write_timestamp() {
    struct State {
        uint64_t sent_us{123};
        bool writes{false};
    } state;
    struct motor_ops ops{};
    ops.set_cmd = [](struct motor_dev *dev, const struct motor_cmd *) {
        auto &state = *static_cast<State *>(dev->priv_data);
        if (state.writes)
            ++state.sent_us;
        return -ETIMEDOUT;
    };
    ops.get_paras = [](struct motor_dev *dev, const void *name, void *out, uint32_t size) {
        assert(!strcmp(static_cast<const char *>(name), "command_sent_us") && size == sizeof(uint64_t));
        *static_cast<uint64_t *>(out) = static_cast<State *>(dev->priv_data)->sent_us;
        return 0;
    };
    struct motor_dev device{};
    device.ops = &ops;
    device.priv_data = &state;
    for (bool writes : {false, true}) {
        state.writes = writes;
        uint64_t sent_us = 999;
        bool failed = false;
        try {
            go2::transact(&device, go2::brake_command(), {}, nullptr, false, nullptr, &sent_us);
        } catch (const go2::MotorTransportError &) {
            failed = true;
        }
        assert(failed);
        // A failed write cannot relabel an earlier send. A new full write
        // counts even when its receive phase times out.
        assert(sent_us == (writes ? 124U : 0U));
    }
}

int main(void) {
    test_iwt603_quaternion();
    test_failed_write_timestamp();
    test_protective_transmission_clock();
    test_native_uart_diagnostics();
    assert(!rpmsg_open(NULL) && errno == EINVAL);
    assert(!rpmsg_open("unknown") && errno == EINVAL);
    for (int bad = 0; bad < 4; ++bad) {
        int sockets[2];
        assert(!socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0, sockets));
        assert(fcntl(sockets[1], F_SETFL, 0) == 0);
        transport_fd = sockets[0];
        endpoint_created = bad != 0;
        created_count = 0;
        wrong_address = 1;
        assert(rpmsg_endpoint("go2-motor-fr", 0) < 0);
        wrong_address = 0;
        struct server server{};
        server.fd = sockets[1];
        server.malformed = bad;
        pthread_t thread;
        assert(!pthread_create(&thread, NULL, serve, &server));
        RpmsgSession *p = rpmsg_open("go2-motor-fr");
        assert(p);
        assert(created_count == (bad == 0));
        RotorCommand commands[3]{};
        for (unsigned id = 0; id < 3; ++id) {
            commands[id].mode = MotorMode::kFoc;
            commands[id].torque_nm = (float)id + 1;
            commands[id].position_rad = (float)id + 10;
        }
        commands[0].kp = NAN;
        assert(rpmsg_set(p, commands) == -EINVAL);
        commands[0].kp = 0;
        assert(!rpmsg_set(p, commands));
        RpmsgSnapshot state;
        int result = rpmsg_get(p, &state);
        if (bad) {
            assert(result < 0 && p->failed && !state.valid[0]);
            assert(rpmsg_get(p, &state) < 0);
            assert(rpmsg_set(p, commands) < 0);
        } else {
            assert(result == 0);
            for (unsigned id = 0; id < 3; ++id) {
                assert(state.valid[id] && state.motors[id].position_rad == (float)id + 10);
                assert(state.age_ms[id] >= 21 && state.sample[id] == 7);
            }
            assert(!rpmsg_get(p, &state)); // GET is not SET.
        }
        rpmsg_close(p);
        assert(!pthread_join(thread, NULL));
        assert(server.sets == 1 && server.stops == 1);
        close(sockets[0]);
        close(sockets[1]);
    }
    int fd[2];
    assert(!socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0, fd));
    RpmsgSession p{};
    p.fd = fd[0];
    p.session = 43;
    RpmsgSnapshot state;
    int64_t start = rpmsg_milliseconds();
    assert(rpmsg_get(&p, &state) == -ETIMEDOUT);
    assert(rpmsg_milliseconds() - start < 500 && !p.failed && !state.valid[0]);
    p.seq = UINT32_MAX;
    assert(rpmsg_exchange(&p, GO2_RPC_GET, NULL, NULL) == -EOVERFLOW);
    close(fd[0]);
    close(fd[1]);
    test_workers();
    puts("PASS RPMsg v2: batch targets, snapshots/age, endpoint creation, stale session, invalid version/data, "
        "timeout, STOP");
}
