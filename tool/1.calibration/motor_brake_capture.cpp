#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <thread>

#include "motor.h"

namespace {
bool parse_number(const char* text, int& value) {
    const auto end = text + std::strlen(text);
    const auto result = std::from_chars(text, end, value);
    return result.ec == std::errc{} && result.ptr == end;
}
}  // namespace

int main(int argc, char** argv) {
    int id = -1, samples = 0;
    if (argc != 4 || !parse_number(argv[2], id) || id < 0 || id > 2 || !parse_number(argv[3], samples) || samples < 1 ||
        samples > 10000) {
        std::cerr << "Usage: motor_brake_capture PORT MOTOR_ID SAMPLES\n"
                        "Sends only BRAKE requests; stop all other motor bus users.\n";
        return 2;
    }
    auto* dev = motor_alloc_uart("drv_485_m8010a", argv[1], 4000000, static_cast<std::uint8_t>(id), nullptr);
    if (!dev) return 1;
    int result = motor_init_one(dev);
    motor_cmd command{};
    command.mode = MOTOR_MODE_IDLE;
    std::cout << std::setprecision(9);
    for (int sample = 0; result == 0 && sample < samples; ++sample) {
        motor_state state{};
        std::uint8_t feedback_id = 255;
        if (motor_set_cmd_one(dev, &command) < 0 || motor_get_state_one(dev, &state) < 0 ||
            motor_get_paras(dev, "feedback_id", &feedback_id, sizeof(feedback_id)) < 0 || feedback_id != id ||
            state.err != 0 || !std::isfinite(state.pos) || !std::isfinite(state.vel) || !std::isfinite(state.trq)) {
            std::cerr << "Invalid/missing BRAKE feedback for motor " << id << '\n';
            result = -1;
            break;
        }
        std::cout << "reply id=" << static_cast<unsigned>(feedback_id) << " temp=" << state.temp
                    << " error=" << state.err << " q=" << state.pos << " dq=" << state.vel << " tau=" << state.trq
                    << '\n';
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    motor_free(&dev, 1);
    if (result != 0) return 1;
    std::cout << "valid replies: " << samples << '/' << samples << '\n';
    return 0;
}
