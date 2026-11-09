#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <SDL.h>

#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) { stop_requested = 1; }

class SdlSession {
public:
    SdlSession() {
        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0) {
            throw std::runtime_error(std::string("Cannot open keyboard control display: ") + SDL_GetError());
        }
    }
    ~SdlSession() { SDL_Quit(); }
};
}  // namespace

class KilinKeyboard : public rclcpp::Node {
public:
    KilinKeyboard() : Node("kilin_keyboard") {
        vmax_ = declare_parameter<double>("vmax", 0.3);
        wmax_ = declare_parameter<double>("wmax", 1.0);
        if (!std::isfinite(vmax_) || vmax_ <= 0.0 ||
            !std::isfinite(wmax_) || wmax_ <= 0.0) {
            throw std::runtime_error("vmax and wmax must be finite and positive.");
        }
        window_.reset(SDL_CreateWindow(
            "Kilin: click here | Arrows: move | Q/E: turn | Space: STOP | Esc: quit",
            SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 760, 260, SDL_WINDOW_SHOWN));
        if (!window_) {
            throw std::runtime_error(std::string("Cannot create keyboard window: ") + SDL_GetError());
        }
        focused_ = (SDL_GetWindowFlags(window_.get()) & SDL_WINDOW_INPUT_FOCUS) != 0;
        publisher_ = create_publisher<geometry_msgs::msg::Twist>("/kilin/cmd_vel", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { tick(); });
        RCLCPP_INFO(get_logger(),
            "Click the keyboard control window. Arrow keys: eight-direction movement; Q/E: left/right rotation.\n"
            "Movement and rotation can be combined. Opposite keys cancel. WASD is disabled.\n"
            "Space/X: stop until all movement keys are released. Esc/window close/Ctrl+C: exit.\n"
            "Release keys or leave window focus to stop. vmax=%.2f m/s, wmax=%.2f rad/s",
            vmax_, wmax_);
    }

    void stop() { publisher_->publish(geometry_msgs::msg::Twist{}); }

private:
    static int key_index(SDL_Scancode key) {
        switch (key) {
            case SDL_SCANCODE_UP: return 0;
            case SDL_SCANCODE_DOWN: return 1;
            case SDL_SCANCODE_LEFT: return 2;
            case SDL_SCANCODE_RIGHT: return 3;
            case SDL_SCANCODE_Q: return 4;
            case SDL_SCANCODE_E: return 5;
            default: return -1;
        }
    }

    bool any_key_down() const {
        for (bool down : pressed_) {
            if (down) return true;
        }
        return false;
    }

    void event(const SDL_Event &input) {
        if (input.type == SDL_QUIT) {
            stop_requested = 1;
            return;
        }
        if (input.type == SDL_WINDOWEVENT) {
            if (input.window.event == SDL_WINDOWEVENT_FOCUS_GAINED) {
                focused_ = true;
                pressed_.fill(false);
                stop_keys_.fill(false);
                stop_latched_ = false;
            } else if (input.window.event == SDL_WINDOWEVENT_FOCUS_LOST ||
                       input.window.event == SDL_WINDOWEVENT_MINIMIZED ||
                       input.window.event == SDL_WINDOWEVENT_HIDDEN) {
                focused_ = false;
                pressed_.fill(false);
                stop_keys_.fill(false);
                stop_latched_ = false;
            }
            return;
        }
        if (input.type != SDL_KEYDOWN && input.type != SDL_KEYUP) return;
        const auto key = input.key.keysym.scancode;
        const bool down = input.type == SDL_KEYDOWN;
        if (down && (key == SDL_SCANCODE_ESCAPE ||
                     (key == SDL_SCANCODE_C && (input.key.keysym.mod & KMOD_CTRL)))) {
            stop_requested = 1;
            return;
        }
        if (!focused_) return;
        if (key == SDL_SCANCODE_SPACE || key == SDL_SCANCODE_X) {
            stop_keys_[key == SDL_SCANCODE_SPACE ? 0 : 1] = down;
            if (down) stop_latched_ = true;
        }
        const int index = key_index(key);
        if (index >= 0) {
            // After focus returns, ignore auto-repeat from keys held before focus
            // was lost. A fresh key press is required to resume motion.
            if (down && input.key.repeat && !pressed_[index]) return;
            pressed_[index] = down;
        }
        if (!any_key_down() && !stop_keys_[0] && !stop_keys_[1]) stop_latched_ = false;
    }

    void draw(const geometry_msgs::msg::Twist &command) {
        auto *surface = SDL_GetWindowSurface(window_.get());
        if (!surface) return;
        const auto dark = SDL_MapRGB(surface->format, 30, 35, 44);
        const auto idle = SDL_MapRGB(surface->format, 76, 85, 99);
        const auto active = SDL_MapRGB(surface->format, 40, 190, 120);
        SDL_FillRect(surface, nullptr, dark);
        // A 3x3 direction pad: screen up is robot forward, screen left is +y.
        for (int row = -1; row <= 1; ++row) {
            for (int col = -1; col <= 1; ++col) {
                const bool selected =
                    (row == (command.linear.x > 0 ? -1 : command.linear.x < 0 ? 1 : 0)) &&
                    (col == (command.linear.y > 0 ? -1 : command.linear.y < 0 ? 1 : 0));
                SDL_Rect rect{305 + col * 60, 100 + row * 60, 50, 50};
                SDL_FillRect(surface, &rect, selected ? active : idle);
            }
        }
        SDL_Rect left_turn{100, 100, 90, 50};
        SDL_Rect right_turn{490, 100, 90, 50};
        SDL_FillRect(surface, &left_turn, command.angular.z > 0 ? active : idle);
        SDL_FillRect(surface, &right_turn, command.angular.z < 0 ? active : idle);
        SDL_UpdateWindowSurface(window_.get());
        const std::string title = std::string("Kilin | ") +
            (!focused_ ? "UNFOCUSED - STOPPED" : stop_latched_ ? "STOP - release keys" :
             "Arrows: move | Q/E: turn | Space: STOP | Esc: quit") +
            " | vx=" + std::to_string(command.linear.x).substr(0, 5) +
            " vy=" + std::to_string(command.linear.y).substr(0, 5) +
            " wz=" + std::to_string(command.angular.z).substr(0, 5);
        SDL_SetWindowTitle(window_.get(), title.c_str());
    }

    void tick() {
        SDL_Event input;
        while (SDL_PollEvent(&input)) event(input);
        geometry_msgs::msg::Twist command;
        if (focused_ && !stop_latched_ && !stop_requested) {
            const double x = static_cast<int>(pressed_[0]) - static_cast<int>(pressed_[1]);
            const double y = static_cast<int>(pressed_[2]) - static_cast<int>(pressed_[3]);
            const double norm = std::hypot(x, y);
            if (norm > 0.0) {
                command.linear.x = vmax_ * x / norm;
                command.linear.y = vmax_ * y / norm;
            }
            command.angular.z = wmax_ *
                (static_cast<int>(pressed_[4]) - static_cast<int>(pressed_[5]));
        }
        publisher_->publish(command);
        // Rendering at 20 Hz keeps the 100 Hz command loop lightweight.
        if (++draw_ticks_ >= 5) {
            draw_ticks_ = 0;
            draw(command);
        }
    }

    double vmax_, wmax_;
    bool focused_ = false;
    bool stop_latched_ = false;
    std::array<bool, 2> stop_keys_{};
    int draw_ticks_ = 0;
    std::array<bool, 6> pressed_{};
    std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)> window_{nullptr, SDL_DestroyWindow};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);
    int result = 0;
    try {
        SdlSession sdl;
        auto node = std::make_shared<KilinKeyboard>();
        while (rclcpp::ok() && !stop_requested) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        for (int i = 0; i < 3 && rclcpp::ok(); ++i) {
            node->stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } catch (const std::exception &error) {
        RCLCPP_ERROR(rclcpp::get_logger("kilin_keyboard"), "%s", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
