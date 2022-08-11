#pragma once
#include <husky_panda_control/core/config.h>
#include <husky_panda_control/core/rollout.h>
#include <husky_panda_control/core/typedefs.h>

#include <GLFW/glfw3.h>

namespace husky_panda_tools {

// utility structure for realtime plot
struct data_point_t {
  data_point_t() = default;
  data_point_t(float v1, float v2) : x(v1), y(v2){};
  float x;
  float y;
};

struct scrolling_buffer_t {
  int max_size;
  int offset;
  int first_index;
  int last_index;
  std::vector<data_point_t> data;
  scrolling_buffer_t(int _max_size = 2000) {
    max_size = _max_size;
    offset = 0;
    first_index = 0;
    last_index = -1;
    data.reserve(max_size);
  }
  void add_point(float x, float y) {
    if (data.size() < max_size) {
      data.emplace_back(x, y);
      last_index++;
    } else {
      data[offset] = data_point_t(x, y);
      last_index = offset;
      offset = (offset + 1) % max_size;
      first_index = offset;
    }
  }

  data_point_t operator[](int i) {
    int idx = (first_index + i) % max_size;
    return data_point_t(data[idx].x, data[idx].y);
  }

  data_point_t back() { return data[last_index]; }

  void Erase() {
    if (data.size() > 0) {
      data.resize(0);
      offset = 0;
    }
  }
};

class ControlGui {
 public:
  ControlGui() = default;
  ~ControlGui();

  bool init();
  bool render();
  void update();
  void draw();
  bool close();

  void window_resize(int width, int height);

  void reset_config(const husky_panda_control::config_t& config);
  void reset_input(const husky_panda_control::input_t& u, const double t);
  void reset_averaged_policy(const husky_panda_control::input_array_t& u);
  void reset_policy(const husky_panda_control::input_array_t& u);
  void reset_rollouts(const std::vector<husky_panda_control::Rollout>& rollouts);
  void reset_weights(const std::vector<double>& weights);

  bool should_pause() const { return pause; }
  bool& step_simulation() { return step_simulation_; };
  bool& step_controller() { return step_controller_; };
  bool& step_all() { return step_all_; };

 private:
  bool setup_glfw();

 private:
  GLFWwindow* window_ptr_;

  // control
  bool pause = false;
  bool step_simulation_ = false;
  bool step_controller_ = false;
  bool step_all_ = false;

  // husky_panda_control data
  husky_panda_control::Config config_;
  std::vector<husky_panda_control::Rollout> rollouts_;
  std::vector<scrolling_buffer_t> u_;
  husky_panda_control::input_array_t uu_;
  husky_panda_control::input_array_t uu_avg_;
  std::vector<double> weights_;
};
}  // namespace husky_panda_tools
