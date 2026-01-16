#include <ros_tools/profiling.h>
#include <ros_tools/logging.h>

#include <mpc_planner/planner.h>
#include <mpc_planner/data_preparation.h>
#include <mpc_planner_solver/state.h>
#include <mpc_planner_solver/model_detector.h>
#include <mpc_planner_types/realtime_data.h>
#include <mpc_planner_util/parameters.h>
#include <guidance_planner/global_guidance.h>

#include <mpc_planner_types/data_types.h>

#define WITHOUT_NUMPY
#include "matplotlibcpp.h"
#include "third_party/simple_json.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <ros_tools/spline.h>

namespace plt = matplotlibcpp;
namespace fs = std::filesystem;
using namespace MPCPlanner;

namespace
{

volatile std::sig_atomic_t g_running = 1;
constexpr double kPi = 3.14159265358979323846;

void handleSignal(int signal)
{
    (void)signal;
    g_running = 0;
}

std::pair<std::vector<double>, std::vector<double>> makeCircle(double cx, double cy, double radius, int samples = 48)
{
    std::vector<double> xs;
    std::vector<double> ys;
    xs.reserve(samples + 1);
    ys.reserve(samples + 1);

    for (int i = 0; i <= samples; ++i)
    {
        double theta = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(samples);
        xs.push_back(cx + radius * std::cos(theta));
        ys.push_back(cy + radius * std::sin(theta));
    }

    return {xs, ys};
}

struct SimObstacle
{
    int id;
    Eigen::Vector2d position;
    Eigen::Vector2d velocity;
    double radius;
    bool bounce_x{false};
    bool bounce_y{true};
    double min_x{0.0};
    double max_x{0.0};
    double min_y{-1.5};
    double max_y{1.5};
    bool is_static{false};

    // 矩形障碍物信息（用于可视化）
    bool is_rectangle{false};
    double rect_width{0.0};
    double rect_height{0.0};
    double rect_yaw{0.0};

    void update(double dt)
    {
        if (is_static)
            return;

        position += velocity * dt;

        if (bounce_x)
        {
            if (position.x() < min_x)
            {
                position.x() = min_x;
                velocity.x() = std::abs(velocity.x());
            }
            else if (position.x() > max_x)
            {
                position.x() = max_x;
                velocity.x() = -std::abs(velocity.x());
            }
        }

        if (bounce_y)
        {
            if (position.y() < min_y)
            {
                position.y() = min_y;
                velocity.y() = std::abs(velocity.y());
            }
            else if (position.y() > max_y)
            {
                position.y() = max_y;
                velocity.y() = -std::abs(velocity.y());
            }
        }
    }
};

struct GuidancePath
{
    std::vector<Eigen::Vector2d> points;
    bool selected{false};
    int color_index{-1};
};

std::string colorFromIndex(int idx)
{
    static const std::vector<std::string> palette = {
        "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
        "#9467bd", "#8c564b", "#e377c2", "#7f7f7f",
        "#bcbd22", "#17becf", "#e6550d", "#31a354"
    };

    if (idx < 0)
        return "#4c566a";
    return palette[static_cast<size_t>(idx) % palette.size()];
}

class MatplotlibVisualizer
{
public:
    MatplotlibVisualizer() = default;

    void initialize(const std::map<int, std::string>& button_labels = {})
    {
        if (_initialized)
            return;

        matplotlibcpp::detail::_interpreter::get();

        // 设置场景按钮标签到 Python
        _scenario_button_labels = button_labels;
        if (!button_labels.empty())
        {
            PyRun_SimpleString("import matplotlib.pyplot as plt\n");
            PyRun_SimpleString("plt._mpc_scenario_labels = {}\n");
            for (const auto& [id, label] : button_labels)
            {
                std::string cmd = "plt._mpc_scenario_labels[" + std::to_string(id) + "] = '" + label + "'\n";
                PyRun_SimpleString(cmd.c_str());
            }
        }

        const char *script = R"PYTHON(
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, CheckButtons

plt.ion()
plt.close('all')
fig = plt.figure(figsize=(12.8, 6.4))
main_ax = fig.add_subplot(2, 1, 1)
speed_ax = fig.add_subplot(2, 1, 2)
fig.subplots_adjust(top=0.85, right=0.75)

if not hasattr(plt, '_mpc_pause_state'):
    plt._mpc_pause_state = {'paused': False}
else:
    plt._mpc_pause_state['paused'] = bool(plt._mpc_pause_state.get('paused', False))

# 场景切换状态
if not hasattr(plt, '_mpc_scenario_switch'):
    plt._mpc_scenario_switch = {'requested': False, 'scenario_id': -1}

# Initialize visibility state for legend items
if not hasattr(plt, '_mpc_visibility'):
    plt._mpc_visibility = {}

def _mpc_pause(event):
    plt._mpc_pause_state['paused'] = True

def _mpc_start(event):
    plt._mpc_pause_state['paused'] = False

def _mpc_on_legend_pick(event):
    legline = event.artist
    label = legline.get_label()
    if label not in plt._mpc_visibility:
        plt._mpc_visibility[label] = True
    plt._mpc_visibility[label] = not plt._mpc_visibility[label]

    # Update visibility of all lines with this label
    ax = event.inaxes
    if ax is None:
        return
    for line in ax.get_lines():
        if line.get_label() == label:
            line.set_visible(plt._mpc_visibility[label])
    for patch in ax.patches:
        if patch.get_label() == label:
            patch.set_visible(plt._mpc_visibility[label])
    fig.canvas.draw_idle()

pause_ax = fig.add_axes([0.58, 0.9, 0.08, 0.05])
start_ax = fig.add_axes([0.67, 0.9, 0.08, 0.05])

pause_btn = Button(pause_ax, 'Pause')
start_btn = Button(start_ax, 'Start')
pause_btn.on_clicked(_mpc_pause)
start_btn.on_clicked(_mpc_start)

# 场景切换按钮 (3x3 网格)
def _mpc_switch_scenario(scenario_id):
    def callback(event):
        plt._mpc_scenario_switch['requested'] = True
        plt._mpc_scenario_switch['scenario_id'] = scenario_id
    return callback

# 3x3 场景按钮网格
# 按钮标签将从 C++ 传入
if not hasattr(plt, '_mpc_scenario_labels'):
    plt._mpc_scenario_labels = {}

scenario_buttons = []
button_width = 0.07  # 按钮宽度（适中大小）
button_height = 0.045  # 按钮高度（适中大小）
start_x = 0.75  # 调整起始位置
start_y = 0.80  # 往下移动（原来 0.88）
gap_x = 0.01  # 按钮间距
gap_y = 0.01  # 按钮间距

for row in range(3):
    for col in range(3):
        scenario_id = row * 3 + col + 1
        x = start_x + col * (button_width + gap_x)
        y = start_y - row * (button_height + gap_y)
        btn_ax = fig.add_axes([x, y, button_width, button_height])
        # 使用中文标签（如果有），否则使用默认标签
        label = plt._mpc_scenario_labels.get(scenario_id, f'S{scenario_id}')
        btn = Button(btn_ax, label, color='lightblue', hovercolor='skyblue')
        # 设置支持中文的字体
        try:
            from matplotlib import font_manager
            # 尝试使用常见的中文字体
            chinese_fonts = ['WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'Noto Sans CJK SC',
                           'Noto Sans CJK TC', 'SimHei', 'Microsoft YaHei', 'DejaVu Sans']
            font_found = False
            for font_name in chinese_fonts:
                try:
                    btn.label.set_fontproperties(font_manager.FontProperties(family=font_name, size=11))
                    font_found = True
                    break
                except:
                    continue
            if not font_found:
                btn.label.set_fontsize(11)
        except:
            btn.label.set_fontsize(11)
        btn.on_clicked(_mpc_switch_scenario(scenario_id))
        scenario_buttons.append(btn)

plt._mpc_scenario_buttons = scenario_buttons

fig.canvas.mpl_connect('pick_event', _mpc_on_legend_pick)

plt._mpc_axes = {'main': main_ax, 'speed': speed_ax}

# Visibility toggles via CheckButtons (checkboxes)
try:
    labels = ['Reference Path','Ego History','Candidate Guidance Path','Selected Guidance Path',
              'MPC Candidates','MPC Best Trajectory','MPC Selected','Static Obstacle',
              'Dynamic Obstacle','Obstacle Prediction','Robot Footprint','Goal',
              'Planned Speed','History Speed']
    if not hasattr(plt, '_mpc_visibility'):
        plt._mpc_visibility = {}
    for key in labels:
        if key not in plt._mpc_visibility:
            plt._mpc_visibility[key] = True

    cb_ax = fig.add_axes([0.76, 0.05, 0.23, 0.55])
    # 设置标题使用中文字体
    from matplotlib import font_manager
    cb_ax.set_title('显示项', fontsize=10, fontweight='bold',
                    fontproperties=font_manager.FontProperties(family='WenQuanYi Micro Hei'))
    # 隐藏复选框区域的边框
    cb_ax.spines['top'].set_visible(False)
    cb_ax.spines['right'].set_visible(False)
    cb_ax.spines['bottom'].set_visible(False)
    cb_ax.spines['left'].set_visible(False)
    cb_ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
    plt._mpc_checks = CheckButtons(cb_ax, labels, [plt._mpc_visibility[k] for k in labels])

    # 修改复选框样式：使用实心方块而不是 x
    for rect in plt._mpc_checks.rectangles:
        rect.set_facecolor('white')
        rect.set_edgecolor('gray')
        rect.set_linewidth(1.5)

    # 保存应用可见性的函数引用
    def _mpc_apply_visibility_main():
        ax = plt._mpc_axes['main']
        for line in ax.get_lines():
            lab = line.get_label()
            if lab in plt._mpc_visibility:
                line.set_visible(plt._mpc_visibility[lab])
        for patch in ax.patches:
            lab = patch.get_label()
            if lab in plt._mpc_visibility:
                patch.set_visible(plt._mpc_visibility[lab])

    def _mpc_apply_visibility_speed():
        ax = plt._mpc_axes['speed']
        for line in ax.get_lines():
            lab = line.get_label()
            if lab in plt._mpc_visibility:
                line.set_visible(plt._mpc_visibility[lab])

    def _mpc_on_check(label):
        plt._mpc_visibility[label] = not plt._mpc_visibility.get(label, True)
        # 更新复选框显示：使用实心方块
        idx = labels.index(label)
        if plt._mpc_visibility[label]:
            plt._mpc_checks.rectangles[idx].set_facecolor('dodgerblue')
        else:
            plt._mpc_checks.rectangles[idx].set_facecolor('white')
        # 应用可见性
        _mpc_apply_visibility_main()
        _mpc_apply_visibility_speed()
        fig.canvas.draw_idle()

    plt._mpc_checks.on_clicked(_mpc_on_check)

    # 初始化复选框颜色
    for i, label in enumerate(labels):
        if plt._mpc_visibility.get(label, True):
            plt._mpc_checks.rectangles[i].set_facecolor('dodgerblue')
        else:
            plt._mpc_checks.rectangles[i].set_facecolor('white')

    # 保存函数引用以便在 update 中调用
    plt._mpc_apply_visibility_main = _mpc_apply_visibility_main
    plt._mpc_apply_visibility_speed = _mpc_apply_visibility_speed
except Exception as _e:
    pass
)PYTHON";
        PyRun_SimpleString(script);
        _initialized = true;
    }

    void resetLayout()
    {
        if (!_initialized)
            return;
        // 只清除图形内容，不关闭窗口
        const char* script = R"PYTHON(
import matplotlib.pyplot as plt
# 清除主轴和速度轴的内容
if hasattr(plt, '_mpc_axes'):
    plt._mpc_axes['main'].clear()
    plt._mpc_axes['speed'].clear()
# 重置参考路径缓存标志
)PYTHON";
        PyRun_SimpleString(script);
        _reference_cache_ready = false;
    }

    void update(const State &state,
                const RealTimeData &data,
                const PlannerOutput &output,
                const std::vector<CandidateTrajectory> &candidates,
                const std::vector<double> &history_x,
                const std::vector<double> &history_y,
                const std::vector<double> &history_speed,
                const std::vector<double> &history_time,
                const std::vector<GuidancePath> &guidance_paths,
                double sim_time,
                int iteration)
    {
        if (!_initialized)
            initialize();

        if (!_reference_cache_ready && !data.reference_path.x.empty())
        {
            _ref_x = data.reference_path.x;
            _ref_y = data.reference_path.y;
            _reference_cache_ready = true;
        }

        setActiveAxis("main");
        plt::cla();


        if (!_ref_x.empty())
        {
            std::map<std::string, std::string> ref_opts;
            ref_opts["color"] = "k";
            ref_opts["linestyle"] = "--";
            ref_opts["label"] = "Reference Path";
            plt::plot(_ref_x, _ref_y, ref_opts);
        }

        if (!history_x.empty())
        {
            std::map<std::string, std::string> history_opts;
            history_opts["color"] = "b";
            history_opts["linestyle"] = "-";
            history_opts["label"] = "Ego History";

        // MPC Candidate Trajectories (from T-MPC parallel planners)
        if (!candidates.empty())
        {
            for (const auto &c : candidates)
            {
                if (!c.success || c.traj.positions.empty())
                    continue;

                std::vector<double> cx;
                std::vector<double> cy;
                cx.reserve(c.traj.positions.size());
                cy.reserve(c.traj.positions.size());
                for (const auto &p : c.traj.positions)
                {
                    cx.push_back(p.x());
                    cy.push_back(p.y());
                }

                std::map<std::string, std::string> opts;
                opts["color"] = colorFromIndex(c.color);

                if (c.is_best)
                {
                    // Best trajectory: solid, thick, bright
                    opts["linestyle"] = "-";
                    opts["linewidth"] = "3.5";
                    opts["label"] = "MPC Best Trajectory";
                }
                else
                {
                    // Other candidates: solid, bold
                    opts["linestyle"] = "-";
                    opts["linewidth"] = "2.2";
                    opts["label"] = "MPC Candidates";
                }
                plt::plot(cx, cy, opts);
            }
        }

            plt::plot(history_x, history_y, history_opts);
        }

        if (output.success && !output.trajectory.positions.empty())
        {
            std::vector<double> traj_x;
            std::vector<double> traj_y;
            traj_x.reserve(output.trajectory.positions.size());
            traj_y.reserve(output.trajectory.positions.size());

            for (const auto &p : output.trajectory.positions)
            {
                traj_x.push_back(p.x());
                traj_y.push_back(p.y());
            }

            std::map<std::string, std::string> traj_opts;
            traj_opts["color"] = "lime";
            traj_opts["linestyle"] = "-";
            traj_opts["linewidth"] = "4.0";
            traj_opts["label"] = "MPC Selected";
            plt::plot(traj_x, traj_y, traj_opts);
        }

        // Obstacles and predictions
        // 注意：如果使用固定坐标范围，不需要动态计算边界
        double min_x = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();

        // 只在未设置固定范围时才计算动态边界
        auto update_bounds = [&](double x, double y) {
            if (!_fixed_bounds_set)
            {
                min_x = std::min(min_x, x);
                max_x = std::max(max_x, x);
                min_y = std::min(min_y, y);
                max_y = std::max(max_y, y);
            }
        };

        if (!_ref_x.empty())
        {
            for (size_t i = 0; i < _ref_x.size(); ++i)
                update_bounds(_ref_x[i], _ref_y[i]);
        }

        for (size_t i = 0; i < history_x.size(); ++i)
            update_bounds(history_x[i], history_y[i]);

        for (const auto &obs : data.dynamic_obstacles)
        {
            if (obs.index < 0)
                continue; // Skip dummy obstacles added for padding

            bool is_static_obstacle = (obs.type == ObstacleType::STATIC);
            auto circle = makeCircle(obs.position.x(), obs.position.y(), obs.radius);
            std::map<std::string, std::string> circle_opts;
            circle_opts["color"] = is_static_obstacle ? "#ff8c00" : "#d62728";
            circle_opts["linewidth"] = is_static_obstacle ? "1.8" : "1.4";
            if (is_static_obstacle)
            {
                circle_opts["label"] = "Static Obstacle";
            }
            else
            {
                circle_opts["label"] = "Dynamic Obstacle";
            }
            plt::plot(circle.first, circle.second, circle_opts);

            if (!is_static_obstacle && !obs.prediction.modes.empty() && !obs.prediction.modes.front().empty())
            {
                std::vector<double> pred_x;
                std::vector<double> pred_y;
                pred_x.reserve(obs.prediction.modes.front().size());
                pred_y.reserve(obs.prediction.modes.front().size());

                for (const auto &step : obs.prediction.modes.front())
                {
                    pred_x.push_back(step.position.x());
                    pred_y.push_back(step.position.y());
                }
                std::map<std::string, std::string> pred_opts;
                pred_opts["color"] = "#d62728";
                pred_opts["linestyle"] = ":";
                pred_opts["label"] = "Obstacle Prediction";
                plt::plot(pred_x, pred_y, pred_opts);

                for (size_t i = 0; i < pred_x.size(); ++i)
                    update_bounds(pred_x[i], pred_y[i]);
            }

            update_bounds(obs.position.x(), obs.position.y());
        }

        for (const auto &path : guidance_paths)
        {
            for (const auto &pt : path.points)
                update_bounds(pt.x(), pt.y());
        }

        const double state_x = state.get("x");
        const double state_y = state.get("y");
        const double psi = state.get("psi");
        const double cos_psi = std::cos(psi);
        const double sin_psi = std::sin(psi);

        for (const auto &disc : data.robot_area)
        {
            const double cx = state_x + disc.offset * cos_psi;
            const double cy = state_y + disc.offset * sin_psi;
            auto circle = makeCircle(cx, cy, disc.radius);

            std::map<std::string, std::string> disc_opts;
            disc_opts["color"] = "#4169e1";
            disc_opts["linewidth"] = "2.2";
            disc_opts["linestyle"] = "-";
            disc_opts["label"] = "Robot Footprint";
            plt::plot(circle.first, circle.second, disc_opts);

            for (size_t i = 0; i < circle.first.size(); ++i)
                update_bounds(circle.first[i], circle.second[i]);
        }

        std::vector<double> ego_x = {state_x};
        std::vector<double> ego_y = {state_y};
        std::map<std::string, std::string> ego_opts;
        ego_opts["color"] = "#0c2c84";
        ego_opts["label"] = "Ego Center";
        plt::scatter(ego_x, ego_y, 160.0, ego_opts);
        update_bounds(ego_x.front(), ego_y.front());

        if (data.goal_received)
        {
            std::vector<double> goal_x = {data.goal.x()};
            std::vector<double> goal_y = {data.goal.y()};
            std::map<std::string, std::string> goal_opts;
            goal_opts["color"] = "gold";
            goal_opts["label"] = "Goal";
            plt::scatter(goal_x, goal_y, 220.0, goal_opts);
            update_bounds(goal_x.front(), goal_y.front());
        }

        for (const auto &path : guidance_paths)
        {
            if (path.points.size() < 2)
                continue;

            std::vector<double> gx;
            std::vector<double> gy;
            gx.reserve(path.points.size());
            gy.reserve(path.points.size());

            for (const auto &pt : path.points)
            {
                gx.push_back(pt.x());
                gy.push_back(pt.y());
            }

            std::map<std::string, std::string> opts;
            opts["color"] = colorFromIndex(path.color_index);
            // Guidance paths: always dashed, thinner
            opts["linewidth"] = "1.2";
            opts["linestyle"] = "--";
            if (path.selected)
            {
                opts["label"] = "Selected Guidance Path";
            }
            else
            {
                opts["label"] = "Candidate Guidance Path";
            }

            plt::plot(gx, gy, opts);
        }

        plt::xlabel("X [m]");
        plt::ylabel("Y [m]");

        std::ostringstream title_stream;
        title_stream << "MPC Jackal-like Simulation  |  Iter: " << iteration
                     << "  Time: " << std::fixed << std::setprecision(1) << sim_time << " s";
        plt::title(title_stream.str());

        if (min_x == std::numeric_limits<double>::infinity())
        {
            min_x = -2.0;
            max_x = 2.0;
            min_y = -2.0;
            max_y = 2.0;
        }

        // 使用固定坐标范围（如果已设置）
        if (_fixed_bounds_set)
        {
            // 使用 Python 代码强制固定坐标范围，禁用自动缩放
            const char* fix_bounds_script = R"PYTHON(
import matplotlib.pyplot as plt
ax = plt._mpc_axes['main']
# 禁用自动缩放
ax.set_autoscale_on(False)
# 设置固定坐标范围
ax.set_xlim(%f, %f)
ax.set_ylim(%f, %f)
# 设置等比例，但不调整坐标范围
ax.set_aspect('equal', adjustable='datalim', anchor='C')
)PYTHON";
            char cmd[512];
            snprintf(cmd, sizeof(cmd), fix_bounds_script, _fixed_min_x, _fixed_max_x, _fixed_min_y, _fixed_max_y);
            PyRun_SimpleString(cmd);
        }
        else
        {
            // 回退到动态计算（用于兼容性）
            const double dx = std::max(1e-3, max_x - min_x);
            const double dy = std::max(1e-3, max_y - min_y);

            const double margin_x = std::max(1.0, 0.15 * dx);
            const double margin_y = std::max(1.0, 0.15 * dy);

            plt::xlim(min_x - margin_x, max_x + margin_x);
            plt::ylim(min_y - margin_y, max_y + margin_y);
            plt::axis("equal");
        }
        plt::grid(true);

        // Apply checkbox visibility and create de-duplicated legend on main axis
        PyRun_SimpleString(R"PYTHON(
import matplotlib.pyplot as plt
ax = plt._mpc_axes['main']
if hasattr(plt, '_mpc_apply_visibility_main'):
    plt._mpc_apply_visibility_main()
handles, labels = ax.get_legend_handles_labels()
uniq_h, uniq_l = [], []
for h, l in zip(handles, labels):
    if l and l not in uniq_l:
        uniq_l.append(l); uniq_h.append(h)
ax.legend(uniq_h, uniq_l, loc='upper right')
)PYTHON");

        setActiveAxis("speed");
        plt::cla();

        _planned_speed_time.clear();
        _planned_speed_values.clear();
        if (output.success && output.trajectory.dt > 1e-6 && !output.trajectory.positions.empty())
        {
            const auto &positions = output.trajectory.positions;
            _planned_speed_time.reserve(positions.size());
            _planned_speed_values.reserve(positions.size());

            for (size_t i = 0; i < positions.size(); ++i)
            {
                _planned_speed_time.push_back(sim_time + output.trajectory.dt * static_cast<double>(i));
                if (i == 0)
                {
                    _planned_speed_values.push_back(state.get("v"));
                }
                else
                {
                    const double dx = positions[i].x() - positions[i - 1].x();
                    const double dy = positions[i].y() - positions[i - 1].y();
                    const double speed = std::sqrt(dx * dx + dy * dy) / output.trajectory.dt;
                    _planned_speed_values.push_back(speed);
                }
            }
        }

        if (!_planned_speed_time.empty())
        {
            std::map<std::string, std::string> traj_speed_opts;
            traj_speed_opts["color"] = "#2ca02c";
            traj_speed_opts["label"] = "Planned Speed";
            traj_speed_opts["linewidth"] = "1.8";
            plt::plot(_planned_speed_time, _planned_speed_values, traj_speed_opts);
        }

        if (!history_time.empty() && history_time.size() == history_speed.size())
        {
            std::map<std::string, std::string> hist_speed_opts;
            hist_speed_opts["color"] = "#1f77b4";
            hist_speed_opts["label"] = "History Speed";
            hist_speed_opts["linewidth"] = "1.6";
            plt::plot(history_time, history_speed, hist_speed_opts);
        }

        plt::xlabel("Time [s]");
        plt::ylabel("Speed [m/s]");
        plt::grid(true);

        // Apply checkbox visibility and create de-duplicated legend on speed axis
        PyRun_SimpleString(R"PYTHON(
import matplotlib.pyplot as plt
ax = plt._mpc_axes['speed']
if hasattr(plt, '_mpc_apply_visibility_speed'):
    plt._mpc_apply_visibility_speed()
handles, labels = ax.get_legend_handles_labels()
uniq_h, uniq_l = [], []
for h, l in zip(handles, labels):
    if l and l not in uniq_l:
        uniq_l.append(l); uniq_h.append(h)
ax.legend(uniq_h, uniq_l, loc='upper right')
)PYTHON");
        plt::pause(0.001);
    }

    void waitWhilePaused()
    {
        if (!_initialized)
            return;

        while (isPaused() && g_running)
        {
            plt::pause(0.05);
        }
    }

    bool isPaused() const
    {
        PyObject *plt_mod = PyImport_AddModule("matplotlib.pyplot");
        if (!plt_mod)
            return false;

        PyObject *state = PyObject_GetAttrString(plt_mod, "_mpc_pause_state");
        if (!state || !PyDict_Check(state))
        {
            Py_XDECREF(state);
            return false;
        }

        PyObject *paused_obj = PyDict_GetItemString(state, "paused");
        bool paused = paused_obj && PyObject_IsTrue(paused_obj);
        Py_DECREF(state);
        return paused;
    }

    void setActiveAxis(const std::string &axis_key)
    {
        std::string script =
            "import matplotlib.pyplot as plt\n"
            "plt.sca(plt._mpc_axes['" + axis_key + "'])\n";
        PyRun_SimpleString(script.c_str());
    }

    // 设置固定坐标范围（根据起点和终点）
    void setFixedBounds(double start_x, double start_y, double goal_x, double goal_y)
    {
        // 计算起点和终点的边界
        double min_x = std::min(start_x, goal_x);
        double max_x = std::max(start_x, goal_x);
        double min_y = std::min(start_y, goal_y);
        double max_y = std::max(start_y, goal_y);

        // 计算路径长度和宽度
        double path_length = max_x - min_x;
        double path_width = max_y - min_y;

        // 添加边距（路径长度和宽度的20%，最小5米）
        double margin_x = std::max(5.0, path_length * 0.2);
        double margin_y = std::max(5.0, path_width * 0.2);

        // 如果路径太窄，确保有足够的Y轴范围
        if (path_width < 10.0)
        {
            margin_y = std::max(margin_y, (10.0 - path_width) / 2.0);
        }

        _fixed_min_x = min_x - margin_x;
        _fixed_max_x = max_x + margin_x;
        _fixed_min_y = min_y - margin_y;
        _fixed_max_y = max_y + margin_y;
        _fixed_bounds_set = true;

        LOG_INFO("设置固定坐标范围: X[" << _fixed_min_x << ", " << _fixed_max_x
                 << "], Y[" << _fixed_min_y << ", " << _fixed_max_y << "]");
    }

    // 检查是否有场景切换请求
    bool checkScenarioSwitch(int& scenario_id)
    {
        const char* script = R"PYTHON(
import matplotlib.pyplot as plt
requested = plt._mpc_scenario_switch.get('requested', False)
scenario_id = plt._mpc_scenario_switch.get('scenario_id', -1)
)PYTHON";
        PyRun_SimpleString(script);

        PyObject* main_module = PyImport_AddModule("__main__");
        PyObject* main_dict = PyModule_GetDict(main_module);
        PyObject* requested_obj = PyDict_GetItemString(main_dict, "requested");
        PyObject* scenario_id_obj = PyDict_GetItemString(main_dict, "scenario_id");

        bool requested = requested_obj && PyObject_IsTrue(requested_obj);
        if (requested && scenario_id_obj)
        {
            scenario_id = static_cast<int>(PyLong_AsLong(scenario_id_obj));
            // 重置请求标志
            PyRun_SimpleString("plt._mpc_scenario_switch['requested'] = False\n");
            return true;
        }
        return false;
    }

private:
    bool _initialized{false};
    bool _reference_cache_ready{false};
    std::vector<double> _ref_x;
    std::vector<double> _ref_y;
    std::vector<double> _planned_speed_time;
    std::vector<double> _planned_speed_values;
    std::map<int, std::string> _scenario_button_labels;

    // 固定坐标范围（避免窗口抖动）
    bool _fixed_bounds_set{false};
    double _fixed_min_x{0.0};
    double _fixed_max_x{10.0};
    double _fixed_min_y{-5.0};
    double _fixed_max_y{5.0};
};

class JackalLikeSimulation
{
public:
    explicit JackalLikeSimulation(const fs::path &config_dir)
    {
        loadConfiguration(config_dir);

        // 初始化模型检测器
        model_detector_ = std::make_unique<ModelDetector>();
        model_detector_->printModelInfo();

        control_frequency_ = CONFIG["control_frequency"].as<double>();
        dt_ = 1.0 / control_frequency_;
        horizon_steps_ = CONFIG["N"].as<int>();
        enable_output_ = CONFIG["enable_output"].as<bool>();
        deceleration_ = CONFIG["deceleration_at_infeasible"].as<double>();

        planner_ = std::make_unique<Planner>();
        global_guidance_ = std::make_shared<GuidancePlanner::GlobalGuidance>();
        global_guidance_->SetPlanningFrequency(control_frequency_);
        global_guidance_->DoNotPropagateNodes();

        data_.robot_area = defineRobotArea(CONFIG["robot"]["length"].as<double>(),
                                           CONFIG["robot"]["width"].as<double>(),
                                           CONFIG["n_discs"].as<int>());
        data_.past_trajectory = FixedSizeTrajectory(200);

        // 加载场景按钮配置
        loadScenarioButtonConfig();
    }

    void loadScenarioButtonConfig()
    {
        const std::string config_file = "scenarios/scenario_buttons.yaml";
        try
        {
            YAML::Node config = YAML::LoadFile(config_file);
            if (config["buttons"] && config["buttons"].IsSequence())
            {
                for (const auto& btn : config["buttons"])
                {
                    int id = btn["id"].as<int>();
                    std::string label = btn["label"].as<std::string>();
                    std::string short_label = btn["short_label"] ? btn["short_label"].as<std::string>() : label;
                    std::string scenario = btn["scenario"].as<std::string>();
                    scenario_button_map_[id] = scenario;
                    scenario_button_labels_[id] = short_label;
                    LOG_INFO("场景按钮 " << id << ": " << label << " -> " << scenario);
                }
            }
        }
        catch (const std::exception& e)
        {
            LOG_WARN("Failed to load scenario button config: " << e.what());
            LOG_WARN("场景切换功能将不可用");
        }
    }

    void loadScenarioFile(const std::string& filepath)
    {
        LOG_INFO("Loading scenario from: " << filepath);
        scenario_file_ = filepath;
        use_scenario_file_ = true;

        // 解析场景文件一次并缓存
        try
        {
            scenario_json_ = simple_json::Parser::parse_file(scenario_file_);
        }
        catch (const std::exception& e)
        {
            LOG_ERROR("Failed to parse scenario file: " << e.what());
            use_scenario_file_ = false;
        }
    }

    // 切换场景
    void switchScenario(int scenario_id)
    {
        auto it = scenario_button_map_.find(scenario_id);
        if (it == scenario_button_map_.end())
        {
            LOG_WARN("场景 ID " << scenario_id << " 未配置");
            return;
        }

        LOG_INFO("========================================");
        LOG_INFO("切换到场景 " << scenario_id << ": " << it->second);
        LOG_INFO("========================================");

        // 加载新场景
        loadScenarioFile(it->second);

        // 重置仿真状态
        resetSimulation();
    }

    // 重置仿真状态
    void resetSimulation()
    {
        LOG_INFO("重置仿真状态...");

        // 重置时间
        sim_time_ = 0.0;

        // 清空历史记录
        history_x_.clear();
        history_y_.clear();
        history_speed_.clear();
        history_time_.clear();

        // 重新初始化状态、参考路径和障碍物
        initializeState();
        buildReferencePath();
        initializeObstacles();

        // 通知规划器数据更新
        planner_->onDataReceived(data_, "reference_path");
        planner_->onDataReceived(data_, "dynamic obstacles");

        // 重置样条参数
        state_.set("spline", computeReferenceProgress());
        updateGuidanceTrajectories();

        // 添加初始历史点
        history_x_.push_back(state_.get("x"));
        history_y_.push_back(state_.get("y"));
        history_speed_.push_back(state_.get("v"));
        history_time_.push_back(sim_time_);

        // 清空可视化缓存
        visualizer_.resetLayout();

        LOG_INFO("仿真状态重置完成");
    }

    void run()
    {
        LOG_INFO("Preparing simulation environment");
        initializeState();
        buildReferencePath();
        initializeObstacles();

        planner_->onDataReceived(data_, "reference_path");
        planner_->onDataReceived(data_, "dynamic obstacles");

        state_.set("spline", computeReferenceProgress());
        updateGuidanceTrajectories();

        history_x_.push_back(state_.get("x"));
        history_y_.push_back(state_.get("y"));
        history_speed_.push_back(state_.get("v"));
        history_time_.push_back(sim_time_);

        visualizer_.initialize(scenario_button_labels_);
        visualizer_.resetLayout();

        RosTools::Instrumentor::Get().BeginSession("mpc_planner_pure_cpp_demo");

        const int max_iterations = static_cast<int>(control_frequency_ * max_sim_time_);
        int iteration = 0;

        while (g_running && iteration < max_iterations)
        {
            visualizer_.waitWhilePaused();

            // 检查场景切换请求
            int requested_scenario_id = -1;
            if (visualizer_.checkScenarioSwitch(requested_scenario_id))
            {
                switchScenario(requested_scenario_id);
                iteration = 0;  // 重置迭代计数
                continue;
            }

            const auto loop_start = std::chrono::steady_clock::now();


            data_.planning_start_time = std::chrono::system_clock::now();

            state_.set("spline", computeReferenceProgress());

            updateObstacles(dt_);
            updateGuidanceTrajectories();

            auto output = planner_->solveMPC(state_, data_);

            double v_cmd{0.0};
            double w_cmd{0.0};
            auto candidates = planner_->getTMPCandidates();


            if (enable_output_ && output.success)
            {
                v_cmd = planner_->getSolution(1, "v");
                w_cmd = planner_->getSolution(0, "w");
            }
            else
            {
                const double velocity = state_.get("v");
                const double velocity_after_brake = std::max(velocity - deceleration_ * dt_, 0.0);
                v_cmd = velocity_after_brake;
                w_cmd = 0.0;
            }

            integrateState(v_cmd, w_cmd, dt_);
            state_.set("spline", computeReferenceProgress());

            sim_time_ += dt_;
            history_x_.push_back(state_.get("x"));
            history_y_.push_back(state_.get("y"));
            history_speed_.push_back(state_.get("v"));
            history_time_.push_back(sim_time_);
            data_.past_trajectory.add(state_.getPos());

            visualizer_.update(state_, data_, output, candidates, history_x_, history_y_, history_speed_, history_time_, guidance_paths_, sim_time_, iteration);

            if (objectiveReached())
            {
                LOG_INFO("Objective reached, stopping simulation");
                break;
            }

            ++iteration;

            auto loop_end = std::chrono::steady_clock::now();
            const double loop_duration = std::chrono::duration<double>(loop_end - loop_start).count();
            const double sleep_time = dt_ - loop_duration;

            if (sleep_time > 0.0)
            {
                std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
            }
            else
            {
                LOG_WARN_THROTTLE(200, "Control loop overrun: " << loop_duration << " s (target " << dt_ << " s)");
            }
        }

        RosTools::Instrumentor::Get().EndSession();
        LOG_INFO("Simulation finished after " << iteration << " iterations and " << sim_time_ << " seconds");
    }

private:
    void loadConfiguration(const fs::path &config_dir)
    {
        fs::path config_file = config_dir;

        if (fs::is_directory(config_dir))
        {
            config_file = config_dir / "settings.yaml";
        }

        if (!fs::exists(config_file))
        {
            std::ostringstream err;
            err << "Config file not found: " << config_file;
            throw std::runtime_error(err.str());
        }

        LOG_INFO("Loading configuration from " << config_file);
        Configuration::getInstance().initialize(config_file.string());
    }

    void initializeState()
    {
        // 基础状态（所有模型都有）
        double start_x = 0.0;
        double start_y = 0.0;
        double start_yaw = 0.0;
        double start_v = 0.5;

        // 如果有场景文件，从场景文件读取起点
        if (use_scenario_file_)
        {
            try
            {
                if (scenario_json_.contains("startPose"))
                {
                    const auto& start = scenario_json_["startPose"];
                    if (!start.is_null())
                    {
                        start_x = start["x"].as_double();
                        start_y = start["y"].as_double();
                        start_yaw = start["yaw"].as_double();
                        LOG_INFO("✓ 从场景文件加载起点: (" << start_x << ", " << start_y << ", " << start_yaw << ")");
                    }
                }
            }
            catch (const std::exception& e)
            {
                LOG_WARN("Failed to load start pose from scenario: " << e.what());
            }
        }

        state_.set("x", start_x);
        state_.set("y", start_y);
        state_.set("psi", start_yaw);
        state_.set("v", start_v);

        // 自适应初始化：根据模型检测器初始化额外的状态
        if (model_detector_->hasState("delta"))
        {
            state_.set("delta", 0.0);
            LOG_INFO("✓ 初始化 delta 状态（自行车模型）");
        }

        if (model_detector_->hasState("slack"))
        {
            state_.set("slack", 0.0);
            LOG_INFO("✓ 初始化 slack 状态");
        }

        // 路径跟踪状态
        state_.set("spline", 0.0);

        reference_progress_initialized_ = false;
        reference_segment_ = -1;
        reference_parameter_ = 0.0;

        LOG_INFO("状态初始化完成");
    }

    void buildReferencePath()
    {
        data_.reference_path.clear();

        // 检查是否使用场景文件，如果是则根据起点和终点生成参考路径
        if (use_scenario_file_)
        {
            try
            {
                Eigen::Vector2d start_pos(state_.get("x"), state_.get("y"));
                Eigen::Vector2d goal_pos = start_pos + Eigen::Vector2d(20.0, 0.0); // 默认终点

                // 从场景文件读取终点
                if (scenario_json_.contains("goalPose"))
                {
                    const auto& goal = scenario_json_["goalPose"];
                    if (!goal.is_null())
                    {
                        goal_pos.x() = goal["x"].as_double();
                        goal_pos.y() = goal["y"].as_double();
                        LOG_INFO("✓ 从场景文件加载终点: (" << goal_pos.x() << ", " << goal_pos.y() << ")");
                    }
                }

                // 生成从起点到终点的直线参考路径
                const int points = 200;
                const double total_distance = (goal_pos - start_pos).norm();
                const Eigen::Vector2d direction = (goal_pos - start_pos).normalized();
                const double heading = std::atan2(direction.y(), direction.x());

                data_.reference_path.x.reserve(points);
                data_.reference_path.y.reserve(points);
                data_.reference_path.psi.reserve(points);
                data_.reference_path.v.reserve(points);
                data_.reference_path.s.reserve(points);

                for (int i = 0; i < points; ++i)
                {
                    const double alpha = static_cast<double>(i) / static_cast<double>(points - 1);
                    const Eigen::Vector2d point = start_pos + alpha * (goal_pos - start_pos);

                    data_.reference_path.x.push_back(point.x());
                    data_.reference_path.y.push_back(point.y());
                    data_.reference_path.psi.push_back(heading);
                    data_.reference_path.v.push_back(1.0); // 恒定参考速度
                    data_.reference_path.s.push_back(alpha * total_distance);
                }

                data_.goal = goal_pos;
                data_.goal_received = true;

                LOG_INFO("✓ 生成参考路径: 起点(" << start_pos.x() << ", " << start_pos.y()
                         << ") -> 终点(" << goal_pos.x() << ", " << goal_pos.y()
                         << "), 距离=" << total_distance << "m");

                // 设置固定坐标范围
                visualizer_.setFixedBounds(start_pos.x(), start_pos.y(), goal_pos.x(), goal_pos.y());
            }
            catch (const std::exception& e)
            {
                LOG_ERROR("从场景文件生成参考路径失败: " << e.what());
                LOG_WARN("使用默认参考路径");
                buildDefaultReferencePath();
                return;
            }
        }
        else
        {
            // 使用默认的正弦曲线参考路径
            buildDefaultReferencePath();
        }

        reference_spline_ = std::make_shared<RosTools::Spline2D>(data_.reference_path.x, data_.reference_path.y);
        reference_progress_initialized_ = false;
        reference_segment_ = -1;
        reference_parameter_ = 0.0;
    }

    void buildDefaultReferencePath()
    {
        const double segment_length = 20.0;
        const int points = 200;

        data_.reference_path.clear();
        data_.reference_path.x.reserve(points);
        data_.reference_path.y.reserve(points);
        data_.reference_path.psi.reserve(points);
        data_.reference_path.v.reserve(points);
        data_.reference_path.s.clear();
        data_.reference_path.s.reserve(points);

        Eigen::Vector2d prev_point(0.0, 0.0);
        for (int i = 0; i < points; ++i)
        {
            const double s = segment_length * static_cast<double>(i) / static_cast<double>(points - 1);
            const double curvature = 0.15;
            const double y = 1.5 * std::sin(curvature * 0.5 * s);

            Eigen::Vector2d current_point(s, y);

            data_.reference_path.x.push_back(s);
            data_.reference_path.y.push_back(y);
            data_.reference_path.psi.push_back(std::atan2(curvature * 0.75 * std::cos(curvature * 0.5 * s), 1.0));
            data_.reference_path.v.push_back(std::max(0.1, 1.0 + 0.3 * std::cos(0.2 * s)));

            if (i == 0)
                data_.reference_path.s.push_back(0.0);
            else
                data_.reference_path.s.push_back(data_.reference_path.s.back() + (current_point - prev_point).norm());

            prev_point = current_point;
        }

        const auto goal_index = data_.reference_path.x.size() - 1;
        data_.goal = Eigen::Vector2d(data_.reference_path.x[goal_index], data_.reference_path.y[goal_index]);
        data_.goal_received = true;
    }

    void loadDefaultObstacles()
    {
        sim_obstacles_.clear();

        SimObstacle o1;
        o1.id = 0;
        o1.position = Eigen::Vector2d(6.0, 0.8);
        o1.velocity = Eigen::Vector2d(-0.3, 0.0);
        o1.radius = 0.45;
        o1.bounce_x = true;
        o1.min_x = 2.0;
        o1.max_x = 12.0;

        SimObstacle o2;
        o2.id = 1;
        o2.position = Eigen::Vector2d(10.0, -1.0);
        o2.velocity = Eigen::Vector2d(0.0, 0.4);
        o2.radius = 0.35;
        o2.bounce_y = true;
        o2.min_y = -1.5;
        o2.max_y = 1.5;

        SimObstacle o3;
        o3.id = 2;
        o3.position = Eigen::Vector2d(14.0, 0.0);
        o3.velocity = Eigen::Vector2d(-0.2, -0.35);
        o3.radius = 0.4;
        o3.bounce_x = true;
        o3.bounce_y = true;
        o3.min_x = 8.0;
        o3.max_x = 18.0;
        o3.min_y = -2.0;
        o3.max_y = 2.0;

        SimObstacle s1;
        s1.id = 3;
        s1.position = Eigen::Vector2d(4.0, 1.5);
        s1.velocity = Eigen::Vector2d::Zero();
        s1.radius = 0.35;
        s1.is_static = true;

        SimObstacle s2;
        s2.id = 4;
        s2.position = Eigen::Vector2d(8.5, -1.2);
        s2.velocity = Eigen::Vector2d::Zero();
        s2.radius = 0.4;
        s2.is_static = true;

        SimObstacle s3;
        s3.id = 5;
        s3.position = Eigen::Vector2d(11.5, 1.1);
        s3.velocity = Eigen::Vector2d::Zero();
        s3.radius = 0.35;
        s3.is_static = true;

        SimObstacle s4;
        s4.id = 6;
        s4.position = Eigen::Vector2d(16.0, -0.6);
        s4.velocity = Eigen::Vector2d::Zero();
        s4.radius = 0.45;
        s4.is_static = true;

        sim_obstacles_ = {o1, o2, o3, s1, s2, s3, s4};
        LOG_INFO("使用默认障碍物配置（7 个障碍物）");
    }

    // 矩形障碍物近似为一个外接圆
    std::vector<SimObstacle> approximateRectangle(int& id_counter, const Eigen::Vector2d& center,
                                                   double width, double height, double yaw,
                                                   const Eigen::Vector2d& velocity, bool is_static)
    {
        std::vector<SimObstacle> circles;

        // 计算外接圆半径：矩形对角线的一半
        const double radius = std::sqrt(width * width + height * height) / 2.0;

        SimObstacle obs;
        obs.id = id_counter++;
        obs.position = center;
        obs.velocity = velocity;
        obs.radius = radius;
        obs.is_static = is_static;
        obs.bounce_x = false;
        obs.bounce_y = false;

        // 保存矩形信息用于可视化
        obs.is_rectangle = true;
        obs.rect_width = width;
        obs.rect_height = height;
        obs.rect_yaw = yaw;

        circles.push_back(obs);

        LOG_INFO("  矩形近似: " << width << "x" << height << " -> 外接圆 (半径=" << radius << ")");
        return circles;
    }

    void initializeObstacles()
    {
        sim_obstacles_.clear();

        // 如果有场景文件，从场景文件加载障碍物
        if (use_scenario_file_)
        {
            try
            {
                if (!scenario_json_.contains("obstacles"))
                {
                    LOG_WARN("场景文件中没有 obstacles 字段");
                    return;
                }

                const auto& obstacles = scenario_json_["obstacles"];
                int id_counter = 0;

                // 1. 加载静态圆形障碍物
                if (obstacles.contains("circles") && obstacles["circles"].is_array())
                {
                    auto circles = obstacles["circles"];
                    for (size_t i = 0; i < circles.size(); ++i)
                    {
                        auto c = circles[i];
                        double x = c["x"].as_double();
                        double y = c["y"].as_double();
                        double radius = c["radius"].as_double();

                        SimObstacle obs;
                        obs.id = id_counter++;
                        obs.position = Eigen::Vector2d(x, y);
                        obs.radius = radius;
                        obs.velocity = Eigen::Vector2d::Zero();
                        obs.is_static = true;
                        sim_obstacles_.push_back(obs);
                    }
                    LOG_INFO("✓ 加载了 " << circles.size() << " 个静态圆形障碍物");
                }

                // 2. 加载静态多边形障碍物（近似为多个圆）
                if (obstacles.contains("polygons") && obstacles["polygons"].is_array())
                {
                    const auto& polygons = obstacles["polygons"];
                    int poly_count = 0;
                    for (size_t i = 0; i < polygons.size(); ++i)
                    {
                        const auto& poly = polygons[i];
                        if (!poly.contains("points") || !poly["points"].is_array())
                            continue;

                        const auto& points = poly["points"];
                        if (points.size() < 3)
                            continue;

                        // 计算多边形的边界框和中心
                        double min_x = points[0]["x"].as_double();
                        double max_x = min_x;
                        double min_y = points[0]["y"].as_double();
                        double max_y = min_y;

                        for (size_t j = 1; j < points.size(); ++j)
                        {
                            double px = points[j]["x"].as_double();
                            double py = points[j]["y"].as_double();
                            min_x = std::min(min_x, px);
                            max_x = std::max(max_x, px);
                            min_y = std::min(min_y, py);
                            max_y = std::max(max_y, py);
                        }

                        // 用矩形近似多边形
                        double center_x = (min_x + max_x) / 2.0;
                        double center_y = (min_y + max_y) / 2.0;
                        double width = max_x - min_x;
                        double height = max_y - min_y;

                        auto rect_circles = approximateRectangle(id_counter, Eigen::Vector2d(center_x, center_y),
                                                                  width, height, 0.0, Eigen::Vector2d::Zero(), true);
                        sim_obstacles_.insert(sim_obstacles_.end(), rect_circles.begin(), rect_circles.end());
                        poly_count++;
                    }
                    if (poly_count > 0)
                        LOG_INFO("✓ 加载了 " << poly_count << " 个多边形障碍物");
                }

                // 3. 加载动态障碍物
                if (obstacles.contains("dynamic") && obstacles["dynamic"].is_array())
                {
                    const auto& dynamics = obstacles["dynamic"];
                    int dyn_count = 0;
                    for (size_t i = 0; i < dynamics.size(); ++i)
                    {
                        const auto& dyn = dynamics[i];
                        if (!dyn.contains("kind") || !dyn.contains("state") || !dyn.contains("data"))
                            continue;

                        std::string kind = dyn["kind"].as_string();
                        const auto& state = dyn["state"];
                        const auto& data = dyn["data"];

                        Eigen::Vector2d pos(state["x"].as_double(), state["y"].as_double());
                        Eigen::Vector2d vel(state["vx"].as_double(), state["vy"].as_double());

                        if (kind == "circle")
                        {
                            SimObstacle obs;
                            obs.id = id_counter++;
                            obs.position = pos;
                            obs.velocity = vel;
                            obs.radius = data["r"].as_double();
                            obs.is_static = false;
                            obs.bounce_x = false;
                            obs.bounce_y = false;
                            sim_obstacles_.push_back(obs);
                            dyn_count++;
                        }
                        else if (kind == "rect")
                        {
                            double width = data["w"].as_double();
                            double height = data["h"].as_double();
                            double yaw = data["yaw"].as_double();

                            auto rect_circles = approximateRectangle(id_counter, pos, width, height, yaw, vel, false);
                            sim_obstacles_.insert(sim_obstacles_.end(), rect_circles.begin(), rect_circles.end());
                            dyn_count++;
                        }
                    }
                    if (dyn_count > 0)
                        LOG_INFO("✓ 加载了 " << dyn_count << " 个动态障碍物");
                }

                LOG_INFO("场景加载完成，共 " << sim_obstacles_.size() << " 个障碍物圆形");
            }
            catch (const std::exception& e)
            {
                LOG_ERROR("加载场景文件失败: " << e.what());
                LOG_WARN("使用默认障碍物配置");
                loadDefaultObstacles();
            }
        }
        else
        {
            // 使用默认障碍物配置
            loadDefaultObstacles();
        }

        data_.dynamic_obstacles.clear();

        for (const auto &sim_obs : sim_obstacles_)
        {
            DynamicObstacle dyn(sim_obs.id, sim_obs.position, 0.0, sim_obs.radius);
            dyn.prediction = getConstantVelocityPrediction(sim_obs.position,
                                                           sim_obs.velocity,
                                                           CONFIG["integrator_step"].as<double>(),
                                                           horizon_steps_);
            if (sim_obs.is_static)
                dyn.type = ObstacleType::STATIC;
            data_.dynamic_obstacles.push_back(dyn);
        }

        ensureObstacleSize(data_.dynamic_obstacles, state_);
    }

    void updateObstacles(double dt)
    {
        const int steps = horizon_steps_;
        const double integrator_step = CONFIG["integrator_step"].as<double>();

        for (size_t i = 0; i < sim_obstacles_.size(); ++i)
        {
            auto &sim_obs = sim_obstacles_[i];
            sim_obs.update(dt);

            auto &dyn = data_.dynamic_obstacles[i];
            dyn.position = sim_obs.position;
            dyn.prediction = getConstantVelocityPrediction(sim_obs.position,
                                                           sim_obs.velocity,
                                                           integrator_step,
                                                           steps);
        }

        planner_->onDataReceived(data_, "dynamic obstacles");
    }

    void integrateState(double v, double w, double dt)
    {
        const double prev_v = state_.get("v");

        double x = state_.get("x");
        double y = state_.get("y");
        double psi = state_.get("psi");

        // 自适应模型：根据模型检测器选择不同的运动学模型
        if (model_detector_->hasState("delta"))
        {
            // ========================================
            // 自行车模型（Bicycle Model）
            // ========================================
            const double prev_delta = state_.get("delta");
            double delta = prev_delta;

            // 从模型检测器获取参数
            const double wheel_base = model_detector_->getWheelBase();
            const double max_delta = model_detector_->getMaxSteeringAngle();

            const double lr = wheel_base / 2.0;
            const double lf = wheel_base / 2.0;
            const double ratio = lr / (lr + lf);

            // 计算 beta（侧偏角）
            const double beta = std::atan(ratio * std::tan(delta));

            // 自行车模型运动学方程
            x += v * std::cos(psi + beta) * dt;
            y += v * std::sin(psi + beta) * dt;
            psi += (v / lr) * std::sin(beta) * dt;
            delta += w * dt;  // w 是转向角速度

            // 限制转向角
            if (delta > max_delta)
                delta = max_delta;
            else if (delta < -max_delta)
                delta = -max_delta;

            state_.set("delta", delta);
        }
        else
        {
            // ========================================
            // 单轮模型（Unicycle Model）
            // ========================================
            // 单轮模型运动学方程
            x += v * std::cos(psi) * dt;
            y += v * std::sin(psi) * dt;
            psi += w * dt;  // w 是角速度
        }

        // 限制航向角在 [-pi, pi]
        if (psi > kPi)
            psi -= 2.0 * kPi;
        else if (psi < -kPi)
            psi += 2.0 * kPi;

        // 更新状态
        state_.set("x", x);
        state_.set("y", y);
        state_.set("psi", psi);
        state_.set("v", v);
        state_.set("w", w);
        state_.set("a", (v - prev_v) / dt);
    }

    bool objectiveReached() const
    {
        const Eigen::Vector2d diff = state_.getPos() - data_.goal;
        return diff.norm() < 0.5;
    }

    double computeReferenceProgress()
    {
        if (!reference_spline_)
            return 0.0;

        if (!reference_progress_initialized_)
        {
            reference_spline_->initializeClosestPoint(state_.getPos(), reference_segment_, reference_parameter_);
            reference_progress_initialized_ = true;
        }
        else
        {
            reference_spline_->findClosestPoint(state_.getPos(), reference_segment_, reference_parameter_);
        }

        return reference_parameter_;
    }

    void updateGuidanceTrajectories()
    {
        guidance_paths_.clear();

        if (!global_guidance_ || !reference_spline_)
            return;

        double spline_position = computeReferenceProgress();
        double robot_radius = data_.robot_area.empty() ? 0.0 : data_.robot_area.front().radius;

        std::vector<GuidancePlanner::Obstacle> obstacles;
        obstacles.reserve(data_.dynamic_obstacles.size());

        for (const auto &obs : data_.dynamic_obstacles)
        {
            if (obs.index < 0)
                continue;

            std::vector<Eigen::Vector2d> positions;
            positions.reserve(1 + (obs.prediction.modes.empty() ? 0 : obs.prediction.modes.front().size()));
            positions.push_back(obs.position);
            if (!obs.prediction.modes.empty())
            {
                for (const auto &step : obs.prediction.modes.front())
                    positions.push_back(step.position);
            }

            obstacles.emplace_back(obs.index, positions, obs.radius + robot_radius);
        }

        global_guidance_->LoadObstacles(obstacles, {});
        global_guidance_->SetStart(state_.getPos(), state_.get("psi"), state_.get("v"));

        double reference_velocity = std::max(0.5, state_.get("v"));
        if (!data_.reference_path.x.empty())
        {
            double best_dist = std::numeric_limits<double>::infinity();
            size_t best_idx = 0;
            const double robot_x = state_.get("x");
            const double robot_y = state_.get("y");

            for (size_t i = 0; i < data_.reference_path.x.size(); ++i)
            {
                double dx = data_.reference_path.x[i] - robot_x;
                double dy = data_.reference_path.y[i] - robot_y;
                double dist = std::hypot(dx, dy);
                if (dist < best_dist)
                {
                    best_dist = dist;
                    best_idx = i;
                }
            }

            reference_velocity = std::max(0.3, data_.reference_path.v[best_idx]);
        }
        global_guidance_->SetReferenceVelocity(std::max(0.3, reference_velocity));

        double road_half_width = CONFIG["road"]["width"].as<double>() / 2.0 - robot_radius - 0.1;
        road_half_width = std::max(0.5, road_half_width);

        global_guidance_->LoadReferencePath(std::max(0.0, spline_position), reference_spline_, road_half_width, road_half_width);

        bool success = false;
        try
        {
            success = global_guidance_->Update();
        }
        catch (const std::exception &e)
        {
            LOG_WARN_THROTTLE(5000, "Guidance planner update failed: " << e.what());
            return;
        }

        if (!success || global_guidance_->NumberOfGuidanceTrajectories() == 0)
            return;

        int selected_topology = -1;
        try
        {
            auto &selected = global_guidance_->GetUsedTrajectory();
            selected_topology = selected.topology_class;
        }
        catch (...)
        {
            selected_topology = -1;
        }

        for (int i = 0; i < global_guidance_->NumberOfGuidanceTrajectories(); ++i)
        {
            auto &traj = global_guidance_->GetGuidanceTrajectory(i);
            RosTools::Spline2D &trajectory_spline = traj.spline.GetTrajectory();

            int samples = 80;
            std::vector<Eigen::Vector2d> points;
            points.reserve(samples + 1);
            double end_param = trajectory_spline.parameterLength();
            for (int k = 0; k <= samples; ++k)
            {
                double t = (samples == 0) ? 0.0 : end_param * static_cast<double>(k) / static_cast<double>(samples);
                points.push_back(trajectory_spline.getPoint(t));
            }

            GuidancePath path;
            path.points = std::move(points);
            path.color_index = traj.color_;
            path.selected = (traj.topology_class == selected_topology);
            guidance_paths_.push_back(std::move(path));
        }
    }

    std::unique_ptr<Planner> planner_;
    State state_;
    RealTimeData data_;
    MatplotlibVisualizer visualizer_;
    std::vector<SimObstacle> sim_obstacles_;

    std::vector<double> history_x_;
    std::vector<double> history_y_;
    std::vector<double> history_speed_;
    std::vector<double> history_time_;
    std::vector<GuidancePath> guidance_paths_;

    std::shared_ptr<GuidancePlanner::GlobalGuidance> global_guidance_;
    std::shared_ptr<RosTools::Spline2D> reference_spline_;
    bool reference_progress_initialized_{false};
    int reference_segment_{-1};
    double reference_parameter_{0.0};

    double control_frequency_{20.0};
    double dt_{0.05};
    int horizon_steps_{30};
    double deceleration_{3.0};
    bool enable_output_{true};

    double sim_time_{0.0};
    const double max_sim_time_{25.0};

    // 模型检测器
    std::unique_ptr<ModelDetector> model_detector_;

    // 场景配置
    std::string scenario_file_;
    bool use_scenario_file_{false};
    simple_json::Value scenario_json_;  // 缓存解析后的场景数据
    std::map<int, std::string> scenario_button_map_;  // 场景按钮ID到场景文件的映射
    std::map<int, std::string> scenario_button_labels_;  // 场景按钮ID到显示标签的映射
};

} // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::cout << "========================================\n";
    std::cout << "MPC Planner - Pure C++ Jackal Simulator\n";
    std::cout << "========================================\n";

    // 智能查找配置文件路径
    fs::path config_path;
    std::string scenario_file;

    if (argc > 1)
    {
        // 用户指定了配置路径
        config_path = argv[1];
    }
    else
    {
        // 自动查找配置文件
        std::vector<fs::path> search_paths = {
            "mpc_planner_jackalsimulator/config",           // 从项目根目录运行
            "../mpc_planner_jackalsimulator/config",        // 从 build 目录运行
            "../../mpc_planner_jackalsimulator/config",     // 从 build/pure_cpp 运行
        };

        for (const auto &path : search_paths)
        {
            if (fs::exists(path / "settings.yaml"))
            {
                config_path = path;
                break;
            }
        }

        if (config_path.empty())
        {
            std::cerr << "\n[ERROR] Could not find configuration file!\n";
            std::cerr << "Searched in:\n";
            for (const auto &path : search_paths)
            {
                std::cerr << "  - " << fs::absolute(path) << "\n";
            }
            std::cerr << "\nPlease specify config path: " << argv[0] << " <config_path> [scenario_file]\n";
            return 1;
        }
    }

    // 检查是否指定了场景文件（第二个参数）
    if (argc > 2)
    {
        scenario_file = argv[2];
        if (!fs::exists(scenario_file))
        {
            std::cerr << "\n[ERROR] Scenario file not found: " << scenario_file << "\n";
            return 1;
        }
        std::cout << "Using scenario file: " << scenario_file << "\n";
    }

    try
    {
        JackalLikeSimulation simulation(config_path);

        // 如果指定了场景文件，加载场景
        if (!scenario_file.empty())
        {
            simulation.loadScenarioFile(scenario_file);
        }

        simulation.run();

        std::cout << "\nSimulation completed successfully.\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "\nFatal error: " << e.what() << '\n';
        return 1;
    }
}
