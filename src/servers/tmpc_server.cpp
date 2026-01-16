#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <nlohmann/json.hpp>

#include <Eigen/Dense>

#include <mpc_planner/planner.h>
#include <mpc_planner/data_preparation.h>
#include <mpc_planner_types/realtime_data.h>
#include <mpc_planner_solver/state.h>
#include <mpc_planner_util/parameters.h>
#include <guidance_planner/global_guidance.h>
#include <ros_tools/spline.h>
#include <ros_tools/profiling.h>

using json = nlohmann::json;
using Server = websocketpp::server<websocketpp::config::asio>;

namespace {
constexpr double kEpsilon = 1e-6;
constexpr double kHalfPi = 1.5707963267948966;

double getOptionalDouble(const json &node, const char *key, double fallback);
int getOptionalInt(const json &node, const char *key, int fallback);

struct ParsedPrediction {
  std::vector<std::vector<Eigen::Vector2d>> positions;
  std::vector<std::vector<double>> yaws;
  std::vector<double> probabilities;
};

[[maybe_unused]] bool parsePrediction(const json &obs, ParsedPrediction &out) {
  if (!obs.contains("prediction") || !obs["prediction"].is_object()) return false;
  const auto &pred = obs["prediction"];
  if (!pred.contains("modes") || !pred["modes"].is_array()) return false;
  const auto &modes = pred["modes"];
  if (modes.empty()) return false;
  out.positions.clear();
  out.yaws.clear();
  out.probabilities.clear();
  for (const auto &mode : modes) {
    if (!mode.contains("points") || !mode["points"].is_array()) continue;
    const auto &points = mode["points"];
    if (points.empty()) continue;
    std::vector<Eigen::Vector2d> pos_list;
    std::vector<double> yaw_list;
    pos_list.reserve(points.size());
    yaw_list.reserve(points.size());
    for (const auto &pt : points) {
      const double px = getOptionalDouble(pt, "x", 0.0);
      const double py = getOptionalDouble(pt, "y", 0.0);
      pos_list.emplace_back(px, py);
      yaw_list.push_back(getOptionalDouble(pt, "yaw", 0.0));
    }
    out.positions.push_back(std::move(pos_list));
    out.yaws.push_back(std::move(yaw_list));
    out.probabilities.push_back(getOptionalDouble(mode, "probability", 1.0));
  }
  if (out.positions.empty()) return false;
  double sum = 0.0;
  for (double p : out.probabilities) sum += p;
  if (sum <= kEpsilon) {
    const double uniform = 1.0 / static_cast<double>(out.probabilities.size());
    for (double &p : out.probabilities) p = uniform;
  } else {
    for (double &p : out.probabilities) p /= sum;
  }
  return true;
}

[[maybe_unused]] MPCPlanner::Prediction buildPredictionFromParsed(const ParsedPrediction &parsed,
                                                                  const Eigen::Vector2d &offset) {
  const bool probabilistic = CONFIG["probabilistic"]["enable"].as<bool>();
  const bool multi_mode = parsed.positions.size() > 1;
  MPCPlanner::Prediction prediction(
      multi_mode
          ? MPCPlanner::PredictionType::NONGAUSSIAN
          : (probabilistic ? MPCPlanner::PredictionType::GAUSSIAN
                           : MPCPlanner::PredictionType::DETERMINISTIC));
  prediction.modes.resize(parsed.positions.size());
  prediction.probabilities = parsed.probabilities;
  const double noise = probabilistic ? 0.3 : 0.0;
  for (size_t m = 0; m < parsed.positions.size(); ++m) {
    const auto &mode_pos = parsed.positions[m];
    const auto &mode_yaw = parsed.yaws[m];
    auto &mode = prediction.modes[m];
    mode.reserve(mode_pos.size());
    for (size_t i = 0; i < mode_pos.size(); ++i) {
      const Eigen::Vector2d pos = mode_pos[i] + offset;
      const double yaw = i < mode_yaw.size() ? mode_yaw[i] : 0.0;
      mode.emplace_back(pos, yaw, noise, noise);
    }
  }
  if (probabilistic && prediction.type == MPCPlanner::PredictionType::GAUSSIAN) {
    MPCPlanner::propagatePredictionUncertainty(prediction);
  }
  return prediction;
}

double getOptionalDouble(const json &node, const char *key, double fallback) {
  if (!node.is_object() || !node.contains(key)) return fallback;
  const auto &val = node.at(key);
  if (!val.is_number()) return fallback;
  return val.get<double>();
}

int getOptionalInt(const json &node, const char *key, int fallback) {
  if (!node.is_object() || !node.contains(key)) return fallback;
  const auto &val = node.at(key);
  if (!val.is_number_integer()) return fallback;
  return val.get<int>();
}

struct TmpcContext {
  std::unique_ptr<MPCPlanner::Planner> planner;
  std::shared_ptr<GuidancePlanner::GlobalGuidance> guidance;
  MPCPlanner::RealTimeData data;
  MPCPlanner::State state;
  std::shared_ptr<RosTools::Spline2D> reference_spline;
  int horizon_steps = 30;
  double integrator_step = 0.2;
  int max_obstacles = 50;
  int n_discs = 1;
  double robot_radius = 0.325;
  double robot_length = 0.65;
  double robot_width = 0.65;
  double road_width = 4.0;
  bool verbose = false;
};

json buildConfigMessage(const TmpcContext &ctx) {
  json msg;
  msg["type"] = "config";
  msg["tmpc"] = {
      {"horizon_steps", ctx.horizon_steps},
      {"integrator_step", ctx.integrator_step},
      {"max_obstacles", ctx.max_obstacles},
      {"n_discs", ctx.n_discs},
      {"robot_length", ctx.robot_length},
      {"robot_width", ctx.robot_width},
      {"robot_radius", ctx.robot_radius},
      {"road_width", ctx.road_width},
  };
  return msg;
}

json buildReferenceDebug(const MPCPlanner::ReferencePath &reference_path) {
  json out = json::array();
  const size_t count = reference_path.x.size();
  for (size_t i = 0; i < count; ++i) {
    json p;
    p["x"] = reference_path.x[i];
    p["y"] = reference_path.y[i];
    if (i < reference_path.psi.size()) {
      p["yaw"] = reference_path.psi[i];
    }
    out.push_back(p);
  }
  return out;
}

const char *predictionTypeLabel(MPCPlanner::PredictionType type) {
  switch (type) {
    case MPCPlanner::PredictionType::DETERMINISTIC:
      return "DETERMINISTIC";
    case MPCPlanner::PredictionType::GAUSSIAN:
      return "GAUSSIAN";
    case MPCPlanner::PredictionType::NONGAUSSIAN:
      return "NONGAUSSIAN";
    case MPCPlanner::PredictionType::NONE:
      return "NONE";
    default:
      return "UNKNOWN";
  }
}

json buildDynamicObstacleDump(const std::vector<MPCPlanner::DynamicObstacle> &obstacles) {
  json out = json::array();
  for (const auto &obs : obstacles) {
    json entry;
    entry["index"] = obs.index;
    entry["position"] = {{"x", obs.position.x()}, {"y", obs.position.y()}};
    entry["angle"] = obs.angle;
    entry["radius"] = obs.radius;
    entry["type"] = (obs.type == MPCPlanner::ObstacleType::STATIC) ? "STATIC" : "DYNAMIC";
    json pred;
    pred["type"] = predictionTypeLabel(obs.prediction.type);
    pred["probabilities"] = obs.prediction.probabilities;
    json modes = json::array();
    for (size_t m = 0; m < obs.prediction.modes.size(); ++m) {
      const auto &mode = obs.prediction.modes[m];
      json mode_entry;
      mode_entry["index"] = static_cast<int>(m);
      json steps = json::array();
      for (const auto &step : mode) {
        json s;
        s["x"] = step.position.x();
        s["y"] = step.position.y();
        s["angle"] = step.angle;
        s["major_radius"] = step.major_radius;
        s["minor_radius"] = step.minor_radius;
        steps.push_back(s);
      }
      mode_entry["steps"] = steps;
      modes.push_back(mode_entry);
    }
    pred["modes"] = modes;
    entry["prediction"] = pred;
    out.push_back(entry);
  }
  return out;
}

json buildGuidanceDebug(GuidancePlanner::GlobalGuidance &guidance, int samples_per_path) {
  json out = json::array();
  const int count = guidance.NumberOfGuidanceTrajectories();
  for (int i = 0; i < count; ++i) {
    auto &traj = guidance.GetGuidanceTrajectory(i);
    json path = json::array();
    const auto &spline = traj.spline.GetTrajectory();
    const double length = spline.parameterLength();
    const int samples = std::max(2, samples_per_path);
    for (int s = 0; s < samples; ++s) {
      const double t = length * static_cast<double>(s) / static_cast<double>(samples - 1);
      const Eigen::Vector2d point = spline.getPoint(t);
      const Eigen::Vector2d vel = spline.getVelocity(t);
      json p;
      p["x"] = point.x();
      p["y"] = point.y();
      if (vel.squaredNorm() > 1e-8) {
        p["yaw"] = std::atan2(vel.y(), vel.x());
      }
      path.push_back(p);
    }
    json entry;
    entry["id"] = i;
    entry["topologyClass"] = traj.topology_class;
    entry["points"] = path;
    out.push_back(entry);
  }
  return out;
}

bool convertReferencePath(const std::vector<Eigen::Vector3d> &reference_line,
                          MPCPlanner::ReferencePath &reference_path,
                          std::shared_ptr<RosTools::Spline2D> &spline_out) {
  reference_path.clear();
  if (reference_line.empty()) return false;

  const size_t points = reference_line.size();
  reference_path.x.reserve(points);
  reference_path.y.reserve(points);
  reference_path.psi.reserve(points);
  reference_path.v.reserve(points);
  reference_path.s.reserve(points);

  double s = 0.0;
  for (size_t i = 0; i < points; ++i) {
    reference_path.x.push_back(reference_line[i].x());
    reference_path.y.push_back(reference_line[i].y());
    reference_path.psi.push_back(reference_line[i].z());
    reference_path.v.push_back(1.0);
    if (i == 0) {
      reference_path.s.push_back(0.0);
    } else {
      const double dx = reference_line[i].x() - reference_line[i - 1].x();
      const double dy = reference_line[i].y() - reference_line[i - 1].y();
      s += std::hypot(dx, dy);
      reference_path.s.push_back(s);
    }
  }

  std::vector<double> xs(reference_path.x.begin(), reference_path.x.end());
  std::vector<double> ys(reference_path.y.begin(), reference_path.y.end());
  spline_out = std::make_shared<RosTools::Spline2D>(xs, ys);
  return true;
}

double computeReferenceProgress(const MPCPlanner::State &state,
                                const std::shared_ptr<RosTools::Spline2D> &spline) {
  if (!spline) return 0.0;
  Eigen::Vector2d robot_pos = state.getPos();
  double best_param = 0.0;
  double best_dist = std::numeric_limits<double>::infinity();
  double max_param = spline->parameterLength();
  int samples = 100;
  for (int i = 0; i <= samples; ++i) {
    double param = max_param * i / samples;
    Eigen::Vector2d spline_point = spline->getPoint(param);
    double dist = (spline_point - robot_pos).norm();
    if (dist < best_dist) {
      best_dist = dist;
      best_param = param;
    }
  }
  return best_param;
}

void updateGuidanceTrajectories(const MPCPlanner::State &state, TmpcContext &ctx) {
  if (!ctx.guidance || !ctx.reference_spline) return;
  double spline_position = state.get("spline");
  double robot_radius = ctx.data.robot_area.empty() ? 0.0 : ctx.data.robot_area.front().radius;

  ctx.guidance->SetStart(state.getPos(), state.get("psi"), state.get("v"));
  ctx.guidance->SetReferenceVelocity(std::max(0.5, state.get("v")));
  ctx.guidance->LoadReferencePath(std::max(0.0, spline_position), ctx.reference_spline, ctx.road_width);

  std::vector<GuidancePlanner::Obstacle> obstacles;
  obstacles.reserve(ctx.data.dynamic_obstacles.size());
  for (const auto &obs : ctx.data.dynamic_obstacles) {
    if (obs.index < 0) continue;
    std::vector<Eigen::Vector2d> positions;
    positions.reserve(1 + (obs.prediction.modes.empty() ? 0 : obs.prediction.modes.front().size()));
    positions.push_back(obs.position);
    if (!obs.prediction.modes.empty()) {
      for (const auto &step : obs.prediction.modes.front()) {
        positions.push_back(step.position);
      }
    }
    if (positions.size() < 2) continue;
    bool valid = true;
    for (const auto &pos : positions) {
      if (!std::isfinite(pos.x()) || !std::isfinite(pos.y())) {
        valid = false;
        break;
      }
    }
    if (!valid) continue;
    obstacles.emplace_back(obs.index, positions, obs.radius + robot_radius);
  }
  std::vector<GuidancePlanner::Halfspace> static_obstacles;
  ctx.guidance->LoadObstacles(obstacles, static_obstacles);
  try {
    bool success = ctx.guidance->Update();
    std::cout << "[GUIDANCE] ctx.guidance->Update() success=" << (success ? "true" : "FALSE")
              << " num_trajectories=" << ctx.guidance->NumberOfGuidanceTrajectories()
              << std::endl;
  } catch (const std::exception &e) {
    std::cout << "[GUIDANCE] ctx.guidance->Update() EXCEPTION: " << e.what() << std::endl;
  } catch (...) {
    std::cout << "[GUIDANCE] ctx.guidance->Update() UNKNOWN EXCEPTION" << std::endl;
  }
}

}  // namespace

int main(int argc, char **argv) {
  int port = 8083;
  std::string config_file = "src/tmpc/mpc_planner_jackalsimulator/config/settings.yaml";
  if (argc > 1) {
    port = std::atoi(argv[1]);
  }
  if (argc > 2) {
    config_file = argv[2];
  }

  TmpcContext ctx;

  try {
    Configuration::getInstance().initialize(config_file);
    ctx.horizon_steps = CONFIG["N"].as<int>();
    ctx.integrator_step = CONFIG["integrator_step"].as<double>();
    ctx.max_obstacles = CONFIG["max_obstacles"].as<int>();
    ctx.n_discs = CONFIG["n_discs"].as<int>();
    ctx.robot_radius = CONFIG["robot_radius"].as<double>();
    ctx.robot_length = CONFIG["robot"]["length"].as<double>();
    ctx.robot_width = CONFIG["robot"]["width"].as<double>();
    if (CONFIG["road"] && CONFIG["road"]["width"]) {
      ctx.road_width = CONFIG["road"]["width"].as<double>();
    }
  } catch (const std::exception &e) {
    std::cerr << "[TMPC] Failed to load config: " << e.what() << std::endl;
    return 1;
  }

  ctx.planner = std::make_unique<MPCPlanner::Planner>();
  ctx.guidance = std::make_shared<GuidancePlanner::GlobalGuidance>();

  ctx.data.robot_area = MPCPlanner::defineRobotArea(ctx.robot_length, ctx.robot_width, ctx.n_discs);
  ctx.data.past_trajectory = MPCPlanner::FixedSizeTrajectory(200);
  ctx.state.initialize();

  Server server;
  server.init_asio();
  server.set_reuse_addr(true);
  server.clear_access_channels(websocketpp::log::alevel::all);
  server.clear_error_channels(websocketpp::log::elevel::all);
  server.set_error_channels(websocketpp::log::elevel::rerror);
  server.set_open_handler([&ctx, &server](websocketpp::connection_hdl hdl) {
    server.send(hdl, buildConfigMessage(ctx).dump(), websocketpp::frame::opcode::text);
  });

  server.set_message_handler([&ctx, &server](websocketpp::connection_hdl hdl, Server::message_ptr msg) {
    json response;
    try {
      json payload = json::parse(msg->get_payload());
      if (!payload.is_object()) {
        response["type"] = "error";
        response["message"] = "Invalid payload";
        server.send(hdl, response.dump(), websocketpp::frame::opcode::text);
        return;
      }
      const std::string type = payload.value("type", "");
      if (type == "config") {
        server.send(hdl, buildConfigMessage(ctx).dump(), websocketpp::frame::opcode::text);
        return;
      }
      if (type != "plan") {
        response["type"] = "error";
        response["message"] = "Invalid payload";
        server.send(hdl, response.dump(), websocketpp::frame::opcode::text);
        return;
      }

      const auto &start_node = payload.at("start");
      const auto &goal_node = payload.at("goal");
      Eigen::Vector2d start(getOptionalDouble(start_node, "x", 0.0),
                            getOptionalDouble(start_node, "y", 0.0));
      Eigen::Vector2d goal(getOptionalDouble(goal_node, "x", 0.0),
                           getOptionalDouble(goal_node, "y", 0.0));
      double start_yaw = getOptionalDouble(start_node, "yaw", 0.0);
      double start_v = getOptionalDouble(start_node, "v", 0.0);
      double start_vx = getOptionalDouble(start_node, "vx", 0.0);
      double start_vy = getOptionalDouble(start_node, "vy", 0.0);
      if (std::abs(start_v) < kEpsilon) {
        start_v = std::hypot(start_vx, start_vy);
      }
      if (start_v < 0.01) start_v = 0.1;

      ctx.state.initialize();
      ctx.state.set("x", start.x());
      ctx.state.set("y", start.y());
      ctx.state.set("psi", start_yaw);
      ctx.state.set("v", start_v);
      ctx.state.set("spline", 0.0);

      if (payload.contains("robot") && payload["robot"].is_object()) {
        const auto &robot = payload["robot"];
        ctx.robot_length = getOptionalDouble(robot, "length", ctx.robot_length);
        ctx.robot_width = getOptionalDouble(robot, "width", ctx.robot_width);
        ctx.data.robot_area =
            MPCPlanner::defineRobotArea(ctx.robot_length, ctx.robot_width, ctx.n_discs);
      }

      std::vector<Eigen::Vector3d> reference_line;
      reference_line.reserve(51);
      const int num_waypoints = 50;
      double ref_yaw = std::atan2(goal.y() - start.y(), goal.x() - start.x());
      for (int i = 0; i <= num_waypoints; ++i) {
        double t = static_cast<double>(i) / num_waypoints;
        reference_line.emplace_back(start.x() + t * (goal.x() - start.x()),
                                    start.y() + t * (goal.y() - start.y()),
                                    ref_yaw);
      }
      if (!convertReferencePath(reference_line, ctx.data.reference_path, ctx.reference_spline)) {
        response["type"] = "error";
        response["message"] = "Reference path creation failed";
        server.send(hdl, response.dump(), websocketpp::frame::opcode::text);
        return;
      }
      ctx.planner->onDataReceived(ctx.data, "reference_path");

      ctx.data.goal = goal;
      ctx.data.goal_received = true;

      std::vector<MPCPlanner::DynamicObstacle> temp_obstacles;
      json processed_obstacles = json::array();
      int payload_obstacles = 0;
      if (payload.contains("obstacles") && payload["obstacles"].is_array()) {
        int index = 0;
        for (const auto &obs : payload["obstacles"]) {
          if (index >= ctx.max_obstacles) break;
          const std::string shape = obs.value("shape", "circle");
          const double ox = getOptionalDouble(obs, "x", 0.0);
          const double oy = getOptionalDouble(obs, "y", 0.0);
          const double ovx = getOptionalDouble(obs, "vx", 0.0);
          const double ovy = getOptionalDouble(obs, "vy", 0.0);
          const Eigen::Vector2d position(ox, oy);
          const Eigen::Vector2d velocity(ovx, ovy);
          const bool is_static = obs.value("is_static", false);
          std::cout << "[DEBUG] Parsing obstacle: pos=(" << ox << "," << oy << ") vel=(" << ovx << "," << ovy << ") is_static=" << is_static << std::endl;
          double radius = getOptionalDouble(obs, "r", 0.2);
          if (shape == "rect") {
            const double w = getOptionalDouble(obs, "width", 0.4);
            const double h = getOptionalDouble(obs, "height", 0.4);
            radius = std::sqrt(w * w + h * h) / 2.0;
          }
          if (index >= ctx.max_obstacles) break;
          MPCPlanner::DynamicObstacle mpc_obs(index, position, 0.0, radius);
          mpc_obs.prediction =
              MPCPlanner::getConstantVelocityPrediction(position, velocity, ctx.integrator_step, ctx.horizon_steps);
          if (is_static) {
            mpc_obs.type = MPCPlanner::ObstacleType::STATIC;
          }
          temp_obstacles.push_back(mpc_obs);
          json item;
          item["x"] = position.x();
          item["y"] = position.y();
          item["r"] = radius;
          processed_obstacles.push_back(item);
          index++;
        }
        payload_obstacles = index;
      }

      // ============ DEBUG: 阶段计时开始 ============
      auto debug_t0 = std::chrono::steady_clock::now();
      
      // 打印真实障碍物信息
      std::cout << "\n========== TMPC DEBUG START ==========" << std::endl;
      std::cout << "[DEBUG] payload_obstacles = " << payload_obstacles << std::endl;
      for (size_t i = 0; i < temp_obstacles.size() && i < static_cast<size_t>(payload_obstacles); ++i) {
        const auto &obs = temp_obstacles[i];
        std::cout << "[DEBUG] Real obstacle[" << i << "]: "
                  << "pos=(" << obs.position.x() << "," << obs.position.y() << ") "
                  << "radius=" << obs.radius << " "
                  << "type=" << (obs.type == MPCPlanner::ObstacleType::DYNAMIC ? "DYNAMIC" : "STATIC")
                  << " prediction.modes[0].size=" << (obs.prediction.modes.empty() ? 0 : obs.prediction.modes[0].size())
                  << " prediction.type=" << static_cast<int>(obs.prediction.type)
                  << std::endl;
        // 打印前 3 个 prediction 位置
        if (!obs.prediction.modes.empty() && obs.prediction.modes[0].size() >= 3) {
          std::cout << "[DEBUG]   pred[0]=(" << obs.prediction.modes[0][0].position.x() << "," << obs.prediction.modes[0][0].position.y() << ")"
                    << " pred[1]=(" << obs.prediction.modes[0][1].position.x() << "," << obs.prediction.modes[0][1].position.y() << ")"
                    << " pred[2]=(" << obs.prediction.modes[0][2].position.x() << "," << obs.prediction.modes[0][2].position.y() << ")"
                    << std::endl;
        }
      }

      auto debug_t1 = std::chrono::steady_clock::now();
      MPCPlanner::ensureObstacleSize(temp_obstacles, ctx.state);
      auto debug_t2 = std::chrono::steady_clock::now();
      std::cout << "[DEBUG] ensureObstacleSize took: " 
                << std::chrono::duration<double, std::milli>(debug_t2 - debug_t1).count() << " ms" << std::endl;

      ctx.data.dynamic_obstacles = temp_obstacles;
      
      auto debug_t3 = std::chrono::steady_clock::now();
      ctx.planner->onDataReceived(ctx.data, "dynamic obstacles");
      auto debug_t4 = std::chrono::steady_clock::now();
      std::cout << "[DEBUG] planner->onDataReceived took: " 
                << std::chrono::duration<double, std::milli>(debug_t4 - debug_t3).count() << " ms" << std::endl;

      double spline_progress = computeReferenceProgress(ctx.state, ctx.reference_spline);
      ctx.state.set("spline", spline_progress);

      auto debug_t5 = std::chrono::steady_clock::now();
      updateGuidanceTrajectories(ctx.state, ctx);
      auto debug_t6 = std::chrono::steady_clock::now();
      std::cout << "[DEBUG] updateGuidanceTrajectories took: " 
                << std::chrono::duration<double, std::milli>(debug_t6 - debug_t5).count() << " ms" << std::endl;

      ctx.data.planning_start_time = std::chrono::system_clock::now();
      std::cout << "[DEBUG] Robot pos=(" << ctx.state.get("x") << "," << ctx.state.get("y") 
                << ") yaw=" << ctx.state.get("psi") << " v=" << ctx.state.get("v") << std::endl;
      std::cout << "[DEBUG] Total pre-solve time: " 
                << std::chrono::duration<double, std::milli>(debug_t6 - debug_t0).count() << " ms" << std::endl;

      json debug;
      debug["reference"] = buildReferenceDebug(ctx.data.reference_path);
      if (ctx.guidance && ctx.guidance->NumberOfGuidanceTrajectories() > 0) {
        debug["guidance"] = buildGuidanceDebug(*ctx.guidance, 40);
      } else {
        debug["guidance"] = json::array();
      }
      debug["obstacles"] = {
          {"payload", payload_obstacles},
          {"padded", static_cast<int>(ctx.data.dynamic_obstacles.size())},
          {"processed", processed_obstacles},
      };
      {
        json predictions = json::array();
        for (const auto &obs : temp_obstacles) {
          if (obs.prediction.modes.empty()) continue;
          json entry;
          entry["id"] = obs.index;
          json points = json::array();
          for (const auto &step : obs.prediction.modes.front()) {
            json p;
            p["x"] = step.position.x();
            p["y"] = step.position.y();
            points.push_back(p);
          }
          entry["points"] = points;
          predictions.push_back(entry);
        }
        debug["predictions"] = predictions;
      }
      {
        json debug_msg;
        debug_msg["type"] = "debug";
        debug_msg["debug"] = debug;
        server.send(hdl, debug_msg.dump(), websocketpp::frame::opcode::text);
      }
      if (payload.value("debugOnly", false)) {
        return;
      }

      ctx.data.past_trajectory.add(ctx.state.getPos());

      std::cout << "[DEBUG] Starting solveMPC..." << std::endl;
      auto debug_solve_start = std::chrono::steady_clock::now();
      MPCPlanner::PlannerOutput out = ctx.planner->solveMPC(ctx.state, ctx.data);
      auto debug_solve_end = std::chrono::steady_clock::now();
      std::cout << "[DEBUG] solveMPC took: " 
                << std::chrono::duration<double, std::milli>(debug_solve_end - debug_solve_start).count() << " ms"
                << ", success=" << out.success << std::endl;
      std::cout << "========== TMPC DEBUG END ==========" << std::endl;
      std::cout.flush();

      if (!out.success || out.trajectory.positions.empty()) {
        std::cout << "[ERROR] TMPC solve failed: success=" << out.success 
                  << " positions.size=" << out.trajectory.positions.size() << std::endl;
        response["type"] = "error";
        response["message"] = "TMPC solve failed";
        response["debug"] = debug;
        server.send(hdl, response.dump(), websocketpp::frame::opcode::text);
        return;
      }

      std::cout << "[DEBUG] Building trajectory response: positions.size=" << out.trajectory.positions.size() 
                << " dt=" << out.trajectory.dt << std::endl;
      std::cout.flush();

      auto build_start = std::chrono::steady_clock::now();
      json samples = json::array();
      for (size_t i = 0; i < out.trajectory.positions.size(); ++i) {
        const int k = static_cast<int>(i) + 1;
        json sample;
        sample["t"] = i * out.trajectory.dt;
        sample["x"] = out.trajectory.positions[i].x();
        sample["y"] = out.trajectory.positions[i].y();
        sample["yaw"] = ctx.planner->getSolution(k, "psi");
        sample["v"] = ctx.planner->getSolution(k, "v");
        sample["omega"] = ctx.planner->getSolution(k, "w");
        samples.push_back(sample);
      }

      response["type"] = "traj";
      response["samples"] = samples;
      response["dt"] = out.trajectory.dt;
      response["debug"] = debug;
      
      auto build_end = std::chrono::steady_clock::now();
      std::cout << "[DEBUG] Building response took: " 
                << std::chrono::duration<double, std::milli>(build_end - build_start).count() << " ms" << std::endl;
      std::cout.flush();

      auto send_start = std::chrono::steady_clock::now();
      server.send(hdl, response.dump(), websocketpp::frame::opcode::text);
      auto send_end = std::chrono::steady_clock::now();
      std::cout << "[DEBUG] WebSocket send took: " 
                << std::chrono::duration<double, std::milli>(send_end - send_start).count() << " ms" << std::endl;
      std::cout.flush();
    } catch (const std::exception &e) {
      response["type"] = "error";
      response["message"] = e.what();
      server.send(hdl, response.dump(), websocketpp::frame::opcode::text);
    }
  });

  server.listen(port);
  server.start_accept();
  std::cout << "TMPC server listening on ws://127.0.0.1:" << port << std::endl;
  std::cout << "Config: " << config_file << std::endl;
  server.run();
  return 0;
}
