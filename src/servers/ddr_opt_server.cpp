#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include "ddr-opt/esdf/esdf_map.hpp"
#include "ddr-opt/path/jps_planner.hpp"
#include "ddr-opt/path/jps_data_structures.hpp"
#include "ddr-opt/opt/optimizer.h"

using json = nlohmann::json;
using Server = websocketpp::server<websocketpp::config::asio>;

namespace {
constexpr double kEpsilon = 1e-6;

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

template <typename T>
void readYamlScalar(const YAML::Node &node, const char *key, T &out) {
  if (node && node[key]) {
    out = node[key].as<T>();
  }
}

template <typename T>
void readYamlVector(const YAML::Node &node, const char *key, std::vector<T> &out) {
  if (node && node[key] && node[key].IsSequence()) {
    out.clear();
    for (const auto &item : node[key]) {
      out.push_back(item.as<T>());
    }
  }
}

JPS::JPSConfig loadPlannerConfig(const std::string &path) {
  JPS::JPSConfig config;
  YAML::Node root = YAML::LoadFile(path);

  YAML::Node path_lbfgs = root["path_lbfgs_params"];
  readYamlScalar(path_lbfgs, "mem_size", config.optimizer.path_lbfgs_mem_size);
  readYamlScalar(path_lbfgs, "past", config.optimizer.path_lbfgs_past);
  readYamlScalar(path_lbfgs, "g_epsilon", config.optimizer.path_lbfgs_g_epsilon);
  readYamlScalar(path_lbfgs, "min_step", config.optimizer.path_lbfgs_min_step);
  readYamlScalar(path_lbfgs, "delta", config.optimizer.path_lbfgs_delta);
  readYamlScalar(path_lbfgs, "max_iterations", config.optimizer.path_lbfgs_max_iterations);
  readYamlScalar(path_lbfgs, "shot_path_past", config.optimizer.path_lbfgs_shot_path_past);
  readYamlScalar(path_lbfgs, "shot_path_horizon", config.optimizer.path_lbfgs_shot_path_horizon);

  YAML::Node path_penalty = root["PathpenaltyWeights"];
  readYamlScalar(path_penalty, "time_weight", config.optimizer.path_time_weight);
  readYamlScalar(path_penalty, "bigpath_sdf_weight", config.optimizer.path_bigpath_sdf_weight);
  readYamlScalar(path_penalty, "mean_time_weight", config.optimizer.path_mean_time_weight);
  readYamlScalar(path_penalty, "moment_weight", config.optimizer.path_moment_weight);
  readYamlScalar(path_penalty, "acc_weight", config.optimizer.path_acc_weight);
  readYamlScalar(path_penalty, "domega_weight", config.optimizer.path_domega_weight);

  readYamlVector(root, "energyWeights", config.optimizer.energyWeights);

  YAML::Node lbfgs = root["lbfgs_params"];
  readYamlScalar(lbfgs, "mem_size", config.optimizer.lbfgs_mem_size);
  readYamlScalar(lbfgs, "past", config.optimizer.lbfgs_past);
  readYamlScalar(lbfgs, "g_epsilon", config.optimizer.lbfgs_g_epsilon);
  readYamlScalar(lbfgs, "min_step", config.optimizer.lbfgs_min_step);
  readYamlScalar(lbfgs, "delta", config.optimizer.lbfgs_delta);
  readYamlScalar(lbfgs, "max_iterations", config.optimizer.lbfgs_max_iterations);

  readYamlScalar(root, "mean_time_lowBound", config.optimizer.mean_time_lowBound);
  readYamlScalar(root, "mean_time_uppBound", config.optimizer.mean_time_uppBound);

  YAML::Node penalty = root["penaltyWeights"];
  readYamlScalar(penalty, "time_weight", config.optimizer.time_weight);
  readYamlScalar(penalty, "acc_weight", config.optimizer.acc_weight);
  readYamlScalar(penalty, "domega_weight", config.optimizer.domega_weight);
  readYamlScalar(penalty, "collision_weight", config.optimizer.collision_weight);
  readYamlScalar(penalty, "moment_weight", config.optimizer.moment_weight);
  readYamlScalar(penalty, "mean_time_weight", config.optimizer.mean_time_weight);
  readYamlScalar(penalty, "cen_acc_weight", config.optimizer.cen_acc_weight);

  readYamlVector(root, "EqualLambda", config.optimizer.EqualLambda);
  readYamlVector(root, "EqualRho", config.optimizer.EqualRho);
  readYamlVector(root, "EqualRhoMax", config.optimizer.EqualRhoMax);
  readYamlVector(root, "EqualGamma", config.optimizer.EqualGamma);
  readYamlVector(root, "EqualTolerance", config.optimizer.EqualTolerance);

  readYamlVector(root, "CutEqualLambda", config.optimizer.CutEqualLambda);
  readYamlVector(root, "CutEqualRho", config.optimizer.CutEqualRho);
  readYamlVector(root, "CutEqualRhoMax", config.optimizer.CutEqualRhoMax);
  readYamlVector(root, "CutEqualGamma", config.optimizer.CutEqualGamma);
  readYamlVector(root, "CutEqualTolerance", config.optimizer.CutEqualTolerance);

  readYamlScalar(root, "sparseResolution", config.optimizer.sparseResolution);
  readYamlScalar(root, "safeDis", config.optimizer.safeDis);
  readYamlScalar(root, "finalMinSafeDis", config.optimizer.finalMinSafeDis);
  readYamlScalar(root, "finalSafeDisCheckNum", config.optimizer.finalSafeDisCheckNum);
  readYamlScalar(root, "safeReplanMaxTime", config.optimizer.safeReplanMaxTime);
  readYamlScalar(root, "timeResolution", config.optimizer.timeResolution);
  readYamlScalar(root, "mintrajNum", config.optimizer.mintrajNum);
  readYamlScalar(root, "trajPredictResolution", config.optimizer.trajPredictResolution);
  readYamlScalar(root, "if_visual_optimization", config.optimizer.if_visual_optimization);

  readYamlScalar(root, "trajCutLength", config.traj_cut_length);

  config.safe_dis = config.optimizer.safeDis;
  config.sample_time = config.optimizer.timeResolution;
  config.min_traj_num = config.optimizer.mintrajNum;

  return config;
}

json serializeMincoTrajectorySamples(const Trajectory<5, 2> &traj,
                                     const JPS::FlatTrajData &flat_traj, double dt) {
  json out = json::array();
  if (flat_traj.UnOccupied_positions.empty()) {
    return out;
  }

  const double total = traj.getTotalDuration();
  double t = 0.0;
  double last_s = 0.0;
  Eigen::Vector2d pos_xy(flat_traj.UnOccupied_positions.front().x(),
                         flat_traj.UnOccupied_positions.front().y());

  Eigen::Vector2d pos_ys = traj.getPos(0.0);
  last_s = pos_ys.size() > 1 ? pos_ys[1] : 0.0;

  while (t <= total + 1e-9) {
    Eigen::Vector2d pos = traj.getPos(t);  // [yaw, s]
    Eigen::Vector2d vel = traj.getVel(t);  // [dyaw/dt, ds/dt]
    Eigen::Vector2d acc = traj.getAcc(t);  // [d2yaw/dt2, d2s/dt2]

    const double yaw = pos.size() > 0 ? pos[0] : 0.0;
    const double s = pos.size() > 1 ? pos[1] : last_s;
    const double ds = s - last_s;
    if (t > 0.0) {
      pos_xy.x() += ds * std::cos(yaw);
      pos_xy.y() += ds * std::sin(yaw);
    }
    last_s = s;

    json sample;
    sample["t"] = t;
    sample["x"] = pos_xy.x();
    sample["y"] = pos_xy.y();
    sample["yaw"] = yaw;
    const double v = vel.size() > 1 ? vel[1] : 0.0;
    sample["vx"] = v;
    sample["vy"] = 0.0;
    sample["ax"] = acc.size() > 1 ? acc[1] : 0.0;
    sample["ay"] = 0.0;
    out.push_back(sample);

    t += dt;
  }
  return out;
}

json serializeTrajectoryPoly(const Trajectory<5, 2> &traj) {
  json pieces = json::array();
  for (int i = 0; i < traj.getPieceNum(); ++i) {
    const auto &piece = traj[i];
    const auto &mat = piece.getCoeffMat();
    json coeffs = json::array();
    for (int r = 0; r < mat.rows(); ++r) {
      json row = json::array();
      for (int c = 0; c < mat.cols(); ++c) {
        row.push_back(mat(r, c));
      }
      coeffs.push_back(row);
    }
    json entry;
    entry["duration"] = piece.getDuration();
    entry["coeffs"] = coeffs;
    pieces.push_back(entry);
  }

  json out;
  out["degree"] = 5;
  out["dim"] = 2;
  out["pieces"] = pieces;
  return out;
}

json serializeEsdfGrid(const navsim::perception::ESDFMap &map,
                       const navsim::perception::ESDFMap::Config &config,
                       const Eigen::Vector2d &origin,
                       int occupied_cells) {
  const int width = map.GLX_SIZE_;
  const int height = map.GLY_SIZE_;
  json data = json::array();
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      data.push_back(map.getDistance(x, y));
    }
  }
  json meta;
  meta["width"] = width;
  meta["height"] = height;
  meta["resolution"] = config.resolution;
  meta["origin"] = {{"x", origin.x()}, {"y", origin.y()}};
  meta["maxDistance"] = config.max_distance;
  meta["occupiedCells"] = occupied_cells;
  meta["totalCells"] = width * height;
  meta["occupiedRatio"] =
      width * height > 0 ? static_cast<double>(occupied_cells) / (width * height) : 0.0;
  json out;
  out["data"] = data;
  out["meta"] = meta;
  return out;
}

}  // namespace

int main(int argc, char **argv) {
  uint16_t port = 8082;
  std::string config_path = "src/ddr-opt/opt/global_planning3ms.yaml";
  if (argc > 1) {
    port = static_cast<uint16_t>(std::atoi(argv[1]));
  }
  if (argc > 2) {
    config_path = argv[2];
  }

  JPS::JPSConfig base_config;
  try {
    base_config = loadPlannerConfig(config_path);
  } catch (const std::exception &e) {
    std::cerr << "Failed to load config: " << e.what() << std::endl;
    return 1;
  }

  Server server;
  server.init_asio();
  server.set_reuse_addr(true);
  server.clear_access_channels(websocketpp::log::alevel::all);
  server.clear_error_channels(websocketpp::log::elevel::all);

  server.set_message_handler([&](websocketpp::connection_hdl hdl, Server::message_ptr msg) {
    try {
      const auto payload = json::parse(msg->get_payload());
      if (payload.value("type", "") != "plan") {
        return;
      }

      const auto &map_node = payload.at("map");
      const auto &origin_node = map_node.at("origin");
      const double resolution = getOptionalDouble(map_node, "resolution", 0.1);
      const double width = getOptionalDouble(map_node, "width", 10.0);
      const double height = getOptionalDouble(map_node, "height", 10.0);
      const double max_distance = getOptionalDouble(map_node, "maxDistance",
                                                    std::max(width, height));
      const double origin_x = getOptionalDouble(origin_node, "x", 0.0);
      const double origin_y = getOptionalDouble(origin_node, "y", 0.0);

      std::vector<uint8_t> grid;
      int occupied_cells = 0;
      if (map_node.contains("data") && map_node["data"].is_array()) {
        const auto &data = map_node["data"];
        grid.reserve(data.size());
        for (const auto &cell : data) {
          if (cell.is_number_integer()) {
            const int value = cell.get<int>();
            grid.push_back(static_cast<uint8_t>(value));
            if (value > 50) occupied_cells++;
          } else if (cell.is_number()) {
            const double value = cell.get<double>();
            grid.push_back(static_cast<uint8_t>(value));
            if (value > 50.0) occupied_cells++;
          } else {
            grid.push_back(0);
          }
        }
      }

      navsim::perception::ESDFMap::Config map_config;
      map_config.resolution = resolution;
      map_config.map_width = width;
      map_config.map_height = height;
      map_config.max_distance = max_distance;

      auto map = std::make_shared<navsim::perception::ESDFMap>();
      map->initialize(map_config);
      map->buildFromOccupancyGrid(grid, Eigen::Vector2d(origin_x, origin_y));
      map->computeESDF();
      double esdf_min = std::numeric_limits<double>::infinity();
      double esdf_max = -std::numeric_limits<double>::infinity();
      double esdf_sum = 0.0;
      int esdf_finite = 0;
      int esdf_nan = 0;
      for (int y = 0; y < map->GLY_SIZE_; y++) {
        for (int x = 0; x < map->GLX_SIZE_; x++) {
          const double v = map->getDistance(x, y);
          if (std::isfinite(v)) {
            esdf_min = std::min(esdf_min, v);
            esdf_max = std::max(esdf_max, v);
            esdf_sum += v;
            esdf_finite++;
          } else {
            esdf_nan++;
          }
        }
      }
      const double esdf_mean = esdf_finite > 0 ? esdf_sum / esdf_finite : 0.0;

      JPS::JPSConfig config = base_config;
      if (payload.contains("limits")) {
        const auto &limits = payload["limits"];
        config.max_vel = getOptionalDouble(limits, "maxVel", config.max_vel);
        config.max_acc = getOptionalDouble(limits, "maxAcc", config.max_acc);
        config.max_omega = getOptionalDouble(limits, "maxOmega", config.max_omega);
        config.max_domega = getOptionalDouble(limits, "maxDomega", config.max_domega);
      }

      const auto &start_node = payload.at("start");
      const auto &goal_node = payload.at("goal");
      Eigen::Vector3d start(getOptionalDouble(start_node, "x", 0.0),
                            getOptionalDouble(start_node, "y", 0.0),
                            getOptionalDouble(start_node, "yaw", 0.0));
      Eigen::Vector3d goal(getOptionalDouble(goal_node, "x", 0.0),
                           getOptionalDouble(goal_node, "y", 0.0),
                           getOptionalDouble(goal_node, "yaw", 0.0));

      Eigen::Vector3d vaj(0.0, 0.0, 0.0);
      Eigen::Vector3d oaj(0.0, 0.0, 0.0);
      if (payload.contains("current")) {
        const auto &cur = payload["current"];
        vaj[0] = getOptionalDouble(cur, "v", 0.0);
        vaj[1] = getOptionalDouble(cur, "a", 0.0);
        vaj[2] = getOptionalDouble(cur, "j", 0.0);
        oaj[0] = getOptionalDouble(cur, "omega", 0.0);
        oaj[1] = getOptionalDouble(cur, "alpha", 0.0);
        oaj[2] = getOptionalDouble(cur, "jerk", 0.0);
      }

      JPS::JPSPlanner planner(map);
      planner.setConfig(config);
      planner.setCurrentVelocityState(vaj, oaj);
      if (!planner.plan(start, goal)) {
        json err;
        err["type"] = "error";
        err["message"] = "JPS planning failed";
        server.send(hdl, err.dump(), websocketpp::frame::opcode::text);
        return;
      }

      JPS::MSPlanner optimizer(config.optimizer, map);
      if (!optimizer.minco_plan(planner.getFlatTraj())) {
        json err;
        err["type"] = "error";
        err["message"] = "Optimization failed";
        server.send(hdl, err.dump(), websocketpp::frame::opcode::text);
        return;
      }

      double sample_dt = payload.value("sampleDt", config.optimizer.trajPredictResolution);
      if (sample_dt <= 0.0) sample_dt = 0.05;

      json response;
      response["type"] = "traj";
      json jps_path = json::array();
      const auto &raw_path = planner.getRawPath();
      for (const auto &pt : raw_path) {
        json p;
        p["x"] = pt.x();
        p["y"] = pt.y();
        jps_path.push_back(p);
      }
      response["jpsPath"] = jps_path;
      response["samples"] =
          serializeMincoTrajectorySamples(optimizer.final_traj_, planner.getFlatTraj(), sample_dt);
      response["poly"] = serializeTrajectoryPoly(optimizer.final_traj_);
      if (payload.value("wantEsdf", false)) {
        response["esdf"] = serializeEsdfGrid(
            *map,
            map_config,
            Eigen::Vector2d(origin_x, origin_y),
            occupied_cells);
        response["esdf"]["meta"]["minValue"] = esdf_min;
        response["esdf"]["meta"]["maxValue"] = esdf_max;
        response["esdf"]["meta"]["meanValue"] = esdf_mean;
      }
      server.send(hdl, response.dump(), websocketpp::frame::opcode::text);
    } catch (const std::exception &e) {
      json err;
      err["type"] = "error";
      err["message"] = e.what();
      server.send(hdl, err.dump(), websocketpp::frame::opcode::text);
    }
  });

  server.listen(port);
  server.start_accept();
  std::cout << "DDR-OPT server listening on ws://127.0.0.1:" << port << std::endl;
  std::cout << "Config: " << config_path << std::endl;
  server.run();
  return 0;
}
