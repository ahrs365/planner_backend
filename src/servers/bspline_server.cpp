#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <nlohmann/json.hpp>

#include "bspline_planner/bspline_lattice_planner.h"

using json = nlohmann::json;
using Server = websocketpp::server<websocketpp::config::asio>;

namespace {
struct PlannerParams {
  double reference_interval = 0.2;
  double sample_length = 10.0;
  double ctp_interval_x = 0.6;
  double sample_half_width = 1.5;
  double ctp_interval_y = 0.5;
  double zero_layer_interval = 0.5;
  double bspline_interval = 0.02;
  double vehicle_length = 0.7;
  double vehicle_width = 0.7;
  double collision_margin = 0.05;
  size_t beam_width = 8;
  size_t max_candidate_paths = 12;
  double w_center = 1.0;
  double w_smooth = 1.5;
  double w_collision = 1000.0;
};

PlannerParams readParams(const json& payload) {
  PlannerParams p;
  if (!payload.contains("params")) return p;
  const auto& params = payload.at("params");
  p.reference_interval = params.value("referenceInterval", p.reference_interval);
  p.sample_length = params.value("sampleLength", p.sample_length);
  p.ctp_interval_x = params.value("ctpIntervalX", p.ctp_interval_x);
  p.sample_half_width = params.value("sampleHalfWidth", p.sample_half_width);
  p.ctp_interval_y = params.value("ctpIntervalY", p.ctp_interval_y);
  p.zero_layer_interval = params.value("zeroLayerInterval", p.zero_layer_interval);
  p.bspline_interval = params.value("bsplineInterval", p.bspline_interval);
  p.vehicle_length = params.value("vehicleLength", p.vehicle_length);
  p.vehicle_width = params.value("vehicleWidth", p.vehicle_width);
  p.collision_margin = params.value("collisionMargin", p.collision_margin);
  p.beam_width = params.value("beamWidth", p.beam_width);
  p.max_candidate_paths = params.value("maxCandidatePaths", p.max_candidate_paths);
  p.w_center = params.value("wCenter", p.w_center);
  p.w_smooth = params.value("wSmooth", p.w_smooth);
  p.w_collision = params.value("wCollision", p.w_collision);
  return p;
}

std::vector<ahrs::Obstacle> readObstacles(const json& payload) {
  std::vector<ahrs::Obstacle> obstacles;
  if (!payload.contains("obstacles")) return obstacles;
  const auto& obs = payload.at("obstacles");
  if (!obs.is_array()) return obstacles;

  obstacles.reserve(obs.size());
  for (const auto& o : obs) {
    const double x = o.value("x", 0.0);
    const double y = o.value("y", 0.0);
    const double r = o.value("r", 0.0);
    ahrs::Obstacle obstacle;
    obstacle.center = ahrs::Vec2d(x, y);
    obstacle.radius = r;
    obstacles.push_back(obstacle);
  }
  return obstacles;
}

json serializeTrajectory(const std::vector<ahrs::Point>& points) {
  json out = json::array();
  for (const auto& p : points) {
    out.push_back({
        {"x", p.pose_.x()},
        {"y", p.pose_.y()},
        {"theta", p.theta_},
        {"kappa", p.kappa_},
    });
  }
  return out;
}

json serializeControlPoints(const std::vector<ahrs::Point>& points) {
  json out = json::array();
  for (const auto& p : points) {
    out.push_back({
        {"x", p.pose_.x()},
        {"y", p.pose_.y()},
        {"theta", p.theta_},
    });
  }
  return out;
}

json serializeReferenceLine(const std::vector<ahrs::Point>& points) {
  json out = json::array();
  for (const auto& p : points) {
    out.push_back({
        {"x", p.pose_.x()},
        {"y", p.pose_.y()},
        {"theta", p.theta_},
        {"s", p.s_},
    });
  }
  return out;
}

json serializeTrajectories(const std::vector<std::vector<ahrs::Point>>& samples) {
  json out = json::array();
  for (const auto& traj : samples) {
    out.push_back(serializeTrajectory(traj));
  }
  return out;
}
}  // namespace

int main(int argc, char** argv) {
  uint16_t port = 8084;
  if (argc > 1) {
    port = static_cast<uint16_t>(std::atoi(argv[1]));
  }

  ahrs::BsplineLatticePlanner planner;
  Server server;

  server.init_asio();
  server.set_reuse_addr(true);
  server.clear_access_channels(websocketpp::log::alevel::all);
  server.clear_error_channels(websocketpp::log::elevel::all);

  server.set_message_handler([&](websocketpp::connection_hdl hdl,
                                 Server::message_ptr msg) {
    try {
      const auto payload = json::parse(msg->get_payload());
      if (payload.value("type", "") != "plan") {
        return;
      }

      const auto& state = payload.at("state");
      const auto& goal = payload.at("goal");

      ahrs::RobotState robot_state;
      robot_state.pose_ = ahrs::Vec2d(state.value("x", 0.0),
                                      state.value("y", 0.0));
      robot_state.theta_ = state.value("yaw", 0.0);

      const ahrs::Vec2d goal_pos(goal.value("x", 0.0),
                                 goal.value("y", 0.0));

      ahrs::Config config;
      const PlannerParams params = readParams(payload);
      config.reference_interval_ = params.reference_interval;
      config.sample_length_ = params.sample_length;
      config.ctp_interval_x_ = params.ctp_interval_x;
      config.sample_half_width_ = params.sample_half_width;
      config.ctp_interval_y_ = params.ctp_interval_y;
      config.zero_layer_interval_ = params.zero_layer_interval;
      config.bspline_interval_ = params.bspline_interval;
      config.vehicle_length_ = params.vehicle_length;
      config.vehicle_width_ = params.vehicle_width;
      config.collision_margin_ = params.collision_margin;
      config.beam_width_ = params.beam_width;
      config.max_candidate_paths_ = params.max_candidate_paths;
      config.w_center_ = params.w_center;
      config.w_smooth_ = params.w_smooth;
      config.w_collision_ = params.w_collision;

      ahrs::Environment env;
      env.obstacles_ = readObstacles(payload);

      ahrs::Curve trajectory;
      const bool ok = planner.Plan(robot_state, goal_pos, env, trajectory, config);

      json response;
      response["type"] = "trajectory";
      response["ok"] = ok;
      response["trajectory"] = serializeTrajectory(trajectory.points_);
      response["controlPoints"] = serializeControlPoints(trajectory.control_points_);

      const bool want_debug = payload.value("wantDebug", false);
      if (want_debug) {
        const auto& debug = planner.GetDebugInfo();
        json sampled_layers = json::array();
        for (const auto& layer : debug.sample_control_points_) {
          json layer_points = json::array();
          for (const auto& p : layer) {
            layer_points.push_back({{"x", p.x()}, {"y", p.y()}});
          }
          sampled_layers.push_back(layer_points);
        }
        response["sampledControlPoints"] = sampled_layers;
        response["referenceLine"] =
            serializeReferenceLine(debug.reference_line_points_);
        response["trajectories"] =
            serializeTrajectories(debug.bspline_samples_);
      }

      server.send(hdl, response.dump(), websocketpp::frame::opcode::text);
    } catch (const std::exception& e) {
      json err;
      err["type"] = "error";
      err["message"] = e.what();
      server.send(hdl, err.dump(), websocketpp::frame::opcode::text);
    }
  });

  server.listen(port);
  server.start_accept();
  std::cout << "Bspline server listening on ws://127.0.0.1:" << port
            << std::endl;
  server.run();
  return 0;
}
