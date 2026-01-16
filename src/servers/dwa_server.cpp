#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <nlohmann/json.hpp>

#include "dwa/dwa_planner.h"

using json = nlohmann::json;
using Server = websocketpp::server<websocketpp::config::asio>;

namespace {
struct PlannerParams {
  double v_max = 2.0;
  double v_min = -0.2;
  double w_max = 2.5;
  double accel = 1.0;
  double yaw_accel = 3.0;
  double v_reso = 0.2;
  double w_reso = 0.4;
  double dt = 0.1;
  double predict_time = 2.0;
  double w_goal = 1.0;
  double w_obs = 1.5;
  double w_speed = 1.0;
  double robot_radius = 0.35;
};

PlannerParams readParams(const json &payload) {
  PlannerParams p;
  if (!payload.contains("params")) return p;
  const auto &params = payload.at("params");
  p.v_max = params.value("vMax", p.v_max);
  p.v_min = params.value("vMin", p.v_min);
  p.w_max = params.value("wMax", p.w_max);
  p.accel = params.value("accel", p.accel);
  p.yaw_accel = params.value("yawAccel", p.yaw_accel);
  p.v_reso = params.value("vReso", p.v_reso);
  p.w_reso = params.value("wReso", p.w_reso);
  p.dt = params.value("dt", p.dt);
  p.predict_time = params.value("predictTime", p.predict_time);
  p.w_goal = params.value("wGoal", p.w_goal);
  p.w_obs = params.value("wObs", p.w_obs);
  p.w_speed = params.value("wSpeed", p.w_speed);
  p.robot_radius = params.value("robotRadius", p.robot_radius);
  return p;
}

std::vector<double> readObstacles(const json &payload) {
  std::vector<double> obs_flat;
  if (!payload.contains("obstacles")) return obs_flat;
  const auto &obstacles = payload.at("obstacles");
  if (!obstacles.is_array()) return obs_flat;

  obs_flat.reserve(obstacles.size() * 5);
  for (const auto &o : obstacles) {
    const double x = o.value("x", 0.0);
    const double y = o.value("y", 0.0);
    const double r = o.value("r", 0.0);
    const double vx = o.value("vx", 0.0);
    const double vy = o.value("vy", 0.0);
    obs_flat.push_back(x);
    obs_flat.push_back(y);
    obs_flat.push_back(r);
    obs_flat.push_back(vx);
    obs_flat.push_back(vy);
  }
  return obs_flat;
}
}  // namespace

int main(int argc, char **argv) {
  uint16_t port = 8081;
  if (argc > 1) {
    port = static_cast<uint16_t>(std::atoi(argv[1]));
  }

  DWAPlanner planner;
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
      const auto &state = payload.at("state");
      const auto &goal = payload.at("goal");
      const PlannerParams params = readParams(payload);

      planner.setLimits(params.v_max, params.v_min, params.w_max, params.accel, params.yaw_accel);
      planner.setResolution(params.v_reso, params.w_reso);
      planner.setPredict(params.dt, params.predict_time);
      planner.setWeights(params.w_goal, params.w_obs, params.w_speed);
      planner.setRobotRadius(params.robot_radius);

      const double x = state.value("x", 0.0);
      const double y = state.value("y", 0.0);
      const double yaw = state.value("yaw", 0.0);
      const double v = state.value("v", 0.0);
      const double w = state.value("w", 0.0);
      const double gx = goal.value("x", 0.0);
      const double gy = goal.value("y", 0.0);

      const std::vector<double> obs_flat = readObstacles(payload);
      const std::vector<double> cmd = planner.plan(x, y, yaw, v, w, gx, gy, obs_flat);

      json response;
      response["type"] = "cmd";
      response["v"] = cmd.size() > 0 ? cmd[0] : 0.0;
      response["w"] = cmd.size() > 1 ? cmd[1] : 0.0;
      response["bestCost"] = planner.bestCost();

      const bool want_best = payload.value("wantBest", false);
      if (want_best) {
        response["bestTrajectory"] = planner.bestTrajectory();
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
  std::cout << "DWA server listening on ws://127.0.0.1:" << port << std::endl;
  server.run();
  return 0;
}
