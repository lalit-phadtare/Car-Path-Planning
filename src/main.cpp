#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  double ref_vel = 0.0;
  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
					tk::spline s;
					double ref_x = car_x;
					double ref_y = car_y;
					double ref_angle = deg2rad(car_yaw);
					int lane = static_cast <unsigned int> (std::floor(car_d / 4));
					if (lane > 2) { lane = 2; }
					//double ref_vel = car_speed/2.24;
					double diff_vel = 0.0;
					double max_vel = 22.0;// 49.5mph;
					double max_acc = 5.0;// 10ms2/2;
					double timestep = 0.02; //seconds -> 50ms
					int path_size = previous_path_x.size();
					vector<double> s_pts_x;
					vector<double> s_pts_y;

					bool car_front = false;
					bool car_left = false;
					bool car_right = false;


					if (path_size > 0) {
						car_s = end_path_s;
					}

					for (int i = 0; i < sensor_fusion.size(); i++) {
						float d = sensor_fusion[i][6];
						int sense_car_lane = -1;

						if ((d < 0.0) || (d > 12.0)) {
							continue;
						}
						//get sensed car lane no. 
						if (d >= 0.0 && d < 4.0) {
							sense_car_lane = 0;
						}
						else if (d >= 4.0 && d < 8.0) {
							sense_car_lane = 1;
						}
						else if (d >= 8.0 && d < 12.0) {
							sense_car_lane = 2;
						}
						if (sense_car_lane < 0) {
							continue;
						}

						//get sensed car information
						double vx = sensor_fusion[i][3];
						double vy = sensor_fusion[i][4];
						double check_speed = sqrt((vx * vx) + (vy * vy));
						double check_car_s = sensor_fusion[i][5];

						check_car_s += (double)path_size*timestep*(check_speed);

						//check if car is in front and collision possibility
						if (sense_car_lane == lane) {
							if ((check_car_s > car_s) && ((check_car_s - car_s) < 30.0)) {
								car_front = true;
							}
						}

						//check if car is in left lane
						else if (sense_car_lane - lane == -1) {
							if (fabs(check_car_s - car_s) < 30.0) {
								car_left = true;
							}
						}

						//check if car is right lane
						else if (sense_car_lane - lane == 1) {
							if (fabs(check_car_s - car_s) < 30.0) {
								car_right = true;
							}
						}
					}


					if (car_front) {
						if (!car_left && lane > 0) {
							lane -= 1;
							diff_vel -= (max_acc-2) * timestep;
						}
						else if (!car_right && lane < 2) {
							lane += 1;
							diff_vel -= (max_acc-2) * timestep;
						}
						else {
							diff_vel -= max_acc * timestep;
						}
					}
					else {
						//get to center lane if not
						if ((lane == 0 && !car_right) || (lane == 2 && !car_left)) {
							lane = 1;
						}

						//increase speed if below limit
						if (ref_vel < max_vel) {
							diff_vel += max_acc * timestep;
						}
					}



					if (path_size < 2)
					{
						double prev_car_x = car_x - cos(car_yaw);
						double prev_car_y = car_y - sin(car_yaw);


						s_pts_x.push_back(prev_car_x);
						s_pts_y.push_back(prev_car_y);
						s_pts_x.push_back(car_x);
						s_pts_y.push_back(car_y);

					}
					else
					{
						ref_x = previous_path_x[path_size - 1];
						ref_y = previous_path_y[path_size - 1];
                      	double ref_y_p = previous_path_y[path_size - 2];
                        double ref_x_p = previous_path_x[path_size - 2];
						ref_angle = atan2(ref_y - ref_y_p, ref_x - ref_x_p);
						s_pts_x.push_back(previous_path_x[path_size - 2]);
						s_pts_y.push_back(previous_path_y[path_size - 2]);
						s_pts_x.push_back(previous_path_x[path_size - 1]);
						s_pts_y.push_back(previous_path_y[path_size - 1]);
					}

					
										
					vector<double> s_pts1;
					vector<double> s_pts2;
					vector<double> s_pts3;
					s_pts1 = getXY(car_s + 30, (2 + (4 * lane)), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					s_pts_x.push_back(s_pts1[0]);
					s_pts_y.push_back(s_pts1[1]);
					s_pts2 = getXY(car_s + 60, (2 + (4 * lane)), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					s_pts_x.push_back(s_pts2[0]);
					s_pts_y.push_back(s_pts2[1]);
					s_pts3 = getXY(car_s + 90, (2 + (4 * lane)), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					s_pts_x.push_back(s_pts3[0]);
					s_pts_y.push_back(s_pts3[1]);

					for (int i = 0; i < s_pts_x.size(); i++) {
						double curr_x = s_pts_x[i] - ref_x;
						double curr_y = s_pts_y[i] - ref_y;
						s_pts_x[i] = (curr_x * cos(0 - ref_angle)) - (curr_y * sin(0 - ref_angle));
						s_pts_y[i] = (curr_x * sin(0 - ref_angle)) + (curr_y * cos(0 - ref_angle));
					}

					s.set_points(s_pts_x, s_pts_y);

					double target_x = 30.0;
					double target_y = s(target_x);
					double target_dist = sqrt((target_x*target_x) + (target_y*target_y));
					double x_add_on = 0;
					for (int i = 0; i < path_size; i++)
					{
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}
					for (int i = 1; i <= 50 - path_size; i++)
					{
						ref_vel += diff_vel;
						if (ref_vel > max_vel) {
							ref_vel = max_vel;
						}
						else if (ref_vel < max_acc * timestep) {
							ref_vel = max_acc * timestep;
						}
						double N = target_dist / (timestep * ref_vel);///mphtoms);
						double x_point = x_add_on + (target_x / N);
						//std::cout << " target_dist: " << target_dist << " ref vel: " << ref_vel << " N: " << N << std::endl;
						double y_point = s(x_point);
						x_add_on = x_point;

						double curr_x = x_point;
						double curr_y = y_point;

						x_point = (curr_x*cos(ref_angle)) - (curr_y * sin(ref_angle));
						y_point = (curr_x*sin(ref_angle)) + (curr_y * cos(ref_angle));
						
						x_point += ref_x;
						y_point += ref_y;
						next_x_vals.push_back(x_point);
						next_y_vals.push_back(y_point);
                    }


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}