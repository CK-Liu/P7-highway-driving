#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "json.hpp"
#include "helpers.h"

#include "road.h"
#include "vehicle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;
//using std::endl;

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
  
  int lane = 1;
  double ref_vel = 0;
  
  h.onMessage([&lane,&ref_vel,&max_s,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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
          Vehicle ego; 
          ego = Vehicle(car_x,car_y,car_s,car_d,car_yaw,car_speed); //self car
          ego.id = -1;
          ego.lane = lane;
          ego.ref_vel = ref_vel;         
                              
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          
          Path_Points points = {previous_path_x, previous_path_y}; //store previous not executed path points
          std::cout << "previous path size x = " << points.xs.size() << std::endl;
          std::cout << "previous path size y = " << points.ys.size() << std::endl;

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          std::cout << "sensor fusion size = " << sensor_fusion.size() << std::endl;
          
          int prev_size = points.xs.size();
          
          if (prev_size  > 0) {
           	ego.s = end_path_s; 
          }
                      
          Road road;
          Vehicle vehicle;
          
          for (int i=0; i<sensor_fusion.size(); i++) {
            //  [ id, x, y, vx, vy, s, d]
            vehicle = Vehicle(sensor_fusion[i][0],sensor_fusion[i][1],sensor_fusion[i][2],sensor_fusion[i][3],
                                      sensor_fusion[i][4],sensor_fusion[i][5],sensor_fusion[i][6]);
            road.vehicles.insert(std::pair<int,Vehicle>(vehicle.id, vehicle));    
            std::cout << "id: " << road.vehicles[vehicle.id].id;
          }          
          std::cout << std::endl << "number of vehicles on road: " << road.vehicles.size() << endl;
         // road.vehicles.insert(std::pair<int,Vehicle>(ego.id,ego));  //push self car into all vehicles
          
//           int timestep = 0;
//           while (ego.s <= max_s) {
//             ++timestep;
//             road.advance();
//           }
          std::cout << "Previous lane = " << ego.lane << " ref speed = " << ego.ref_vel << " id = " << ego.id << std::endl;
          ego.state_test(road.vehicles, prev_size, lane, ref_vel); //modify lane and reference speed
          std::cout << "lane = " << ego.lane << " ref speed = " << ego.ref_vel << " id = " << ego.id << std::endl;
          
          json msgJson;
          vector<double> next_x_vals;
          vector<double> next_y_vals;
                    
          ego.generate_path_points(points, end_path_s, end_path_d,
                                  map_waypoints_s,map_waypoints_x,map_waypoints_y);  //pass reference 'points'
          next_x_vals = points.xs;
          next_y_vals = points.ys;
          
          std::cout << "Final points size: " << points.xs.size() << std::endl;

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