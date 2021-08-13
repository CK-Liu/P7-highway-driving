#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>



using std::map;
using std::string;
using std::vector;

struct Path_Points {
  std::vector<double> xs;
  std::vector<double> ys;
};

class Vehicle {
 public:
  // Constructors
  Vehicle();
 // Vehicle(float x, float y, float s, float d, float yaw, float speed);
  Vehicle(double x, double y, double s, double d, double yaw, double speed);
 // Vehicle(int id, float x, float y, float vx, float vy, float s, float d);
  Vehicle(int id, double x, double y, double vx, double vy, double s, double d);

  // Destructor
  virtual ~Vehicle();

  // Vehicle functions
  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> &predictions);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state,
                                      map<int, vector<Vehicle>> &predictions);

  vector<float> get_kinematics(map<int, vector<Vehicle>> &predictions, int lane);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> &predictions);

  vector<Vehicle> lane_change_trajectory(string state,
                                         map<int, vector<Vehicle>> &predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state,
                                              map<int, vector<Vehicle>> &predictions);

  void increment(int dt);

  float position_at(int t);

  bool get_vehicle_behind(map<int, vector<Vehicle>> &predictions, int lane,
                          Vehicle &rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, int lane,
                         Vehicle &rVehicle);

  vector<Vehicle> generate_predictions(int horizon=2);

  void realize_next_state(vector<Vehicle> &trajectory);

  void configure(vector<int> &road_data);
  
  void generate_path_points(Path_Points &points, double end_path_s, double end_path_d,
                                  	const vector<double> &maps_s, 
                    			 	const vector<double> &maps_x, 
                     				const vector<double> &maps_y);
  
  //void state_test (std::map<int, Vehicle> vehicles, int prev_size);
  void state_test (std::map<int, Vehicle> vehicles, int prev_size, int &lane, double &ref_vel);

  // public Vehicle variables
  struct collider{
    bool collision; // is there a collision?
    int  time; // time collision happens
  };

  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1},
                                     {"LCR", -1}, {"PLCR", -1}};

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int id, lane, goal_lane, goal_s, lanes_available;

  double x, y, vx, vy, s, d, yaw, speed, ref_vel, v, a, max_acceleration;
  
  float target_speed;

  string state;
};

#endif  // VEHICLE_H
