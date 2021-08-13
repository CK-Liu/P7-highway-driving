#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <iostream>
#include "cost.h"
#include "spline.h"

#include <map>
#include <string>
#include <vector>

#include "helpers.h"

using std::string;
using std::vector;

int SPEED_LIMIT = 49.5;
int MAX_ACCEL = 2;

// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int id, double x, double y, double vx, double vy, double s, double d) {
  this->id = id;
  this->x = x;
  this->y = y;
  this->vx = vx;
  this->vy = vy;
  this->s = s;
  this->d = d;
}

//initialize ego
Vehicle::Vehicle(double x, double y, double s, double d, double yaw, double speed) {
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
  this->speed = speed;
 // max_acceleration = -1;
}

Vehicle::~Vehicle() {}

void Vehicle::state_test (std::map<int, Vehicle> vehicles, int prev_size, int &lane, double &ref_vel) {  
  bool too_close_ahead = false;
  bool LCR = (lane < 2);
  bool LCL = (lane > 0);
  for (map<int, Vehicle>::iterator it = vehicles.begin();
       it != vehicles.end(); ++it) {
    	float d = it->second.d;
        int car_lane = floor(d/4.0);
        double vx = it->second.vx;
        double vy = it->second.vy;
        double check_speed = sqrt(vx*vx+vy*vy);
        double check_car_s = it->second.s;
    	double car_s = this->s;

        check_car_s += ((double)prev_size*0.02*check_speed);
        bool car_ahead = (check_car_s > car_s);
        bool too_close = (((check_car_s-car_s)<30) and car_ahead) or (check_car_s-car_s<-10);
        // bool too_close = ((check_car_s-car_s)<30) and car_ahead;

        if (too_close) {
          if (car_lane == lane && car_ahead) {
            too_close_ahead = true;
          } else if (car_lane == lane-1) {
            LCL = false;
          } else if (car_lane == lane+1) {
            LCR = false;
          }
        }  
  }
  
  if (too_close_ahead) {
    ref_vel -= 0.224;
    if (LCL == true) {
      lane -= 1;
      //cout << "Changing to left lane" << std::endl;
    } else if (LCR == true) {
      lane += 1;
    }
  } else if (ref_vel < 49.5) {
    ref_vel += 0.224;
  }
  this->lane = lane;
  this->ref_vel = ref_vel;
}

// lane and ref_vel need to be recalculated
void Vehicle::generate_path_points(Path_Points &points, double end_path_s, double end_path_d,
                                  	const vector<double> &maps_s, 
                    			 	const vector<double> &maps_x, 
                     				const vector<double> &maps_y) {
  int prev_size = points.xs.size();
  double car_s = this->s;
  double car_x = this->x;
  double car_y = this->y;
  double car_yaw = this->yaw;
  double ref_vel = this->ref_vel;
  std::vector<double> previous_path_x = points.xs;
  std::vector<double> previous_path_y = points.ys;
//   if (prev_size  > 0) {
//     car_s = end_path_s; 
//   }
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  if (prev_size < 2) {
    double prev_car_x = car_x - cos(car_yaw); 
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y); 
  } else {
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];

    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  vector<double> next_wp0 = getXY(car_s+30,(2+4*(this->lane)),maps_s,maps_x,maps_y);
  vector<double> next_wp1 = getXY(car_s+60,(2+4*(this->lane)),maps_s,maps_x,maps_y);
  vector<double> next_wp2 = getXY(car_s+90,(2+4*(this->lane)),maps_s,maps_x,maps_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for (int i=0; i<ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
    ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
  }

  tk::spline s;
  s.set_points(ptsx, ptsy);

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);
  double x_add_on = 0;

  for (int i=1; i<=50-previous_path_x.size(); i++) {
    double N = target_dist/(0.02*ref_vel/2.24); //mph to m/s divide 2.24
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);
    x_add_on = x_point;
    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
    y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;  

    points.xs.push_back(x_point);
    points.ys.push_back(y_point);
  }
}

vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> &predictions) {
  /**
   * Here you can implement the transition_function code from the Behavior
   *   Planning Pseudocode classroom concept.
   *
   * @param A predictions map. This is a map of vehicle id keys with predicted
   *   vehicle trajectories as values. Trajectories are a vector of Vehicle
   *   objects representing the vehicle at the current timestep and one timestep
   *   in the future.
   * @output The best (lowest cost) trajectory corresponding to the next ego
   *   vehicle state.
   *
   * Functions that will be useful:
   * 1. successor_states - Uses the current state to return a vector of possible
   *    successor states for the finite state machine.
   * 2. generate_trajectory - Returns a vector of Vehicle objects representing
   *    a vehicle trajectory, given a state and predictions. Note that
   *    trajectory vectors might have size 0 if no possible trajectory exists
   *    for the state.
   * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.
   *
   * TODO: Your solution here.
   */
  vector<string> states = successor_states();
  float cost;
  vector<float> costs;
  vector<vector<Vehicle>> final_trajectories;

  for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
    vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
    if (trajectory.size() != 0) {
      cost = calculate_cost(*this, predictions, trajectory);
      costs.push_back(cost);
      final_trajectories.push_back(trajectory);
    }
  }

  vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);

  /**
   * TODO: Change return value here:
   */
  return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states() {
  // Provides the possible next states given the current state for the FSM
  //   discussed in the course, with the exception that lane changes happen
  //   instantaneously, so LCL and LCR can only transition back to KL.
  vector<string> states;
  states.push_back("KL");
  string state = this->state;
  if(state.compare("KL") == 0) {
    states.push_back("PLCL");
    states.push_back("PLCR");
  } else if (state.compare("PLCL") == 0) {
    if (lane != lanes_available - 1) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } else if (state.compare("PLCR") == 0) {
    if (lane != 0) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }

  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state,
                                             map<int, vector<Vehicle>> &predictions) {
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
  vector<Vehicle> trajectory;
  if (state.compare("CS") == 0) {
    trajectory = constant_speed_trajectory();
  } else if (state.compare("KL") == 0) {
    trajectory = keep_lane_trajectory(predictions);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = lane_change_trajectory(state, predictions);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    trajectory = prep_lane_change_trajectory(state, predictions);
  }

  return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> &predictions,
                                      int lane) {
  // Gets next timestep kinematics (position, velocity, acceleration)
  //   for a given lane. Tries to choose the maximum velocity and acceleration,
  //   given other vehicle positions and accel/velocity constraints.
  float max_velocity_accel_limit = this->max_acceleration + this->v;
  float new_position;
  float new_velocity;
  float new_accel;
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
    if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
      // must travel at the speed of traffic, regardless of preferred buffer
      new_velocity = vehicle_ahead.v;
    } else {
      float max_velocity_in_front = (vehicle_ahead.s - this->s
                                  - this->preferred_buffer) + vehicle_ahead.v
                                  - 0.5 * (this->a);
      new_velocity = std::min(std::min(max_velocity_in_front,
                                       max_velocity_accel_limit),
                                       this->target_speed);
    }
  } else {
    new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
  }

  new_accel = new_velocity - this->v; // Equation: (v_1 - v_0)/t = acceleration
  new_position = this->s + new_velocity + new_accel / 2.0;

  return{new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
  // Generate a constant speed trajectory.
  float next_pos = position_at(1);
  vector<Vehicle> trajectory = {Vehicle(),
                                Vehicle()};
  return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> &predictions) {
  // Generate a keep lane trajectory.
  //vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->v, this->a, state)};
  vector<Vehicle> trajectory = {Vehicle()};
  vector<float> kinematics = get_kinematics(predictions, this->lane);
  float new_s = kinematics[0];
  float new_v = kinematics[1];
  float new_a = kinematics[2];
  trajectory.push_back(Vehicle());

  return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state,
                                                     map<int, vector<Vehicle>> &predictions) {
  // Generate a trajectory preparing for a lane change.
  float new_s;
  float new_v;
  float new_a;
  Vehicle vehicle_behind;
  int new_lane = this->lane + lane_direction[state];
//   vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a,
//                                         this->state)};
  vector<Vehicle> trajectory = {Vehicle()};
  vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

  if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
    // Keep speed of current lane so as not to collide with car behind.
    new_s = curr_lane_new_kinematics[0];
    new_v = curr_lane_new_kinematics[1];
    new_a = curr_lane_new_kinematics[2];
  } else {
    vector<float> best_kinematics;
    vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
    // Choose kinematics with lowest velocity.
    if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
      best_kinematics = next_lane_new_kinematics;
    } else {
      best_kinematics = curr_lane_new_kinematics;
    }
    new_s = best_kinematics[0];
    new_v = best_kinematics[1];
    new_a = best_kinematics[2];
  }

 // trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));

  return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state,
                                                map<int, vector<Vehicle>> &predictions) {
  // Generate a lane change trajectory.
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory;
  Vehicle next_lane_vehicle;
  // Check if a lane change is possible (check if another vehicle occupies
  //   that spot).
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
       it != predictions.end(); ++it) {
    next_lane_vehicle = it->second[0];
    if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
      // If lane change is not possible, return empty trajectory.
      return trajectory;
    }
  }
//   trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a,
//                                this->state));
  vector<float> kinematics = get_kinematics(predictions, new_lane);
//   trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1],
//                                kinematics[2], state));
  return trajectory;
}

void Vehicle::increment(int dt = 1) {
  this->s = position_at(dt);
}

float Vehicle::position_at(int t) {
  return this->s + this->v*t + this->a*t*t/2.0;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> &predictions,
                                 int lane, Vehicle &rVehicle) {
  // Returns a true if a vehicle is found behind the current vehicle, false
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  int max_s = -1;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
       it != predictions.end(); ++it) {
    temp_vehicle = it->second[0];
    if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s
        && temp_vehicle.s > max_s) {
      max_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }

  return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> &predictions,
                                int lane, Vehicle &rVehicle) {
  // Returns a true if a vehicle is found ahead of the current vehicle, false
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  int min_s = this->goal_s;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
       it != predictions.end(); ++it) {
    temp_vehicle = it->second[0];
    if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s
        && temp_vehicle.s < min_s) {
      min_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }

  return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
  // Generates predictions for non-ego vehicles to be used in trajectory
  //   generation for the ego vehicle.
  vector<Vehicle> predictions;
  for(int i = 0; i < horizon; ++i) {
    float next_s = position_at(i);
    float next_v = 0;
    if (i < horizon-1) {
      next_v = position_at(i+1) - s;
    }
  //  predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
  }

  return predictions;
}

void Vehicle::realize_next_state(vector<Vehicle> &trajectory) {
  // Sets state and kinematics for ego vehicle using the last state of the trajectory.
  Vehicle next_state = trajectory[1];
  this->state = next_state.state;
  this->lane = next_state.lane;
  this->s = next_state.s;
  this->v = next_state.v;
  this->a = next_state.a;
}

void Vehicle::configure(vector<int> &road_data) {
  // Called by simulator before simulation begins. Sets various parameters which
  //   will impact the ego vehicle.
  target_speed = road_data[0];
  lanes_available = road_data[1];
  goal_s = road_data[2];
  goal_lane = road_data[3];
  max_acceleration = road_data[4];
}
