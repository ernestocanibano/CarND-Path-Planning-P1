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

  //reference speed
  double ref_speed = 0.0;
  int next_lane = 2;
    
  h.onMessage([&next_lane,&ref_speed,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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
		  double lane_width = 4; // Lane width in meters
		  int wp_spacing = 30; 	//distance in meters between two reference waypoints
		  int horizont = 30;   //distance ahead to generate a new trajectory
		  auto prev_size = previous_path_x.size(); //size of the previous path
		  double acceleration = 4; // acceleration m/s^2
		  double max_acceleration = 9; // acceleration m/s^2
		  double max_speed = 49 * 0.44704; //maximum speed m/s
		  double target_speed = ref_speed;
		  
		  //check which line is my car
		  auto lane = getLane(car_d, lane_width);
		 
		  //calculate acceleration in function of the points moved
		  if(prev_size == 0){
			end_path_s =  car_s;
		  }
		  		  
		  //look for the closest car in front of me
		  bool car_in_front = false;
		  bool car_on_left = false;
		  bool car_on_left_slow = false;
		  bool car_on_right = false;
		  bool car_on_right_slow = false;
		  float new_speed = 0;
		 
		  for(int i=0; i<sensor_fusion.size(); i++){
			float check_car_d = sensor_fusion[i][6];
			float check_car_s = sensor_fusion[i][5];
			float vx = sensor_fusion[i][3];
			float vy = sensor_fusion[i][4];
			float check_car_v = sqrt(vx*vx+vy*vy);
						
			int check_lane = getLane(check_car_d, lane_width);
			
			if(check_lane > 0){//ignore them
				if(check_lane == lane){
					if( (check_car_s-car_s) > 0){
						if( (check_car_s-end_path_s) < 0){ //possible collission stop quickly
							car_in_front = true;
							new_speed = 0;
							acceleration = max_acceleration;
						}else if( (check_car_s-end_path_s) < 30){ //adapt of the speed of the car ahead
							car_in_front = true;
							new_speed = check_car_v;
						}
					}
				}else if(check_lane == (lane-1)){
					if( (check_car_s-car_s) < -5 && (check_car_s-car_s) > -20){
						if(check_car_v > ref_speed){
							car_on_left = true;
						}
					}else  if( (check_car_s-car_s) > -5){
						if( (check_car_s-end_path_s) < 10){
							car_on_left = true;
						}else if( (check_car_s-end_path_s) < 30){
							car_on_left = true;
						}
					}
				}else if(check_lane == (lane+1)){
					if( (check_car_s-car_s) < -5 && (check_car_s-car_s) > -20){
						if(check_car_v > ref_speed){
							car_on_right = true;
						}
					}else  if( (check_car_s-car_s) > -5){
						if( (check_car_s-end_path_s) < 10){
							car_on_right = true;
						}else if( (check_car_s-end_path_s) < 30){
							car_on_right = true;
						}
					}
				}
			}
		  }
		  
		 //only perform other action if a lane changing is not in process
		 if(lane == next_lane){
			 if(car_in_front){
				if(lane == 2){ //center
					if(!car_on_left){
						lane--;
						//target_speed = max_speed; 
					}else if(!car_on_right){
						lane++;
						//target_speed = max_speed; 
					}else{
						target_speed = new_speed;
					}
				}else if(lane == 1){ //left
					if(!car_on_right){
						lane++;
						//target_speed = max_speed;
					}else{
						target_speed = new_speed;
					}
				}else if(lane == 3){//right
					if(!car_on_left){
						lane--;
						//target_speed = max_speed; 
					}else{
						target_speed = new_speed;	
					}
				}
			 }else{
				target_speed = max_speed; 
			 }
			 next_lane = lane ;
		 }
		 
		  
		  //This point is the beginning of the trajectory
		  auto current_yaw = deg2rad(car_yaw);
		  double current_x;
		  double current_y;
		  
		  /** The new trajectory of the car has to be tangent to the previous trajectory.
		   *  It will be generated using several reference waypoints and will be smoothed  using spline.
		   *  The first two points of the new trajectory have match the last two points of the old tajectory to get the tangency.
		   */
		  //Reference waypoints to generate a trajectory
		  vector<double> ref_x_vals; //reference trajectory 'x' values
          vector<double> ref_y_vals; //reference trajectory 'y' values
		  vector<double> ref_wp0(2);
		  vector<double> ref_wp1(2);
		  		  		  
		  //If an old trajectory doesn't exist, it can be use the current position of the car as second point of the new trajectory.
		  //The first one is calculated backwards knowing the current yaw angle of the car and assuming the minimum distance unit (1 meter)
		  if(prev_size < 2){
			ref_wp0[0] = car_x - cos(current_yaw);
			ref_wp0[1] = car_y - sin(current_yaw);
			ref_wp1[0] = car_x;
			ref_wp1[1] = car_y;
		  }
		  //if exists an old trajectory, the last two points will be used.
		  else{
			ref_wp0[0] = previous_path_x[prev_size-2];
			ref_wp0[1] = previous_path_y[prev_size-2];
			ref_wp1[0] = previous_path_x[prev_size-1];
			ref_wp1[1] = previous_path_y[prev_size-1]; 
			//update the yaw angle 
			current_yaw = atan2(ref_wp1[1]-ref_wp0[1], ref_wp1[0]-ref_wp0[0]);
			
			//Add the remaining points of the old trajectory to the new trajectory
			for(int i=0; i<prev_size; i++){
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			 }
		  }
		  current_x = ref_wp1[0];
		  current_y = ref_wp1[1];
		  
		  //Add more waypoints to be used as reference, three waypoints are enough to generate a lane change.
		  vector<double> ref_wp2 = getXY(end_path_s+1*wp_spacing, (next_lane-1)*lane_width+lane_width/2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  vector<double> ref_wp3 = getXY(end_path_s+2*wp_spacing, (next_lane-1)*lane_width+lane_width/2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  vector<double> ref_wp4 = getXY(end_path_s+3*wp_spacing, (next_lane-1)*lane_width+lane_width/2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  
		  ref_x_vals.push_back(ref_wp0[0]);
		  ref_x_vals.push_back(ref_wp1[0]);
		  ref_x_vals.push_back(ref_wp2[0]);
		  ref_x_vals.push_back(ref_wp3[0]);
		  ref_x_vals.push_back(ref_wp4[0]);
		  ref_y_vals.push_back(ref_wp0[1]);
		  ref_y_vals.push_back(ref_wp1[1]);
		  ref_y_vals.push_back(ref_wp2[1]);
		  ref_y_vals.push_back(ref_wp3[1]);
		  ref_y_vals.push_back(ref_wp4[1]);
		  
		  //To use spline points have to be sorted, therefore it is better work in local coordinates from here
		  for(int i=0; i<ref_x_vals.size(); i++){
			vector<double> point = getLocalXY(ref_x_vals[i], ref_y_vals[i], current_x, current_y, current_yaw);
			ref_x_vals[i] = point[0];
			ref_y_vals[i] = point[1];
		  }
		 
		  //use spline to smooth the reference trajectory
		  tk::spline s;
		  s.set_points(ref_x_vals, ref_y_vals);
		  
		  double target_x = horizont;
		  double target_y = s(target_x);
		  double target_theta = atan2(target_y, target_x);
		  double target_distance = 0;
		  
		     
		  //Add the new points to trajectory 
		  for(int i=1; i<=(50-prev_size); i++){
			
			if(ref_speed > target_speed){
				ref_speed -= (acceleration * 0.02);
				if(ref_speed < 0){
					ref_speed = 0;
				}
			}else if (ref_speed < target_speed){
				ref_speed += (acceleration * 0.02);
				if(ref_speed > max_speed){
					ref_speed = max_speed;
				}
			}
			
			target_distance += ref_speed*0.02;
			double x_point = target_distance*cos(target_theta);
			double y_point = s(x_point);
			
			vector<double> next_point = getGlobalXY(x_point, y_point, current_x, current_y, current_yaw); 
		
			next_x_vals.push_back(next_point[0]);
			next_y_vals.push_back(next_point[1]);
		  }
			
		  /***********************************************************/


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

  h.onConnection([&ref_speed, &h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
	ref_speed = 0.0;
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