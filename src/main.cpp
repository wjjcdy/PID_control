#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

const int TWIDDLE_INIT =0;
const int TWIDDLE_RUN = 1;
const int TWIDDLE_ERR_CNT_RUN = 2;
const int TWIDDLE_ERR_CHECK = 3;
const int TWIDDLE_RUN_SECOND = 4;
const int TWIDDLE_ERR_CNT_RUN_SECOND = 5;
const int TWIDDLE_ERR_CHECK_SECOND = 6;


double P[3] = {0,0,0};
  double dp[3] = {1,1,1};
  
  double err = 0;
  double best_err = 1000000;
  int state_curr = TWIDDLE_INIT;
  int state_next = TWIDDLE_INIT;
  
  int p_index = 0;
  int count=0;
  int twiddle_ok = 1;   // count twiddle run number 


int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  pid.Init(0.15, 0.001, 1.5);
  

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          if(twiddle_ok == 0)
          { 
            switch(state_curr)
            {
              case TWIDDLE_INIT:
              {
                P[p_index] += dp[p_index]; 
                pid.Pid_set(P);                //update kp ki kd
                state_next = TWIDDLE_RUN;
                count=0;

                double dp_sum = 0;
                for (int i=0;i <3;++i)
                {
                  dp_sum += dp[i];
                }
                if(dp_sum<0.02)               // twiddle finished, the best pid hyperparameter is got
                {
                  twiddle_ok = 1;
                  printf("Best Kp=%f,Ki=%f,Kd=%f\n",P[0],P[1],P[2]);
                }
              }
              break;
              case TWIDDLE_RUN:
              {
                count++;
                if(count>=100)                // run 100 times pid control
                {
                  state_next = TWIDDLE_ERR_CNT_RUN;
                  count = 0;
                  err = 0;
                }
              }
              break;
              case TWIDDLE_ERR_CNT_RUN:       // run another 100 times pid control and count err
              {
                count++;
                err += cte*cte;
                if(count>=100)
                {
                  state_next = TWIDDLE_ERR_CHECK;
                  count = 0;
                }
              }
              break;
              case TWIDDLE_ERR_CHECK:
              {
                if (err < best_err)             //chec err is smaller than best_err
                {
                  dp[p_index] *= 1.1;           // indicate this direction is right 
                  best_err = err;
                  p_index++;                    //check next hyperparameter
                  if(p_index>=3)
                  {
                    p_index = 0;
                  }
                  state_next = TWIDDLE_INIT;    // Next round and change hyperparameter
                } 
                else                             // err is not smaller than best_err
                {
                  P[p_index] -= 2*dp[p_index];   // curr p change should change in the other direction
                  pid.Pid_set(P);                //update kp ki kd
                  state_next = TWIDDLE_RUN_SECOND;  //second check err_sum
                }
              }
              break;
              case TWIDDLE_RUN_SECOND:
              {
                count++;
                if(count>=100)                   // run 100 times pid control
                {
                  state_next = TWIDDLE_ERR_CNT_RUN_SECOND;
                  count = 0;
                  err = 0;
                }
              }
              break;
              case TWIDDLE_ERR_CNT_RUN_SECOND:
              {
                count++;
                err += cte*cte;                // run another 100 times pid control and count err
                if(count>=100)
                {
                  state_next = TWIDDLE_ERR_CHECK_SECOND;
                  count = 0;
                }
              }
              break;
              case TWIDDLE_ERR_CHECK_SECOND:
              {
                if (err < best_err)
                {
                  dp[p_index] *= 1.1;
                  best_err = err;
                }
                else                          //both direction can't make err smaller,make the range of dp smaller 
                {
                  P[p_index] += dp[p_index];  
                  dp[p_index] *=0.9;
                }
                p_index++;                    //check next hyperparameter
                if(p_index>=3)
                {
                  p_index = 0;
                }
                state_next = TWIDDLE_INIT;    // Next round and change hyperparameter
              }
              break;
            }
          }
          else
          {
            
          }
          
          state_curr = state_next;
            
          
          
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          if (steer_value > 1){
             steer_value = 1;
          } else if (steer_value <-1){
             steer_value = -1;
          }
          // DEBUG
          std::cout << "--------------CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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