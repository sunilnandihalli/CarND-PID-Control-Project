#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <vector>
#include <fstream>
#define HAVE_CSTDDEF
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
using namespace std;
// for convenience
using json = nlohmann::json;
const double Lf = 2.67;
const double tol = 1e-5;
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

struct twiddler{
  double twiddle_rate,bestCteAvg;
  vector<double> bestP,dp,curP;
  int bestN;
  int Nmax;
  int paramId;
  int r;
  bool firstTime;
  int iter;
  double curtol;
  enum {POS,NEG} curTry;
  twiddler() {
    double b0,b1,b2,b3,b4,d0,d1,d2,d3,d4;
    {
      
      b0=b1=b2=b3=b4=0.0;
      //b0 = -0.134611, b1= -0.000270736, b2=-3.05349;
      //      b0 = -0.154719,b1 = -0.000198047, b2 = -3.86494;
      b0 = -0.154719,b1=-0.000198047,b2=-3.86494,b3=0.316731,b4=0.0226185;
      d0=d1=d2=d3=d4=0.0;
      d0 = fabs(b0*0.1);
      d1 = fabs(b1*0.1);
      d2 = fabs(b2*0.1);
      d3 = fabs(b3*0.1);
      d4 = fabs(b4*0.1);
    }
    bestP={b0,b1,b2,b3,b4};
    dp={d0,d1,d2,d3,d4};
    curP=bestP;
    twiddle_rate = 0.1;
    bestCteAvg = 1e99;
    paramId = 0;
    curTry = POS;
    firstTime = true;
    iter = 0;
    r = 2;
    curtol = 0.01;
    bestN=0;
    Nmax = 10000;
  }
  bool operator()(PID& pid) {
    double curCteAvg = pid.AvgError();
    if(firstTime) {
      firstTime = false;
      curP[paramId]+=dp[paramId];
    } else {
      if(pid.steps>bestN || (pid.steps==bestN && fabs(curCteAvg)<fabs(bestCteAvg))) {
	bestCteAvg = curCteAvg;
	bestP = curP;
        bestN = pid.steps;
	dp[paramId]*=(1.0+twiddle_rate);
	paramId++;
	if(paramId==3) {
	  iter++;
	  if(dp[0]+dp[1]+dp[2]+dp[3]+dp[4]<curtol) {
	    if(bestN>=Nmax) {
	      cout<<"best params found "<<endl;
	      return false;
	    } 
	  }
	  paramId=0;
	}
	curTry = POS;
	curP[paramId]+=dp[paramId];
      } else {
	if(curTry==POS) {
	  curTry = NEG;
	  curP[paramId]-=(2*dp[paramId]);
	} else {
	  curP[paramId]+=dp[paramId];
	  dp[paramId]*=(1.0-twiddle_rate);
	  paramId++;
	  if(paramId==3) {
	    iter++;
	    if(dp[0]+dp[1]+dp[2]+dp[3]+dp[4]<curtol) {
	      if(bestN>=Nmax) {
		cout<<"best params found "<<endl;
		return false;
	      } 
	    }
	    paramId = 0;
	  }
	  curTry = POS;
	  curP[paramId]+=dp[paramId];
	}
      }
    }
    pid.Init(curP[0],curP[1],curP[2],curP[3],curP[4]);
    return true;
  }
};

int main() {
  uWS::Hub h;
  PID pid;
  twiddler t;
  t(pid);
  bool first = true;
  h.onMessage([&pid,&t,&first](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
      static double cte_min,cte_max,fabs_cte_avg;
      if(first) {
	first = false;
	std::string msg = "42[\"reset\",{}]";
	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	return;
      }
      if (length && length > 2 && data[0] == '4' && data[1] == '2') {
	auto s = hasData(std::string(data).substr(0, length));
	if (s != "") {
	  auto j = json::parse(s);
	  std::string event = j[0].get<std::string>();
	  if (event == "telemetry") {
            double cte = std::stod(j[1]["cte"].get<std::string>());
	    double speed = std::stod(j[1]["speed"].get<std::string>());
	    double angle = std::stod(j[1]["steering_angle"].get<std::string>());
            if(cte<cte_min) cte_min=cte;
            if(cte>cte_max) cte_max=cte;
            fabs_cte_avg = pid.AvgError();
	    pid.UpdateError(cte);
            double steer_value,throttle_value;
            pid.GetControls(steer_value,throttle_value);
	    if(pid.steps>500 || cte>99999999 || fabs(steer_value)>999999) {
              cout<<"current bestParams ";
              for(auto x:t.bestP)
                cout<<x<<" ";
              cout<<endl;
	      if(t(pid)) {
		std::string msg = "42[\"reset\",{}]";
		ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                cte_min=1e99;
                cte_max=-1e99;
                fabs_cte_avg=0;
	      }
	    } else {
	      json msgJson;
	      msgJson["steering_angle"] = steer_value;
	      msgJson["throttle"] = throttle_value;
	      auto msg = "42[\"steer\"," + msgJson.dump() + "]";
	      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	    }
	  }
	} else {
	  std::string msg = "42[\"manual\",{}]";
	  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	}
      }
    });
  
  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
      const std::string s = "<h1>Hello world!</h1>";
      if (req.getUrl().valueLength == 1) {
	res->end(s.data(), s.length());
      } else {
	// i guess this should be done more gracefully?
	res->end(nullptr, 0);
      }
    });
  
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
      std::cout << "Connected!!!" << std::endl;
    });
  
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
