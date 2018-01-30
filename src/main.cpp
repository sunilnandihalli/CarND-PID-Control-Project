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

Eigen::VectorXd polyfit(vector<double> xvals,vector<double> yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  /*cout<<"polyfit : ";
  for(int i=0;i<xvals.size();i++) {
    cout<<"("<<xvals[i]<<","<<yvals[i]<<") ";
  }
  cout<<endl;*/
  Eigen::MatrixXd A(xvals.size(), order + 1);
  Eigen::VectorXd Y(xvals.size());
  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
    Y(i) = yvals[i];
  }
  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals[j];
    }
  }
  auto Q = A.householderQr();
  auto result = Q.solve(Y);
  return result;
}
double polyeval(Eigen::VectorXd& coeffs, double x) {
  auto& c(coeffs);
  double result = 0.0;
  for(int i=c.size()-1;i>-1;i--)
    result = result*x+c[i];
  return result;
}

double polyDerivativeEval(Eigen::VectorXd coeffs, double x) {
  auto& c(coeffs);
  double result = 0;
  for (int i=c.size()-1;i>0;i--) {
    result = result*x + c[i]*i;
  }
  return result;
}

tuple<vector<double>,vector<double>> toCarCoords(double px,double py,double psi,vector<double> xs,vector<double> ys) {
  //cout<<" toCarCoords inp-sizes : "<<xs.size()<<" "<<ys.size()<<endl;
  double ct = cos(psi);
  double st = sin(psi);
  auto f = [&px,&py,&ct,&st](double x,double y)->tuple<double,double> {
    double dx = x-px;
    double dy = y-py;
    double x1= dx*ct+dy*st;
    double y1= -dx*st+dy*ct;
    return std::make_tuple(x1,y1);
  };
  vector<double> rxs(xs.size()),rys(xs.size());
  for(int i =0;i<xs.size();i++) {
    std::tie(rxs[i],rys[i]) = f(xs[i],ys[i]);
  }
  return std::make_tuple(rxs,rys);
}

struct simulator {
  vector<double> ptsx,ptsy;
  int ws/*waypoint_start*/;
  int nws/*number of waypoints sent back*/;
  double t,x0,y0,psi0,v0,delta0,a0;
  simulator() {
    string p("/home/sunil/carnd/CarND-MPC-Project/lake_track_waypoints.txt");
    ws = 0;
    nws = 6;
    ifstream fin(p);
    double x,y;
    while(fin) {
      fin>>x>>y;
      ptsx.push_back(x);
      ptsy.push_back(y);
    }
  }
  

  void startPid(double& cte,double& v,double& delta,double& a){
    double x,y,psi;
    vector<double> xs,ys;
    start(x,y,psi,v,xs,ys);
    Eigen::VectorXd C = polyfit(xs,ys,3);
    double fx = polyeval(C,0);
    double fdash = polyDerivativeEval(C,0);
    double norm = sqrt(1+fdash*fdash);
    cte = fx/norm; // y = 0
  }
  void start(double& x,double& y,double& psi,double& v,vector<double>& xs,vector<double>& ys) {
    vector<double> ptsx0,ptsy0;
    ws = 0;
    for(int i=0;i<nws;i++) {
      int l = ws+i;
      ptsx0.push_back(ptsx[l]);
      ptsy0.push_back(ptsy[l]);
    }
    x = ptsx[0];
    y = ptsy[0];
    double dx(ptsx[1]-x),dy(ptsy[1]-y);
    psi = atan2(dy,dx);
    v = 0.0;
    x0=x;
    y0=y;
    psi0=psi;
    v0=v;
    delta0=0;
    a0=0;
    std::tie(xs,ys) = toCarCoords(x0,y0,psi0,ptsx0,ptsy0);
  }
  bool nextStatePid(double dt,double delta1,double a1,double& cte1,double& v1) {
    double x1,y1,psi1;
    vector<double> cptsx1,cptsy1;
    nextstate(dt,delta1,a1,x1,y1,psi1,v1,cptsx1,cptsy1);
    Eigen::VectorXd C = polyfit(cptsx1,cptsy1,3);
    double fx = polyeval(C,0);
    double fdash = polyDerivativeEval(C,0);
    double norm = sqrt(1+fdash*fdash);
    cte1 = fx/norm; // y=0
  }
  bool nextstate(double dt/* time from last control input */,
		 double delta1,double a1, /* new control inputs */
		 double& x1,double& y1,double& psi1,double& v1, /* position of the vehicle control inputs were recieved */
		 vector<double>& cptsx1,vector<double>& cptsy1) {
    v1 = v0 + a0*dt;
    double dist = v0*dt+0.5*a0*dt*dt;
    psi1 = psi0 + (delta0/Lf)*dist;
    if(fabs(delta0)>tol) {
      double r  =  Lf/delta0;
      x1 = x0 + r*(sin(psi1)-sin(psi0));
      y1 = y0 + r*(cos(psi0)-cos(psi1));
    } else {
      double psi_avg = 0.5*(psi0+psi1);
      x1 = x0 + dist*cos(psi_avg);
      y1 = y0 + dist*sin(psi_avg);
    }
    {
      double ct(cos(psi1)),st(sin(psi1));
      for(int i = 0;i<nws;i++) {
	int l = i+ws;
	if(l==ptsx.size())
	  break;
	double dx(ptsx[l]-x1),dy(ptsy[l]-y1);
	if(dx*ct+dy*st > 0) {
	  ws = l;
	  //cout<<"waypoint_start : "<<ws;
	  break;
	}
      }
      vector<double> ptsx1,ptsy1;
      ptsx1.clear();
      ptsy1.clear();
      for(int i=0;i<nws;i++) {
	int l = i+ws;
	if(l==ptsx.size())
	  break;
	ptsx1.push_back(ptsx[l]);
	ptsy1.push_back(ptsy[l]);
      }
      if(ptsx1.size()<4)
	return false;
      cptsx1.clear();
      cptsy1.clear();
      std::tie(cptsx1,cptsy1) = toCarCoords(x1,y1,psi1,ptsx1,ptsy1);
    }
    {
      x0=x1;
      y0=y1;
      psi0=psi1;
      v0=v1;
      delta0=delta1;
      a0=a1;
      t+=dt;
    }
    return true;
  }
};

struct twiddler{
  double twiddle_rate,bestCteAvg;
  vector<double> bestP,dp,curP;
  int N,paramId;
  int r,Nmax;
  bool firstTime;
  int iter;
  double curtol;
  bool changedN;
  enum {POS,NEG} curTry;
  twiddler() {
    double b0,b1,b2,d0,d1,d2;
    {
      ifstream fin("/home/sunil/carnd/CarND-PID-Control-Project/checkpoint");
      if(fin)
	fin>>N>>b0>>b1>>b2>>d0>>d1>>d2;
      else {
	b0=b1=b2=0.0;
	d0=d1=d2=1.0;
	N=20;
      }
    }
    bestP={b0,b1,b2};
    dp={d0,d1,d2};
    curP=bestP;
    twiddle_rate = 0.1;
    bestCteAvg = 1e99;
    paramId = 0;
    curTry = POS;
    firstTime = true;
    iter = 0;
    r = 2;
    Nmax = 100000;
    curtol = 0.01;
    changedN = true;
  }
  twiddler(vector<double> _bP,vector<double> _dp,int _N):bestP(_bP),dp(_dp),curP(_bP),N(_N) {
    twiddle_rate = 0.1;
    bestCteAvg = 1e99;
    paramId = 0;
    curTry = POS;
    firstTime = true;
    iter = 0;
    r = 2;
    Nmax = 100000;
    curtol = 0.01;
    changedN = true;
  }
  bool operator()(int& nextN,PID& pid) {
    //cout<<" iter : "<<iter<<" N : "<<N<<" paramId : "<<paramId<<" curTry : "<<(curTry==POS?"POS":"NEG")<<endl;
    double curCteAvg = pid.error/N;
    if(firstTime) {
      firstTime = false;
      curP[paramId]+=dp[paramId];
    } else {
      if(changedN) {
	bestCteAvg = fabs(curCteAvg);
	changedN = false;
      }
      if(fabs(curCteAvg)<fabs(bestCteAvg)) {
	bestCteAvg = curCteAvg;
	bestP = curP;
	dp[paramId]*=(1.0+twiddle_rate);
	cout<<"curP : "<<curP[0]<<" "<<curP[1]<<" "<<curP[2]
	    <<" cur_error : "<<curCteAvg
	    <<" N : "<<N
	    <<" dp : "<<dp[0]<<" "<<dp[1]<<" "<<dp[2]<<endl;
	paramId++;
	if(paramId==3) {
	  iter++;
	  cout<<"iteration : "<<iter<<" best_error : "<<bestCteAvg<<" params : "<<bestP[0]<<" "<<bestP[1]<<" "<<bestP[2]<<endl;
	  cout<<" dp : "<<dp[0]<<" "<<dp[1]<<" "<<dp[2]<<endl;
	  if(dp[0]+dp[1]+dp[2]<curtol) {
	    if(N>=Nmax) {
	      cout<<"best params found "<<endl;
	      return false;
	    } else {
	      dp[0]*=3;
	      dp[1]*=3;
	      dp[2]*=3;
	      N*=r;
	      curtol*=0.01;
	      changedN = true;
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
	    //cout<<"iteration : "<<iter<<" best_error : "<<bestCteAvg<<" params : "<<bestP[0]<<" "<<bestP[1]<<" "<<bestP[2]<<" N : "<<N<<endl;
	    //cout<<" dp : "<<dp[0]<<" "<<dp[1]<<" "<<dp[2]<<endl;
	    if(dp[0]+dp[1]+dp[2]<curtol) {
	      if(N>=Nmax) {
		cout<<"best params found "<<endl;
		return false;
	      } else {
		dp[0]*=3;
		dp[1]*=3;
		dp[2]*=3;
		N*=r;
		curtol*=0.01;
		changedN = true;
	      }
	    }
	    paramId = 0;
	  }
	  curTry = POS;
	  curP[paramId]+=dp[paramId];
	}
      }
    }
    nextN = N;
    pid.Init(curP[0],curP[1],curP[2]);
    cout<<"N : "<<N<<" berror : "<<bestCteAvg<<" bP : "<<bestP[0]<<" "<<bestP[1]<<" "<<bestP[2]
	<<" ce : "<< curCteAvg<<" cP : "<<curP[0]<<" "<<curP[1]<<" "<<curP[2]<<" dp : "<<dp[0]<<" "<<dp[1]<<" "<<dp[2]<<endl;
    {
      ofstream fout("/home/sunil/carnd/CarND-PID-Control-Project/checkpoint");
      fout<<N<<" "<<bestP[0]<<" "<<bestP[1]<<" "<<bestP[2]<<" "<<dp[0]<<" "<<dp[1]<<" "<<dp[2]<<endl;
    }
    return true;
  }
};
void testrun() {
  PID pid;
  double dt = 0.05;
  int N;
  twiddler t;
  
  simulator s;
  double cte,v,delta(0),a(0);
  a=3;
  while(t(N,pid)) {
    int stepid = 0;
    s.startPid(cte,v,delta,a);
    while(stepid<N) {
      stepid++;
      pid.UpdateError(cte);
      delta = pid.TotalError();
      s.nextStatePid(dt,delta,a,cte,v);
    }
  }
}
int main() {
  //testrun();
  //return 0;
  //0.573839 -2.1 0
  uWS::Hub h;
  PID pid;
  int N,stepid=0;
  twiddler t;
  t(N,pid);
  //  pid.Init(0.573839,-2.1,0);
  double tol=1e-9;
  bool first = true;
  h.onMessage([&pid,&stepid,&t,&N,&first](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
	    stepid++;
	    double cte = std::stod(j[1]["cte"].get<std::string>());
	    double speed = std::stod(j[1]["speed"].get<std::string>());
	    double angle = std::stod(j[1]["steering_angle"].get<std::string>());
	    //std::cout<<" cte : "<<cte<<" speed : "<<speed<<" angle : "<<angle<<std::endl;
	    pid.UpdateError(cte);
	    if(stepid==N || (pid.error/N) > t.bestCteAvg) {
	      if(t(N,pid)) {
		stepid=0;
		//cout<<"restarting the simulator..."<<endl;
		//std::string msg = "42[\"restart\",{}]";
		std::string msg = "42[\"reset\",{}]";
		ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	      }
	    } else {
	      double steer_value;
	      steer_value = tanh(pid.TotalError());
	      //std::cout <<"sending Steering Value: " << steer_value << std::endl;
	      json msgJson;
	      msgJson["steering_angle"] = steer_value;
	      msgJson["throttle"] = 3;
	      auto msg = "42[\"steer\"," + msgJson.dump() + "]";
	      //std::cout << msg << std::endl;
	      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	    }
	  }
	} else {
	  // Manual driving
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
