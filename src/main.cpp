#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

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
    } else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main() {
    uWS::Hub h;

    PID pid;
    // TODO: Initialize the pid variable.

    h.onMessage([&pid](uWS::WebSocket <uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(std::string(data).substr(0, length));
            if (s != "") {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<std::string>());
                    double speed = std::stod(j[1]["speed"].get<std::string>());
                    //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                    double steer_value;
                    /*
                    * TODO: Calcuate steering value here, remember the steering value is
                    * [-1, 1].
                    * NOTE: Feel free to play around with the throttle and speed. Maybe use
                    * another PID controller to control the speed!
                    */


                    if (!pid.is_initialized_) {
                        // initialize errors with first measurement.
                        pid.Init(0.0, 0.0, 0.0, cte);
                        //pid.Init(0.12, 0.0, 1.5, cte);
                        // run first parameter update.
                        pid.ParamUpdate(pid.INCREASE_COEFF_);
                    } else {
                        pid.iter_counter ++;
                        pid.avg_error_ += cte;
                        if(pid.iter_counter % 10 == 0){
                            pid.avg_error_ = pid.avg_error_ / 10;
                            pid.UpdateError(cte);

                            if(pid.SumDeltas() > 0.001)
                            {
                                switch(pid.cur_step_)
                                {
                                    // after increasing coefficient
                                    case 1:
                                        if(fabs(pid.avg_error_) < fabs(pid.best_error_))
                                        {
                                            pid.DeltaUpdate(pid.INCREASE_DELTA_);
                                            pid.best_error_ = pid.avg_error_;
                                            std::cout << "BEST ERROR UPDATED!!!!!" << std::endl;
                                        }
                                        else
                                        {
                                            pid.ParamUpdate(pid.DOUBLE_DECREASE_COEFF_);
                                        }
                                        break;
                                    case 2:
                                        if(fabs(pid.avg_error_) < fabs(pid.best_error_))
                                        {
                                            pid.DeltaUpdate(pid.INCREASE_DELTA_);
                                            pid.best_error_ = pid.avg_error_;
                                            std::cout << "BEST ERROR UPDATED!!!!!" << std::endl;
                                        }
                                        else
                                        {
                                            pid.ParamUpdate(pid.INCREASE_COEFF_REDUCE_DELTA_);
                                        }
                                        break;
                                        // if in some case step 0 was not performed.
                                    default:
                                        pid.ParamUpdate(pid.INCREASE_COEFF_);
                                }
                            }
                            pid.avg_error_ = 0.0;
                        }

                    }

                    std::cout << "iter: " << pid.iter_counter <<  std::endl;
                    std::cout << "best error: " << pid.best_error_ <<  std::endl;
                    std::cout << "cur step: " << pid.cur_step_ <<  std::endl;
                    std::cout << "p_error: " << pid.p_error_ << "d_error: " << pid.d_error_ << "i_error: " << pid.i_error_ << std::endl;
                    std::cout << "cur pos: " << pid.cur_pos_ <<  std::endl;
                    std::cout << "koef1: " << pid.koeffs_[0] << " | koef2: " << pid.koeffs_[1] << " | koef3: " << pid.koeffs_[2] << std::endl;
                    steer_value = pid.TotalError();

                    // DEBUG
                    std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

                    // give it a chance to tweak coefficients moving slowly
                    // then accelerate before and after the turn.
                    double throttle = 0.03;
                    if(pid.iter_counter > 5000)
                    {
                        throttle = 0.1;
                    }
                    if(pid.iter_counter > 10000)
                    {
                        throttle = 0.2;
                    }

                    if(speed > 25.0)
                        throttle = 0.0;
                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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

    h.onConnection([&h](uWS::WebSocket <uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket <uWS::SERVER> ws, int code, char *message, size_t length) {
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
