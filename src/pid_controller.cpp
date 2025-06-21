#include "tracking/pid_controller.hpp"

PIDController::PIDController(double Kp, double Ki, double Kd, double max_output, double min_output)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd), max_output_(max_output), min_output_(min_output),
      prev_error_(0.0), integral_(0.0)
{
}

double PIDController::compute(double setpoint, double current_value, double dt)
{
    double error = setpoint - current_value;
    integral_ += error * dt;
    double derivative = (error - prev_error_) / dt;

    double output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
    prev_error_ = error;

    // Clamp output within limits
    if (output > max_output_)
        output = max_output_;
    else if (output < min_output_)
        output = min_output_;

    return output;
}
