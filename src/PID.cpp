#include "PID.h"
#include <iostream>
#include <cmath>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {

  Ks = {Kp_, Ki_, Kd_};

  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;

  deltas = {Kp_ / 10.0, Ki_ / 10.0, Kd_ / 10.0};
  index = 0;
  iterations = 0;
  error_sum = 0.0;
  plus = false;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  return Ks[0] * p_error + Ks[1] * i_error + Ks[2] * d_error;
}

void PID::twiddle(double cte) {
  iterations++;
  int number_samples = 200; // 800 = about 1 round
  int initial_offset = 60;

  // allow controller to stabilize before doing anything
  if (iterations > initial_offset) {
    error_sum += cte;
  }

  if (iterations == (initial_offset + number_samples)) {
    // initialize best error
    best_error = fabs(error_sum / number_samples);
    error_sum = 0.0;
    initialize_with_new_parameter();
  }

  if (((iterations - initial_offset) % number_samples) == 0 && iterations > (initial_offset + number_samples)) {
    double tolerance = 0.001;
    if ((best_error < tolerance) && (iterations > 1000)) {
      std::cout << "\n\nFinal parameters: Kp=" << Ks[0] << ", Ki=" << Ks[1] << ", Kd=" << Ks[2] << "\n\n" << std::endl;
      return;
    }
    std::cout << "Iteration: " << iterations << std::endl;
    std::cout << "Best error before twiddle: " << best_error << std::endl;
    twiddle_iteration(number_samples);
    std::cout << "Best error after twiddle: " << best_error << std::endl;
    std::cout << "Parameters: Kp=" << Ks[0] << ", Ki=" << Ks[1] << ", Kd=" << Ks[2] << std::endl;
  }
}

void PID::next_parameter() {
  index++;
  index %= 3;
  initialize_with_new_parameter();
}

void PID::initialize_with_new_parameter() {
  std::cout << "Initialize with new parameter." << std::endl;
  Ks[index] += deltas[index];
  plus = true;
  error_sum = 0.0;
}

void PID::twiddle_iteration(int &number_samples) {
  double err_avg = fabs(error_sum / number_samples);

  if (err_avg < best_error) {
    steps_after_improvement(err_avg);
  } else {
    steps_after_deterioration();
  }
  std::cout << "Deltas: " << deltas[0] << ", " << deltas[1] << ", " << deltas[2] << ", " << std::endl;
  // prepare next iteration
  error_sum = 0.0;
}

void PID::steps_after_improvement(double &err_avg) {
  best_error = err_avg;
  deltas[index] *= 1.1;
  if (plus) {
    Ks[index] += deltas[index];
  } else {
    Ks[index] -= deltas[index];
  }
}

void PID::steps_after_deterioration() {
  if (plus) {
    plus = false;
    Ks[index] -= 2 * deltas[index];
  } else {
    Ks[index] += deltas[index]; // reset to last stable value
    deltas[index] *= 0.9;
    next_parameter();
  }
}