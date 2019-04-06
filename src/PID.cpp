#include "PID.h"
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {

  Ks = {Kp_, Ki_, Kd_};

  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;

  deltas = {0.01, 0.0001, 0.01};
  index = 0;
  iterations = 0;
  error_sum = 0.0;
  plus = false;
  initializeWithNewParameter = true;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  return Ks[0] * p_error + Ks[1] * i_error + Ks[2] * d_error;
}

double PID::twiddle(double cte) {
//  double tolerance = 0.001;
//  if (best_error > tolerance) {}
  iterations++;
  std::cout << "Iteration: " << iterations << std::endl;
  int number_samples = 8;
  int initial_offset = 60;

  // allow controller to stabilize before doing anything
  if (iterations > initial_offset) {
    error_sum += cte;
  }

  if (iterations == (initial_offset + number_samples)) {
    // initialize best error
    best_error = error_sum / number_samples;
    error_sum = 0.0;
  }

  if ((iterations % number_samples) == 0 && iterations >= (initial_offset + number_samples)) {
    std::cout << "Perform twiddle iteration." << std::endl;
    if (initializeWithNewParameter) {
      Ks[index] += deltas[index];
      plus = true;
      initializeWithNewParameter = false;
    } else {
      double err_avg = error_sum / number_samples;
      if (err_avg < best_error) {
        best_error = err_avg;
        deltas[index] *= 1.1;
        if (plus) {
          Ks[index] += deltas[index];
        } else {
          Ks[index] -= deltas[index];
        }
      } else {
        if (plus) {
          plus = false;
          Ks[index] -= 2 * deltas[index];
        } else {
          Ks[index] += deltas[index]; // reset
          deltas[index] *= 0.9;

          // next K parameter
          initializeWithNewParameter = true;
          std::cout << "Start iteration for next parameter." << std::endl;
          index++;
          index %= 3;
        }
      }
    }
    std::cout << "Updated parameters to: Kp=" << Ks[0] << ", Ki=" << Ks[1] << ", Kd=" << Ks[2] << std::endl;
  }


  // prepare next iteration
  error_sum = 0.0;
  return best_error;
}
