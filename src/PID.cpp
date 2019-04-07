#include "PID.h"
#include <iostream>
#include <cmath>

void PID::Init(double Kp_, double Ki_, double Kd_) {

  Ks = {Kp_, Ki_, Kd_};

  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;

  deltas = {Kp_ / 10.0, Ki_ / 10.0, Kd_ / 10.0};

  index = 0;
  iterations = 0;
  mse = 0.0;
  plus = false;
  number_laps = 0;
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
  mse += pow(cte, 2);

  const bool lap_one_complete = (iterations == number_samples);
  if (lap_one_complete) {
    number_laps++;
    printf("\nLap %d complete.\n", number_laps);
    // initialize best error
    best_error = mse / number_samples;
    std::cout << "Lap error: " << best_error << std::endl;
    initialize_with_new_parameter();
    return;
  }

  const bool lap_2_or_higher_complete = (iterations % number_samples == 0) && (iterations >= 2 * number_samples);
  if (lap_2_or_higher_complete) {
    number_laps++;
    printf("\nLap %d complete.\n", number_laps);
    const double tolerance = 0.004;
    if (best_error < tolerance) {
      std::cout << "\n\nFinal parameters: Kp=" << Ks[0] << ", Ki=" << Ks[1] << ", Kd=" << Ks[2] << "\n\n" << std::endl;
      return;
    }
    twiddle_iteration();
    std::cout << "Best error after twiddle: " << best_error << std::endl;
    std::cout << "New parameters: Kp=" << Ks[0] << ", Ki=" << Ks[1] << ", Kd=" << Ks[2] << std::endl;
  }

}

void PID::initialize_with_new_parameter() {
  std::cout << "Initialize with new parameter." << std::endl;
  Ks[index] += deltas[index];
  plus = true;
  mse = 0.0;
}

void PID::twiddle_iteration() {
  mse /= number_samples;

  std::cout << "Lap error: " << mse << std::endl;

  if (mse < best_error) {
    steps_after_improvement();
  } else {
    steps_after_deterioration();
  }
  std::cout << "Deltas: " << deltas[0] << ", " << deltas[1] << ", " << deltas[2] << ", " << std::endl;
  // prepare next iteration
  mse = 0.0;
}

void PID::steps_after_improvement() {
  best_error = mse;
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

    // next parameter
    index++;
    index %= 3;
    initialize_with_new_parameter();
  }
}