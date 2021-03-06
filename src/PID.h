#ifndef PID_H
#define PID_H

#include <vector>

using std::vector;

class PID {
 public:
  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  void twiddle(double cte);

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */
  vector<double> Ks;

  /**
   * Twiddle parameters and helper methods
   */
  vector<double> deltas;
  int index;
  int iterations;
  double mse;
  double best_error;
  bool plus;
  int number_laps;
  const int number_samples = 720; // 720 = about 1 lap
  void twiddle_iteration();
  void initialize_with_new_parameter();
  void steps_after_improvement();
  void steps_after_deterioration();
};

#endif  // PID_H