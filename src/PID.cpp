#include "PID.h"
#include <vector>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool twiddle_it) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	p_error = 0;
	i_error = 0;
	d_error = 0;

	this->twiddle_it = twiddle_it;
	counter = 0;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	i_error += cte;
	p_error = cte;
	if (twiddle_it && counter % 10 == 0 && counter != 0) {
		Twiddle(cte);
	}
}

double PID::TotalError(bool steering_angle) {
	double total_error = -Kp * p_error - Kd * d_error - Ki * i_error;

	// ensure total error is in the [-1, 1] range
	total_error = total_error < -1 ? -1 : total_error;
	total_error = total_error > 1 ? 1 : total_error;

	return total_error;

}

void PID::SetErrors(double p_error, double i_error, double d_error) {
	this->p_error = p_error;
	this->i_error = i_error;
	this->d_error = d_error;
}

void PID::Twiddle(double cte) {

	// initialization parameter vector
	std::vector<double>	p = { Kp, Ki, Kd };

	std::vector<double> dp = { 1.0, 1.0, 1.0 };

	double best_error = TotalError(false);
	double threshold = 0.00001;

	while (dp[0] + dp[1] + dp[2] > threshold){
		PID test_pid;
		double test_error;
		for (int i = 0; i < p.size(); i++) {
			p[i] += dp[i];
			
			test_pid.Init(p[0], p[1], p[2],false);
			test_pid.SetErrors(p_error, i_error, d_error);
			test_pid.UpdateError(cte);
			test_error = test_pid.TotalError(false);
			if (test_error < best_error) {
				dp[i] *= 1.1;
				best_error = test_error;
			}else {
				p[i] -= 2 * dp[i];
				test_pid.Init(p[0], p[1], p[2],false);
				test_pid.SetErrors(p_error, i_error, d_error);
				test_pid.UpdateError(cte);
				test_error = test_pid.TotalError(false);
				if (test_error < best_error) {
					dp[i] *= 1.1;
					best_error = test_error;
				}else {
					p[i] += dp[i];
					dp[i] *= 0.9;
				}
			}
		}
	}
	Kp = p[0];
	Ki = p[1];
	Kd = p[2];

	std::cout << "Kp " << Kp << " Ki " << Ki << " Kd " << Kd << std::endl;
}
