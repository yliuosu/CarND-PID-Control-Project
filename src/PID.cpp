#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
	
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
	
	Kp = 1.0;
	Ki = 1.0;
	Kd = 1.0;
	
	prev_cte = 0.0;
	
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	prev_cte = 0.0;
	total_cte = 0.0;
}

void PID::UpdateError(double cte) {
	
	d_error = cte - prev_cte;
	p_error = cte;
	i_error += cte;
	prev_ct = cte;
	total_cte += cte * cte;
	
}

double PID::TotalError() {
	
	return  total_cte;
}

