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
	
	best_error = 100000;
	
	dp[0] = 0.2;
	dp[1] = 0.2;
	dp[2] = 0.2;
	
	adjust_index = 0;
	
	better = false;
	
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	
	prev_cte = 0.0;
	total_cte = 0.0;
	msgcount = 0;
	best_error = 0.0;
	
}

void PID::UpdateError(double cte) {
	
	d_error = cte - prev_cte;
	p_error = cte;
	i_error += cte;
	prev_cte = cte;
	total_cte += cte * cte;
	
	msgcount++;
}

double PID::TotalError() {
	adjust_index = (adjust_index + 1) % 3;
	return  total_cte;
}

void PID::MulitpleDP(float factor){
	dp[adjust_index] *= facot;
}

void PID::ADDP() {
	switch(adjust_index) {
		case 0:
			Kp += dp[0]
		break;
		case 1:
			Ki += dp[1];
		break;
		case 2:
			Kd += dp[2];
		break;
	}
}

bool  PID::DPThreshold(total){
	if(dp_P + dp_i + dp_e > total)
		return false;
	else 
		return true;
}

