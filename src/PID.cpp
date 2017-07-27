#include "PID.h"
#include <assert.h> 

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
	
	best_error = 0.0;
	
	dp[0] = 1;
	dp[1] = 1;
	dp[2] = 1;
	
	adjust_index = 0;
	
	first_run = true;
	
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
	return  total_cte;
}

void PID::SetAdjustIndex(int index) {
	assert(0 <= index && index <= 2);
	adjust_index = index;
}

void PID::MulitpleDP(float factor){
	dp[adjust_index] *= factor;
}

int PID::CheckOperations() {
	
	if(stack_operations.top() == First_AddDP) {
		return 1;
	}
	
	if(stack_operations.top() == Second_Minus2DP){
		return -2;
	}
	
	if(stack_operations.top() == Third_AddDP){
		return 1;
	}
	
}

void PID::AdjustParams() {
	
	switch(adjust_index) {
		case 0:
			Kp += dp[adjust_index] * CheckOperations();
		break;
		case 1:
			Ki += dp[adjust_index] * CheckOperations();
		break;
		case 2:
			Kd += dp[adjust_index] * CheckOperations();
		break;
	}
	
	total_cte = 0.0;
	
}

bool  PID::DPThreshold(float total){
	if(dp[0] + dp[1] + dp[2] > total)
		return false;
	else 
		return true;
}

