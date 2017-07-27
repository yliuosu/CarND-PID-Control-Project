#include "PID.h"
#include <assert.h> 

using namespace std;
#include <iostream>


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
	
	best_kp = 0;
	best_kd = 0;
	
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
	
	max_speed = 0.0;
	nummaxspeed = 0;
	
	best_max_speed = 0.0;
	best_nummaxspeed = 1;
	
}

void PID::UpdateError(double cte) {
	
	d_error = cte - prev_cte;
	p_error = cte;
	i_error += cte;
	prev_cte = cte;
	total_cte += cte * cte;
	msgcount++;
}

void PID::UpdateSpeed(double speed) {
	
	int spd = (int)speed * 10;
	if( max_speed == spd) {
		nummaxspeed++;
	} else if (max_speed < spd ) {
		max_speed = spd;
		nummaxspeed = 1;
	}
}

bool PID::Better(){
	if (best_max_speed < max_speed) {
		
		std::cout << "Best speed: ***************" << best_max_speed  << "---- max: " << max_speed << std::endl;
		return true;
	} else if(best_max_speed == max_speed && best_nummaxspeed< nummaxspeed  ) {
		
		std::cout << "Best speed: ***************" << best_max_speed  << "---- max: " << max_speed << std::endl;
		std::cout << "Best speed Num: ***************" << best_nummaxspeed  << "---- max num: " << nummaxspeed << std::endl;
		
		return true;
	}
	
	return false;
}

double PID::TotalError() {
	return  total_cte;
}

void PID::SetAdjustIndex(int index) {
	assert(0 <= index && index <= 2);
	adjust_index = index;
}

void PID::MulitpleDP(double factor){
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
			Kd += dp[adjust_index] * CheckOperations();
		break;
		case 2:
			Ki += dp[adjust_index] * CheckOperations();
		break;
	}
	
	//after adjust the parameter, reset the performance index
	total_cte = 0.0;
	max_speed = 0.0;
	nummaxspeed = 1;
	
}

void DisplayPerformanceComparison(int stage){
	//std::cout << "Best error:  " << best_error  << ", Current error: " << TotalError() << std::endl;
	std::cout << stage << " Best Max Speed:  " << best_max_speed  << ",  Current Max Speed:  " << max_speed  << best_nummaxspeed << std::endl;
	std::cout << stage<< "Best Max Speed Point Number:  " << best_nummaxspeed  << ",  Current Max Speed Number:  " << nummaxspeed  << std::endl;
}

void DisplayAdjustmentArray() {
	 // display the adjustment array information
	std::cout << "DPS:  [ " << dp[0] << ", " << dp[1] << ", " << dp[2]  << " ] "<< std::endl;
}

void DisplayCurrentBestParameter(){
	std::cout << "Best KP:  " <<.best_kp << ", Best Kd: " <<.best_kd << ", Best Ki" std::endl;
}

void PID::Twiddle(int nStep){
	
	if (msgcount >= nStep ) {
		
			   if(first_run == true) {
				  
				   DisplayPerformanceComparison(1);
			
				   // update the best records
				   best_max_speed = max_speed;
				   best_nummaxspeed = nummaxspeed;
				
				   first_run = false;
				   
				   // choose the first parameters to be adjusted
				   SetAdjustIndex(0);
				   // record the adjustment operation so that if the adjustment is not good we know what to do next
				   stack_operations.push(First_AddDP);
				   // adjust the parameter
				   AdjustParams();
				  
				  // display the adjustment array information
				  DisplayAdjustmentArray();
	
			   } else {
				  
					 if(Better() == true && stack_operations.top() == First_AddDP ) {
					   // the previous adjustment operation is the add dp[i], the performance improves, then adjust the next parameter
					   DisplayPerformanceComparison(1);
					   
					   // change the range of the adjustment array dp[i]
					   MulitpleDP(1.1);
					
						// update the best records
						best_max_speed = max_speed;
						best_nummaxspeed = nummaxspeed;
						
						// update the current best parameters
						best_kp = Kp ;
						best_kd = Kd;
						DisplayCurrentBestParameter();
					
						// switch to next parameter
						adjust_index = (adjust_index + 1) % 2;
						AdjustParams();
						
						// display the adjustment array information
						 DisplayAdjustmentArray();
					
					 } else if( Better() == false && stack_operations.top() == First_AddDP ) {
						 
						// the previous adjustment operation is the add dp[i], the performance drops, then we minus 2*dp[i] from the parameter
						 DisplayPerformanceComparison(2);
					   
						// the preivous adjustment is not good, keep tuning the same parameter
						// remove the previous operation from the stack
						stack_operations.pop();
						stack_operations.push(Second_Minus2DP);
						AdjustParams();
						
						// display the adjustment array information
						 DisplayAdjustmentArray();
					  } else if(Better() == true  && stack_operations.top() == Second_Minus2DP ) {
						  
						// the previous minus 2*dp[i] adjustment is good, update the parameters and  switch the next parameter to tuning
						DisplayPerformanceComparison(3);
						
						// change the range of the adjustment array dp[i]
						MulitpleDP(1.1);
						
						// update the best records
						best_max_speed = max_speed;
						best_nummaxspeed = nummaxspeed;
						
						// update the current best parameters
						best_kp = Kp ;
						best_kd = Kd;
						DisplayCurrentBestParameter();
						
						// switch to next parameter
						adjust_index = (adjust_index + 1) % 2;
						stack_operations.pop();
						stack_operations.push(First_AddDP);
						// adjust it parameter
						AdjustParams();
						
						// display the adjustment array information
						 DisplayAdjustmentArray();

					} else if( Better() == false && stack_operations.top() == Second_Minus2DP ) {
						
						// the previous minus 2*dp[i] adjustment is not good, add dp[i] back to the parameters, narrow down the range of the dp[i] and  switch the next parameter to tuning
						DisplayPerformanceComparison(4);
						
						// the second adjustment is not good then reduce the range of the adjustment
						pid.stack_operations.pop();
						pid.stack_operations.push(Third_AddDP);
						pid.AdjustParams();
						
						// change the range of the adjustment array dp[i]
						pid.MulitpleDP(0.9);
						
						// switch next parameter
						pid.adjust_index = (pid.adjust_index + 1) % 2;
						pid.stack_operations.pop();
						pid.stack_operations.push(First_AddDP);
						// adjust it parameter
						pid.AdjustParams();
						
						// display the adjustment array information
						 DisplayAdjustmentArray();
					}
			   }
	
}

