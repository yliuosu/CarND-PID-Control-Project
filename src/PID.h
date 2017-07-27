#ifndef PID_H
#define PID_H

#include <stack>


enum Operations {First_AddDP, Second_Minus2DP, Third_AddDP} ;

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  
  double prev_cte;
  double total_cte;
  
   int msgcount;
   
   bool first_run;
   float best_error;
   
   std::stack<Operations> stack_operations;
   
   // which parameter to adjust
   // 0  dp_p;
   // 1  dp_i;
   // 2  dp_d;
   int adjust_index;
   float dp[3];
   
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  void MulitpleDP(float factor);
  
  void AdjustParams();
  
  void SetAdjustIndex(int index) ;
  
  bool  DPThreshold(float total);
  
  int CheckOperations() ;
};

#endif /* PID_H */
