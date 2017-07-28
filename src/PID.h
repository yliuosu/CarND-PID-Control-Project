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
   
   // time steps
   int msgcount;
   
   // whem twiddle the parameter
   // flag to show whether it is the first run
   bool first_run;
   bool twiddle;
   int max_twiddle_steps;
   
   // record the smallest sum square cte 
   // of multiple runs
   double best_error;
   
   // stack to record previous operation on the parameters
   std::stack<Operations> stack_operations;
   
   // which parameter to adjust
   // 0  dp_p to adjust  Kp;
   // 1  dp_d to adjust  Kd;
   // 2  dp_i to adjust  Ki;
   int adjust_index;
   double dp[3];
   
   // record the kp and kd which delivers the best performance
   double best_kp;
   double best_kd;
   
   // maximum speed is the real speed X 10 and take the interger part
   int  max_speed;
   // number of maximum speed 
   int nummaxspeed;
   
   // record the maximum speed and number 
   // in multiple runs 
   int best_max_speed;
   int best_nummaxspeed;
   
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

 // change the range of the adjustment array 
  void MulitpleDP(double factor);
  
  void AdjustParams();
  
  void SetAdjustIndex(int index) ;
  
  int CheckOperations() ;
  
  // check if this run has better results
  bool Better();
  
  void UpdateSpeed(double speed) ;
  
  void Twiddle( bool & reset);
  
  void SetTwiddleFlag(bool flag);
  
  bool  GetTwiddleFlag();
  
  void DisplayPerformanceComparison(int stage);
  
  void DisplayAdjustmentArray();
  
  void DisplayCurrentBestParameter();
  
  void SetMaxTwiddleSteps(int steps) ;
};

#endif /* PID_H */
