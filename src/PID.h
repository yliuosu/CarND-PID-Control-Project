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
   bool twiddle;
   double best_error;
   
   std::stack<Operations> stack_operations;
   
   // which parameter to adjust
   // 0  dp_p;
   // 1  dp_d;
   // 2  dp_i;
   int adjust_index;
   double dp[3];
   
   double best_kp;
   double best_kd;
   
   int  max_speed;
   int nummaxspeed;
   
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
  double TotalError();
  
  void MulitpleDP(double factor);
  
  void AdjustParams();
  
  void SetAdjustIndex(int index) ;
  
  int CheckOperations() ;
  
  bool Better();
  
  void UpdateSpeed(double speed) ;
  
  void Twiddle(int nStep);
  
  void DisplayPerformanceComparison(int stage);
  
  void DisplayAdjustmentArray();
  
  void DisplayCurrentBestParameter();
};

#endif /* PID_H */
