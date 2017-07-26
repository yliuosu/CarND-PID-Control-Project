#ifndef PID_H
#define PID_H

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
   
   float best_error;
   
   float dp[3];
   
   bool better;
   
   // 0  dp_p;
   // 1  dp_i;
   // 2  dp_d;

   int adjust_index;
   
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
  
  void ADDP();
  
};

#endif /* PID_H */
