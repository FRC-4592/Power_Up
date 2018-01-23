package org.usfirst.frc.team4592.robot.Util;

@SuppressWarnings("unused")
public class PID{
	private double Kp;
	private double Ki;
	private double Kd;
	private double error;
	private double error_sum;
	private double output;
	
	//Constructors
		//P Loop
			public PID(double Kp){
				this.Kp = Kp;
			}
		
		//PI Loop
			public PID(double Kp, double Ki){
				this.Kp = Kp;
				this.Ki = Ki;
			}
		
		//PID Loop
			public PID(double Kp, double Ki, double Kd){
				this.Kp = Kp;
				this.Ki = Ki;
				this.Kd = Kd;
			}

	//Caller Methods
		//P Loop
			public double getOutputP(double error){
				this.error = error;
				
				output = error * Kp;
				
				if(output > 1){
					return 1;
				}else if(output < -1){
					return -1;
				}
				
				return output;
			}
			
			public double getControlledOutputP(double error){
				this.error = error;
				
				output = error * Kp;
				
				if(output > 0.75){
					return 0.75;
				}else if(output < -0.75){
					return -0.75;
				}
				
				return output;
			}
			
		//PI Loop
			public double getOutputPI(double error){
				this.error = error;
				error_sum += error;
				
				output = (error * Kp) + (error_sum * Ki);
				
				if(output > 1){
					return 1;
				}else if(output < -1){
					return -1;
				}
				
				return output;
			}
	
	public void setError_Sum(double error_sum){
		this.error_sum = error_sum;
	}		
}