package org.usfirst.frc.team4592.robot.Util;

public class bangBang{
	private double goalRPM;
	private double lowerOutputLimit;
	private double higherOutputLimit;
	private double Max_RPM;
	private double gearRatio;
	private boolean useAccurate = false;
	
	public bangBang(double goalRPM){
		this.goalRPM = goalRPM;
		this.lowerOutputLimit = -0.25;
		this.higherOutputLimit = 1;
	}
	
	public bangBang(double goalRPM, double Max_RPM, double gearRatio){
		this.goalRPM = goalRPM;
		this.Max_RPM = Max_RPM;
		this.gearRatio = gearRatio;
		this.useAccurate = true;
	}
	
	public double getOutput(double currentRPM){
		if(currentRPM < goalRPM){
			return higherOutputLimit;
		}else if(currentRPM >= goalRPM){
			if(!useAccurate){
				return lowerOutputLimit;
			}else{
				return ((goalRPM*gearRatio)/(Max_RPM));
			}
		}else{
			return 0;
		}
	}
}
