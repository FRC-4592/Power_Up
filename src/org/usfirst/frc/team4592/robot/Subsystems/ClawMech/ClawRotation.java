package org.usfirst.frc.team4592.robot.Subsystems.ClawMech;

import org.usfirst.frc.team4592.robot.Constants;
import org.usfirst.frc.team4592.robot.Hardware;
import org.usfirst.frc.team4592.robot.Lib.SubsystemFramework;
import org.usfirst.frc.team4592.robot.Util.PID;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ClawRotation extends SubsystemFramework{
	private TalonSRX clawRotationMotor;
	
	//Constants
	private double Claw_Rotation_Kf;
	private double Claw_Rotation_Kp;
	private double Claw_Rotation_Ki;
	private double Claw_Rotation_Kd;
	
	private ClawRotationState state = ClawRotationState.StowPosition; 
	
	public ClawRotation(TalonSRX clawRotationMotor, double Average_Ticks_Per_Degree, 
						double Claw_Rotation_Kf, double Claw_Rotation_Kp, 
						double Claw_Rotation_Ki, double Claw_Rotation_Kd){
		this.clawRotationMotor = clawRotationMotor;
		
		this.Claw_Rotation_Kf = Claw_Rotation_Kf;
		this.Claw_Rotation_Kp = Claw_Rotation_Kp;
		this.Claw_Rotation_Ki = Claw_Rotation_Ki;
		this.Claw_Rotation_Kd = Claw_Rotation_Kd;
	}
	
	public enum ClawRotationState{
		IntakePosition, PlacePosition, HighScalePosition, StowPosition;  
	}
	
	@Override
	public void update() {
		ClawRotationState newState = state;
		
		switch(state){
			case IntakePosition:
				if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_SWITCH) 
					|| Hardware.driverPad.getRawButton(Constants.ELEVATOR_LOW_SCALE)
					|| Hardware.driverPad.getRawButton(Constants.ELEVATOR_MID_SCALE)){
					newState = ClawRotationState.PlacePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_HIGH_SCALE)){
					newState = ClawRotationState.HighScalePosition;
				}
	break;
			case PlacePosition:
				if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_INTAKE)){
					newState = ClawRotationState.IntakePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_HIGH_SCALE)){
					newState = ClawRotationState.HighScalePosition;
				}
	
	break;
			case HighScalePosition:
				if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_INTAKE)){
					newState = ClawRotationState.IntakePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_SWITCH) 
						  || Hardware.driverPad.getRawButton(Constants.ELEVATOR_LOW_SCALE)
						  || Hardware.driverPad.getRawButton(Constants.ELEVATOR_MID_SCALE)){
					newState = ClawRotationState.PlacePosition;
				}
	break;
			case StowPosition:
				
	break;
		}
	}
	
	@Override
	public void outputToSmartDashboard() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void setupSensors() {
		//Configure Motor Sensor
		//Absolute Allows Position To State The Same After Restart
		clawRotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
		
		//Set Position To Zero When Started
		clawRotationMotor.setSelectedSensorPosition(0, 0, 10);
		
		//Configure Range For Output
		clawRotationMotor.configNominalOutputForward(0, 10);
		clawRotationMotor.configNominalOutputReverse(0, 10);
		clawRotationMotor.configPeakOutputForward(1, 10);
		clawRotationMotor.configPeakOutputReverse(-1, 10);
		
		//Configure Allowable Loop Error Range
		clawRotationMotor.configAllowableClosedloopError(0, 0, 10);
		
		//Set Up PID Loop
		clawRotationMotor.config_kF(0, Claw_Rotation_Kf, 10);
		clawRotationMotor.config_kP(0, Claw_Rotation_Kp, 10);
		clawRotationMotor.config_kI(0, Claw_Rotation_Ki, 10);
		clawRotationMotor.config_kD(0, Claw_Rotation_Kd, 10);
	}
}