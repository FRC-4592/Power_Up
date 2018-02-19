package org.usfirst.frc.team4592.robot.Subsystems.ClawMech;

import org.usfirst.frc.team4592.robot.Constants;
import org.usfirst.frc.team4592.robot.Hardware;
import org.usfirst.frc.team4592.robot.Lib.SubsystemFramework;
import org.usfirst.frc.team4592.robot.Util.PID;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClawRotation extends SubsystemFramework{
	private TalonSRX clawRotationMotor;
	
	//PID
	private PID Claw_Rotation_PI;
	
	//Constants
	private double Average_Ticks_Per_Degree;
	private double goal_Ticks;
	private double goal_Ticks_Error;
	
	
	private ClawRotationState state = ClawRotationState.Stop; 
	
	/*public ClawRotation(TalonSRX clawRotationMotor, double Average_Ticks_Per_Degree, 
						double Claw_Rotation_Kf, double Claw_Rotation_Kp, 
						double Claw_Rotation_Ki, double Claw_Rotation_Kd){
		this.clawRotationMotor = clawRotationMotor;
		
		this.Claw_Rotation_Kf = Claw_Rotation_Kf;
		this.Claw_Rotation_Kp = Claw_Rotation_Kp;
		this.Claw_Rotation_Ki = Claw_Rotation_Ki;
		this.Claw_Rotation_Kd = Claw_Rotation_Kd;
	}*/
	
	public ClawRotation(TalonSRX clawRotationMotor, double Average_Ticks_Per_Degree, 
			double Claw_Rotation_Kp, double Claw_Rotation_Ki){
		this.clawRotationMotor = clawRotationMotor;

		this.Average_Ticks_Per_Degree = Average_Ticks_Per_Degree;
		
		this.Claw_Rotation_PI = new PID(Claw_Rotation_Kp, Claw_Rotation_Ki);
	}
	
	public enum ClawRotationState{
		Stop, TestUp, TestDown, TestPosition, IntakePosition, PlacePosition, HighScalePosition, StowPosition;  
	}
	
	public void setPosition(double angle) {
		goal_Ticks = angle * Average_Ticks_Per_Degree; 
		goal_Ticks_Error = goal_Ticks - clawRotationMotor.getSelectedSensorPosition(0);
		
		SmartDashboard.putNumber("Angle", angle);
		SmartDashboard.putNumber("Average_Ticks_Per_Degree", Average_Ticks_Per_Degree);
		SmartDashboard.putNumber("Goal_Ticks", goal_Ticks);
		SmartDashboard.putNumber("Goal_Ticks_Error", goal_Ticks_Error);
		
		clawRotationMotor.set(ControlMode.PercentOutput, Claw_Rotation_PI.getControlledOutputP(goal_Ticks_Error));
	}
	
	@Override
	public void update() {
		ClawRotationState newState = state;
		
		switch(state){
			case Stop:
				clawRotationMotor.set(ControlMode.PercentOutput, 0);
				
				if(Hardware.driverPad.getRawButton(Constants.DRIVETRAIN_LOWGEAR)) {
					//newState = ClawRotationState.TestUp;
					newState = ClawRotationState.TestPosition;
				}/*else if(Hardware.driverPad.getRawButton(Constants.DRIVETRAIN_HIGHGEAR)) {
					newState = ClawRotationState.TestDown;
				}*/
			break;
			
			case TestUp:
				clawRotationMotor.set(ControlMode.PercentOutput, 0.5);
				
				if(Hardware.driverPad.getRawButton(Constants.CLAW_INTAKE_POS)) {
					newState = ClawRotationState.Stop;
				}else if(Hardware.driverPad.getRawButton(Constants.DRIVETRAIN_HIGHGEAR)) {
					newState = ClawRotationState.TestDown;
				}
			break;
			
			case TestDown:
				clawRotationMotor.set(ControlMode.PercentOutput, -0.5);
				
				if(Hardware.driverPad.getRawButton(Constants.CLAW_INTAKE_POS)) {
					newState = ClawRotationState.Stop;
				}else if(Hardware.driverPad.getRawButton(Constants.DRIVETRAIN_LOWGEAR)) {
					newState = ClawRotationState.TestUp;
				}
			break;
			
			case TestPosition:
				setPosition(120);
				
				if(Hardware.driverPad.getRawButton(Constants.CLAW_INTAKE_POS)) {
					newState = ClawRotationState.Stop;
				}
			break;
			
			case IntakePosition:
				//clawRotationMotor.set(ControlMode.Position, arg1);
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
			
			default:
				newState = ClawRotationState.Stop;
			break;
		}
		
		if (newState != state) {
			state = newState;
		}

		outputToSmartDashboard();
	}
	
	
	
	
	@Override
	public void outputToSmartDashboard() {
		System.out.println("Claw Rotation: " + clawRotationMotor.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Claw Rotation", clawRotationMotor.getSelectedSensorPosition(0));
	}

	@Override
	public void setupSensors() {
		//Set Position To Zero When Started
		clawRotationMotor.setSelectedSensorPosition(0, 0, 10);
				
		//Absolute Allows Position To State The Same After Restart
		clawRotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
	}
}