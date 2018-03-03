package org.usfirst.frc.team4592.robot.Subsystems.ClawMech;

import org.usfirst.frc.team4592.robot.Constants;
import org.usfirst.frc.team4592.robot.Hardware;
import org.usfirst.frc.team4592.robot.Lib.SubsystemFramework;
import org.usfirst.frc.team4592.robot.Subsystems.Elevator;
import org.usfirst.frc.team4592.robot.Subsystems.ClawMech.ClawWheels.ClawWheelsState;
import org.usfirst.frc.team4592.robot.Subsystems.Elevator.ElevatorState;
import org.usfirst.frc.team4592.robot.Util.PID;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClawRotation extends SubsystemFramework{
	private static TalonSRX clawRotationMotor;
	
	//PID
	private PID Claw_Rotation_PI;
	
	//Constants
	private static double Average_Ticks_Per_Degree;
	private double goal_Ticks;
	private double goal_Ticks_Error;
	private int counter = 0;
	private int counter2 = 0;
	private double ClawRotation_Kf;
	private double ClawRotation_Kp;
	private double ClawRotation_Ki;
	private double ClawRotation_Kd;
	
	public static ClawRotationState state = ClawRotationState.Stop; 
	
	public ClawRotation(TalonSRX clawRotationMotor, double Average_Ticks_Per_Degree, 
						double ClawRotation_Kf, double ClawRotation_Kp, 
						double ClawRotation_Ki, double ClawRotation_Kd){
		this.clawRotationMotor = clawRotationMotor;
		
		this.Average_Ticks_Per_Degree = Average_Ticks_Per_Degree;
		
		this.ClawRotation_Kf = ClawRotation_Kf;
		this.ClawRotation_Kp = ClawRotation_Kp;
		this.ClawRotation_Ki = ClawRotation_Ki;
		this.ClawRotation_Kd = ClawRotation_Kd;
	}
	
	public ClawRotation(TalonSRX clawRotationMotor, double Average_Ticks_Per_Degree, 
			double Claw_Rotation_Kp, double Claw_Rotation_Ki){
		this.clawRotationMotor = clawRotationMotor;

		this.Average_Ticks_Per_Degree = Average_Ticks_Per_Degree;
		
		this.Claw_Rotation_PI = new PID(Claw_Rotation_Kp, Claw_Rotation_Ki);
	}
	
	public enum ClawRotationState{
		Stop, TestUp, TestDown, TestPosition, IntakePosition, PlacePosition, ScalePosition, StowPosition;  
	}		
	
	public double setPosition(double angle) {
			if(angle > 133) {
				angle = 133;
			}
		return angle * Average_Ticks_Per_Degree; 
	}
	
	public static boolean testSafeAngle(double Safe_Angle) {
		System.out.println("I'm being called");
		SmartDashboard.putBoolean("I'm being called", true);
		return (Safe_Angle * Average_Ticks_Per_Degree) >= clawRotationMotor.getSelectedSensorPosition(0);
	}
	
	@Override
	public void update() {
		ClawRotationState newState = state;
		
		switch(state){
			case Stop:
				clawRotationMotor.set(ControlMode.PercentOutput, 0);
				
				SmartDashboard.putString("Claw Rotation Stage", "Stop");
				
				if(Hardware.driverPad.getRawButton(Constants.SWITCH)
						|| Hardware.driverPad.getRawButton(Constants.SCALE)) {
					newState = ClawRotationState.PlacePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.HIGH_SCALE)) {
					newState = ClawRotationState.ScalePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.INTAKE)) {
					newState = ClawRotationState.IntakePosition;
				}else if(Hardware.operatorPad.getRawButton(Constants.Button1)
						|| Hardware.operatorPad.getRawButton(Constants.Button6)) {
					newState = ClawRotationState.StowPosition;
				}
	break;					
			case IntakePosition:
				clawRotationMotor.set(ControlMode.Position, setPosition(85));
				
				if(Hardware.driverPad.getRawButton(Constants.SWITCH) 
					|| Hardware.driverPad.getRawButton(Constants.SCALE)){
					newState = ClawRotationState.PlacePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.HIGH_SCALE)){
					newState = ClawRotationState.ScalePosition;
				}else if(Hardware.operatorPad.getRawButton(Constants.Button1)) {
					newState = ClawRotationState.StowPosition;
				}
	break;		
			case PlacePosition:
				if(Elevator.testSafePosition(Constants.Safe_Position)) {					
					clawRotationMotor.set(ControlMode.Position, setPosition(100));
				}
				
				if(Hardware.driverPad.getRawButton(Constants.INTAKE)){
					newState = ClawRotationState.IntakePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.HIGH_SCALE)){
					newState = ClawRotationState.ScalePosition;
				}else if(Hardware.operatorPad.getRawButton(Constants.Button5)) {
					newState = ClawRotationState.Stop;
				}else if(Hardware.operatorPad.getRawButton(Constants.Button1)) {
					newState = ClawRotationState.StowPosition;
				}
	
	break;		
			case ScalePosition:
				if(Elevator.testSafePosition(Constants.Safe_Position)) {	
					
					clawRotationMotor.set(ControlMode.Position, setPosition(133));
				}
				
				if(Hardware.driverPad.getRawButton(Constants.INTAKE)){
					newState = ClawRotationState.IntakePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.SWITCH) 
						  || Hardware.driverPad.getRawButton(Constants.SCALE)){
					newState = ClawRotationState.PlacePosition;
				}else if(Hardware.operatorPad.getRawButton(Constants.Button5)) {
					newState = ClawRotationState.Stop;
				}else if(Hardware.operatorPad.getRawButton(Constants.Button1)) {
					newState = ClawRotationState.StowPosition;
				}
	break;		
			case StowPosition:
				if(Elevator.testSafePosition(Constants.Safe_Position)) {
					clawRotationMotor.set(ControlMode.Position, setPosition(5));
				}
				
				if(Hardware.driverPad.getRawButton(Constants.SWITCH)
						|| Hardware.driverPad.getRawButton(Constants.SCALE)) {
					newState = ClawRotationState.PlacePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.HIGH_SCALE)) {
					newState = ClawRotationState.ScalePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.INTAKE)) {
					newState = ClawRotationState.IntakePosition;
				}else if(Hardware.operatorPad.getRawButton(Constants.Button5)) {
					newState = ClawRotationState.Stop;
				}
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
		System.out.println("Claw Rotation Goal: " + setPosition(10));
		
		SmartDashboard.putNumber("Claw Rotation", clawRotationMotor.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Claw Rotation Goal", setPosition(10));
	}

	@Override
	public void setupSensors() {
		SmartDashboard.putBoolean("I'm being called", false);
		//Set Position To Zero When Started
		clawRotationMotor.setSelectedSensorPosition(0, 0, 10);
				
		//Absolute Allows Position To State The Same After Restart
		clawRotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
		
		//Set peak and nominal outputs
		clawRotationMotor.configNominalOutputForward(0, 10);
		clawRotationMotor.configNominalOutputReverse(0, 10);
		clawRotationMotor.configPeakOutputForward(0.5, 10);
		clawRotationMotor.configPeakOutputReverse(-0.5, 10);
		
		//Set allowable closed-loop error
		clawRotationMotor.configAllowableClosedloopError(0, 0, 10);
		
		//Set closed loop gains in PID slot 0
		clawRotationMotor.config_kF(0, ClawRotation_Kf, 10);
		clawRotationMotor.config_kP(0, ClawRotation_Kp, 10);
		clawRotationMotor.config_kI(0, ClawRotation_Ki, 10);
		clawRotationMotor.config_kD(0, ClawRotation_Kd, 10);
	}
}