package org.usfirst.frc.team4592.robot.Subsystems;

import org.usfirst.frc.team4592.robot.Constants;
import org.usfirst.frc.team4592.robot.Hardware;
import org.usfirst.frc.team4592.robot.Lib.SubsystemFramework;
import org.usfirst.frc.team4592.robot.Util.PID;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemFramework{
	//Hardware
	private TalonSRX elevatorMotor;
	
	//Hardware
	private PID Elevator_Up_PI;
	private PID Elevator_Down_PI;
	
	//Constants
	//Need Setup PID correctly
	private double Average_Ticks_Per_Inch;
	private double goal_Ticks;
	private double goal_Ticks_Error;
	
	public ElevatorState state = ElevatorState.IntakePosition;
	
	public Elevator(TalonSRX elevatorMotor, double Average_Ticks_Per_Inch, 
					double Elevator_Up_Kp, double Elevator_Up_Ki, 
					double Elevator_Down_Kp, double Elevator_Down_Ki){
		this.elevatorMotor = elevatorMotor;
		
		
		this.Average_Ticks_Per_Inch = Average_Ticks_Per_Inch;
		
		this.Elevator_Up_PI = new PID(Elevator_Up_Kp, Elevator_Up_Ki);
		this.Elevator_Down_PI = new PID(Elevator_Down_Kp, Elevator_Down_Ki);
		
	}
	
	public enum ElevatorState{
		IntakePosition, SwitchPosition, LowScalePosition, MidScalePosition, HighScalePosition;
	}
	
	@Override
	public void update() {
		ElevatorState newState = state;
		
		switch(state){
			case IntakePosition:
				elevatorMotor.set(ControlMode.PercentOutput, Elevator_Down_PI.getOutputP(goal_Ticks_Error));
				
				if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_SWITCH)){
					newState = ElevatorState.SwitchPosition;
				}else if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_LOW_SCALE)){
					newState = ElevatorState.LowScalePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_MID_SCALE)){
					newState = ElevatorState.MidScalePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_HIGH_SCALE)){
					newState = ElevatorState.HighScalePosition;
				}
	break;
			case SwitchPosition:
				if(state == ElevatorState.IntakePosition){
					elevatorMotor.set(ControlMode.PercentOutput, Elevator_Up_PI.getOutputP(goal_Ticks_Error));
				}else {
					elevatorMotor.set(ControlMode.PercentOutput, Elevator_Down_PI.getOutputP(goal_Ticks_Error));
				}
				
				if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_INTAKE)){
					newState = ElevatorState.IntakePosition; 
				}else if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_LOW_SCALE)){
					newState = ElevatorState.LowScalePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_MID_SCALE)){
					newState = ElevatorState.MidScalePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_HIGH_SCALE)){
					newState = ElevatorState.HighScalePosition;
				}
	break;			
			case LowScalePosition:
				if(state == ElevatorState.IntakePosition || state == ElevatorState.SwitchPosition){
					elevatorMotor.set(ControlMode.PercentOutput, Elevator_Up_PI.getOutputP(goal_Ticks_Error));
				}else{
					elevatorMotor.set(ControlMode.PercentOutput, Elevator_Down_PI.getOutputP(goal_Ticks_Error));
				}
				
				if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_INTAKE)){
					newState = ElevatorState.IntakePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_SWITCH)){
					newState = ElevatorState.SwitchPosition;
				}else if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_MID_SCALE)){
					newState = ElevatorState.MidScalePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_HIGH_SCALE)){
					newState = ElevatorState.HighScalePosition;
				}
	break;
			case MidScalePosition:
				if(state != ElevatorState.HighScalePosition){
					elevatorMotor.set(ControlMode.PercentOutput, Elevator_Up_PI.getOutputP(goal_Ticks_Error));
				}else{
					elevatorMotor.set(ControlMode.PercentOutput, Elevator_Down_PI.getOutputP(goal_Ticks_Error));
				}
				
				if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_INTAKE)){
					newState = ElevatorState.IntakePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_SWITCH)){
					newState = ElevatorState.SwitchPosition;
				}else if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_LOW_SCALE)){
					newState = ElevatorState.LowScalePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_HIGH_SCALE)){
					newState = ElevatorState.HighScalePosition;
				}
	break;	
			case HighScalePosition:
				elevatorMotor.set(ControlMode.PercentOutput, Elevator_Up_PI.getOutputP(goal_Ticks_Error));
				
				if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_INTAKE)){
					newState = ElevatorState.IntakePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_SWITCH)){
					newState = ElevatorState.SwitchPosition;
				}else if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_LOW_SCALE)){
					newState = ElevatorState.LowScalePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_MID_SCALE)){
					newState = ElevatorState.MidScalePosition;
				}
	break;
			default:
				newState = ElevatorState.IntakePosition;
	break;
		}
		
		if(newState != state){
			state = newState;
		}
		
		outputToSmartDashboard();
	}

	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putNumber("Elevator Position", (elevatorMotor.getSelectedSensorPosition(0)/Average_Ticks_Per_Inch));
		
		System.out.println("Elevator Position" + (elevatorMotor.getSelectedSensorPosition(0)/Average_Ticks_Per_Inch));
	}

	@Override
	public void setupSensors() {
		//Setup Sensor To Absolute
		elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
		
		//Set Position To 0 When The Robot Turns On
		elevatorMotor.setSelectedSensorPosition(0, 0, 10);
	}

}
