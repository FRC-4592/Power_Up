package org.usfirst.frc.team4592.robot.Subsystems;

import org.usfirst.frc.team4592.robot.Constants;
import org.usfirst.frc.team4592.robot.Hardware;
import org.usfirst.frc.team4592.robot.Lib.SubsystemFramework;
import org.usfirst.frc.team4592.robot.Subsystems.ClawMech.ClawRotation;
import org.usfirst.frc.team4592.robot.Subsystems.ClawMech.ClawRotation.ClawRotationState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemFramework {
	// Hardware
	private static TalonSRX elevatorMotor;

	// Constants
	private static double Average_Ticks_Per_Inch;
	private double Elevator_Kf;
	private double Elevator_Kp;
	private double Elevator_Ki;
	private double Elevator_Kd;
	
	public static ElevatorState state = ElevatorState.StartPosition;

	public Elevator(TalonSRX elevatorMotor, double Average_Ticks_Per_Inch, 
						double Elevator_Kf, double Elevator_Kp, 
						double Elevator_Ki, double Elevator_Kd){
		this.elevatorMotor = elevatorMotor;

		this.Average_Ticks_Per_Inch = Average_Ticks_Per_Inch;

		this.Elevator_Kf = Elevator_Kf;
		this.Elevator_Kp = Elevator_Kp;
		this.Elevator_Ki = Elevator_Ki;
		this.Elevator_Kd = Elevator_Kd;
	}
	
	public Elevator(TalonSRX elevatorMotor) {
		this.elevatorMotor = elevatorMotor;
	}
	
	public enum ElevatorState {
		Stop, Up, Down, StowPosition, IntakePosition, SwitchPosition, ScalePosition, AutoScalePosition, StartPosition, ClimbUp, ClimbDown;
	}

	public double setPosition(double pos) {
		return pos * Average_Ticks_Per_Inch;
	}
	
	public static boolean testSafePosition(double Safe_Position) {
		return (Safe_Position * Average_Ticks_Per_Inch) < elevatorMotor.getSelectedSensorPosition(0);
	}
	
	public static boolean testSafeHighPosition (double Safe_High_Position) {
		return (Safe_High_Position * Average_Ticks_Per_Inch) >= elevatorMotor.getSelectedSensorPosition(0);
	}

	@Override
	public void update() {
		ElevatorState newState = state;

		switch (state) {
			case StartPosition:
				elevatorMotor.set(ControlMode.Position, setPosition(0));
				
				ClawRotation.state = ClawRotationState.StartPosition;
				
				if(Hardware.driverPad.getRawButton(Constants.INTAKE)) {
					newState = ElevatorState.IntakePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.SWITCH)) {
					newState = ElevatorState.SwitchPosition;
				}else if(Hardware.driverPad.getRawButton(Constants.HIGH_SCALE)) {
					newState = ElevatorState.ScalePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.CLIMB_UP)) {
					newState = ElevatorState.ClimbUp;
				}
	break;
			case StowPosition:
				setupDown();
				
				elevatorMotor.set(ControlMode.Position, setPosition(4));
				
				ClawRotation.state = ClawRotationState.StowPosition;
				
				if(Hardware.driverPad.getRawButton(Constants.INTAKE)) {
					newState = ElevatorState.IntakePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.SWITCH)) {
					newState = ElevatorState.SwitchPosition;
				}else if(Hardware.driverPad.getRawButton(Constants.HIGH_SCALE)) {
					newState = ElevatorState.ScalePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.CLIMB_UP)) {
					newState = ElevatorState.ClimbUp;
				}
	break;
			case IntakePosition:
				setupDown();
				
				if(ClawRotation.testSafeAngle(Constants.Safe_Angle)) {
					elevatorMotor.set(ControlMode.Position, setPosition(-13));
				}
				
				ClawRotation.state = ClawRotationState.IntakePosition;
				
				if(Hardware.driverPad.getRawButton(Constants.SWITCH)) {
					newState = ElevatorState.SwitchPosition;
				}else if(Hardware.driverPad.getRawButton(Constants.HIGH_SCALE)) {
					newState = ElevatorState.ScalePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.CLIMB_UP)) {
					newState = ElevatorState.ClimbUp;
				}
	break;
			case SwitchPosition:
				setupUp();
				
				elevatorMotor.set(ControlMode.Position, setPosition(10));
	
				ClawRotation.state = ClawRotationState.PlacePosition;
				
				if(Hardware.driverPad.getRawButton(Constants.INTAKE)) {
					newState = ElevatorState.IntakePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.HIGH_SCALE)) {
					newState = ElevatorState.ScalePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.CLIMB_UP)) {
					newState = ElevatorState.ClimbUp;
				}
	break;
			case ScalePosition:
				setupUp();
				
				elevatorMotor.set(ControlMode.Position, setPosition(46));
				
				ClawRotation.state = ClawRotationState.HighScalePosition;
				
				if(Hardware.driverPad.getRawButtonReleased(Constants.HIGH_SCALE)) {
					newState = ElevatorState.StowPosition;
				}
	break;
			case AutoScalePosition:
				setupUp();
				
				elevatorMotor.set(ControlMode.Position, setPosition(46));
				
				ClawRotation.state = ClawRotationState.HighScalePosition;
	break;
			case ClimbUp:
				setupUp();
				
				elevatorMotor.set(ControlMode.Position, setPosition(35));
				
				ClawRotation.state = ClawRotationState.Stop;
				
				if(Hardware.driverPad.getRawButton(Constants.CLIMB_DOWN)) {
					newState = ElevatorState.ClimbDown;
				}else if(Hardware.driverPad.getRawButton(Constants.INTAKE)) {
					newState = ElevatorState.IntakePosition;
				}else if(Hardware.driverPad.getRawButton(Constants.HIGH_SCALE)) {
					newState = ElevatorState.ScalePosition;
				}
	break;
			case ClimbDown:
				setupDown();
				
				elevatorMotor.set(ControlMode.Position, 15);
				
				ClawRotation.state = ClawRotationState.Stop;
				
				if(Hardware.driverPad.getRawButton(Constants.CLIMB_UP)) {
					newState = ElevatorState.ClimbUp;
				}
	break;
			case Stop:
				elevatorMotor.set(ControlMode.PercentOutput, 0);
				
				if(Hardware.operatorPad.getRawButton(Constants.Button2)) {
					newState = ElevatorState.Up;
				}else if(Hardware.operatorPad.getRawButton(Constants.Button3)) {
					newState = ElevatorState.Down;
				}
	break;		
			case Up:
				elevatorMotor.set(ControlMode.PercentOutput, 1);
				
				if(Hardware.operatorPad.getRawButton(Constants.Button1)) {
					newState = ElevatorState.Stop;
				}else if(Hardware.operatorPad.getRawButton(Constants.Button3)) {
					newState = ElevatorState.Down;
				}
	break;
			case Down:				
				elevatorMotor.set(ControlMode.PercentOutput, -1);
				
				if(Hardware.operatorPad.getRawButton(Constants.Button1)) {
					newState = ElevatorState.Stop;
				}else if(Hardware.operatorPad.getRawButton(Constants.Button2)) {
					newState = ElevatorState.Up;
				}
	break;
			default:
				newState = ElevatorState.Stop;
	break;
		}

		if (newState != state) {
			state = newState;
		}

		outputToSmartDashboard();
	}

	public void setupUp() {
		elevatorMotor.configNominalOutputForward(0, 10);
		elevatorMotor.configNominalOutputReverse(0, 10);
		elevatorMotor.configPeakOutputForward(1, 10);
		elevatorMotor.configPeakOutputReverse(-1, 10);	
	}
	
	public void setupDown() {
		elevatorMotor.configNominalOutputForward(0, 10);
		elevatorMotor.configNominalOutputReverse(0, 10);
		elevatorMotor.configPeakOutputForward(0.5, 10);
		elevatorMotor.configPeakOutputReverse(-0.5, 10);
	}
	
	@Override
	public void outputToSmartDashboard() {
		//SmartDashboard.putNumber("Elevator Position", (elevatorMotor.getSelectedSensorPosition(0)));
		//SmartDashboard.putNumber("Elevator Distance", elevatorMotor.getSelectedSensorPosition(0) / Average_Ticks_Per_Inch);
	}

	@Override
	public void setupSensors() {
		// Set Position To 0 When The Robot Turns On
		elevatorMotor.setSelectedSensorPosition(0, 0, 10);
				
		//Setup Sensor To Absolute
		elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
		
		//Set peak and nominal outputs
		elevatorMotor.configNominalOutputForward(0, 10);
		elevatorMotor.configNominalOutputReverse(0, 10);
		elevatorMotor.configPeakOutputForward(1, 10);
		elevatorMotor.configPeakOutputReverse(-1, 10);
		
		//Set allowable closed-loop error
		elevatorMotor.configAllowableClosedloopError(0, 0, 10);
		
		//Set closed loop gains in PID slot 0
		elevatorMotor.config_kF(0, Elevator_Kf, 10);
		elevatorMotor.config_kP(0, Elevator_Kp, 10);
		elevatorMotor.config_kI(0, Elevator_Ki, 10);
		elevatorMotor.config_kD(0, Elevator_Kd, 10);
	}
}