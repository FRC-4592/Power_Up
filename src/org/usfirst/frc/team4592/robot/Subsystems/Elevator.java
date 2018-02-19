package org.usfirst.frc.team4592.robot.Subsystems;

import org.usfirst.frc.team4592.robot.Constants;
import org.usfirst.frc.team4592.robot.Hardware;
import org.usfirst.frc.team4592.robot.Lib.SubsystemFramework;
import org.usfirst.frc.team4592.robot.Util.PID;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemFramework {
	// Hardware
	private TalonSRX elevatorMotor;

	// PID
	private PID Elevator_PI;

	// Constants
	private double Average_Ticks_Per_Inch;
	private double goal_Ticks;
	private double goal_Ticks_Error;

	public ElevatorState state = ElevatorState.Stop;

	public Elevator(TalonSRX elevatorMotor, double Average_Ticks_Per_Inch, double Elevator_Kp, double Elevator_Ki){
		this.elevatorMotor = elevatorMotor;

		this.Average_Ticks_Per_Inch = Average_Ticks_Per_Inch;

		this.Elevator_PI = new PID(Elevator_Kp, Elevator_Ki);
	}
	
	public Elevator(TalonSRX elevatorMotor) {
		this.elevatorMotor = elevatorMotor;
	}
	
	public enum ElevatorState {
		Stop, TestPosition, TestUp, TestDown, IntakePosition, SwitchPosition, LowScalePosition, MidScalePosition, HighScalePosition;
	}

	public void setPosition(double pos) {
		goal_Ticks = pos * Average_Ticks_Per_Inch;
		goal_Ticks_Error = goal_Ticks - elevatorMotor.getSelectedSensorPosition(0);

		elevatorMotor.set(ControlMode.PercentOutput, Elevator_PI.getOutputPI(goal_Ticks_Error));
	}

	@Override
	public void update() {
		ElevatorState newState = state;

		switch (state) {
			case Stop:
				elevatorMotor.set(ControlMode.PercentOutput, 0);
				
				if(Hardware.driverPad.getRawButton(Constants.DRIVETRAIN_LOWGEAR)) {
					//newState = ElevatorState.TestPosition;
					newState = ElevatorState.TestUp;
				}else if(Hardware.driverPad.getRawButton(Constants.DRIVETRAIN_HIGHGEAR)) {
					newState = ElevatorState.TestDown;
				}
			break;
			
			case TestUp:
				elevatorMotor.set(ControlMode.PercentOutput, -0.5);
			
				if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_STOW)) {
					newState = ElevatorState.Stop;
				}else if(Hardware.driverPad.getRawButton(Constants.DRIVETRAIN_HIGHGEAR)) {
					newState = ElevatorState.TestDown;
				}
			break;
			
			case TestDown:
				elevatorMotor.set(ControlMode.PercentOutput, 0.5);
			
				if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_STOW)) {
					newState = ElevatorState.Stop;
				}else if(Hardware.driverPad.getRawButton(Constants.DRIVETRAIN_LOWGEAR)) {
					//newState = ElevatorState.TestPosition;
					newState = ElevatorState.TestUp;
				}
			break;
			
			case TestPosition:
				setPosition(10);
				
				if(Hardware.driverPad.getRawButton(Constants.ELEVATOR_STOW)) {
					newState = ElevatorState.Stop;
				}else if(Hardware.driverPad.getRawButton(Constants.DRIVETRAIN_HIGHGEAR)) {
					newState = ElevatorState.TestDown;
				}
			break;
			
			case IntakePosition:
				//elevatorMotor.set(ControlMode.Position, setPosition(0));
	
				if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_SWITCH)) {
					newState = ElevatorState.SwitchPosition;
				} else if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_LOW_SCALE)) {
					newState = ElevatorState.LowScalePosition;
				} else if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_MID_SCALE)) {
					newState = ElevatorState.MidScalePosition;
				} else if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_HIGH_SCALE)) {
					newState = ElevatorState.HighScalePosition;
				}
			break;
			
			case SwitchPosition:
				//elevatorMotor.set(ControlMode.Position, setPosition(0));
	
				if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_INTAKE)) {
					newState = ElevatorState.IntakePosition;
				} else if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_LOW_SCALE)) {
					newState = ElevatorState.LowScalePosition;
				} else if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_MID_SCALE)) {
					newState = ElevatorState.MidScalePosition;
				} else if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_HIGH_SCALE)) {
					newState = ElevatorState.HighScalePosition;
				}
			break;
			
			case LowScalePosition:
				//elevatorMotor.set(ControlMode.Position, setPosition(0));
	
				if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_INTAKE)) {
					newState = ElevatorState.IntakePosition;
				} else if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_SWITCH)) {
					newState = ElevatorState.SwitchPosition;
				} else if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_MID_SCALE)) {
					newState = ElevatorState.MidScalePosition;
				} else if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_HIGH_SCALE)) {
					newState = ElevatorState.HighScalePosition;
				}
			break;
	
			case MidScalePosition:
				//elevatorMotor.set(ControlMode.Position, setPosition(0));
	
				if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_INTAKE)) {
					newState = ElevatorState.IntakePosition;
				} else if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_SWITCH)) {
					newState = ElevatorState.SwitchPosition;
				} else if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_LOW_SCALE)) {
					newState = ElevatorState.LowScalePosition;
				} else if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_HIGH_SCALE)) {
					newState = ElevatorState.HighScalePosition;
			}
			break;
	
			case HighScalePosition:
				//elevatorMotor.set(ControlMode.Position, setPosition(0));
	
				if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_INTAKE)) {
					newState = ElevatorState.IntakePosition;
				} else if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_SWITCH)) {
					newState = ElevatorState.SwitchPosition;
				} else if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_LOW_SCALE)) {
					newState = ElevatorState.LowScalePosition;
				} else if (Hardware.driverPad.getRawButton(Constants.ELEVATOR_MID_SCALE)) {
					newState = ElevatorState.MidScalePosition;
				}
			break;
	
			default:
				newState = ElevatorState.IntakePosition;
			break;
		}

		if (newState != state) {
			state = newState;
		}

		outputToSmartDashboard();
	}

	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putNumber("Elevator Position", (elevatorMotor.getSelectedSensorPosition(0)));
		SmartDashboard.putNumber("Elevator Motor", elevatorMotor.getMotorOutputPercent());
		
		System.out.println("Elevator Position: " + (elevatorMotor.getSelectedSensorPosition(0) / Average_Ticks_Per_Inch));
		System.out.println("Elevator Motor: " + elevatorMotor.getMotorOutputPercent());
	}

	@Override
	public void setupSensors() {
		// Set Position To 0 When The Robot Turns On
		elevatorMotor.setSelectedSensorPosition(0, 0, 10);
		
		//Setup Sensor To Absolute
		elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		
		//Set peak and nominal outputs
		elevatorMotor.configNominalOutputForward(0, 10);
		elevatorMotor.configNominalOutputReverse(0, 10);
		elevatorMotor.configPeakOutputForward(1, 10);
		elevatorMotor.configPeakOutputReverse(-1, 10);
		
		//Set allowable closed-loop error
		elevatorMotor.configAllowableClosedloopError(0, 0, 10);
		
		//Set closed loop gains in PID slot 0
		/*elevatorMotor.config_kF(0, Elevator_Kf, 10);
		elevatorMotor.config_kP(0, Elevator_Kp, 10);
		elevatorMotor.config_kI(0, Elevator_Ki, 10);
		elevatorMotor.config_kD(0, Elevator_Kd, 10);*/
	}
}