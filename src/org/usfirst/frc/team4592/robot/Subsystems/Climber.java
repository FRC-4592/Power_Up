package org.usfirst.frc.team4592.robot.Subsystems;

import org.usfirst.frc.team4592.robot.Constants;
import org.usfirst.frc.team4592.robot.Hardware;
import org.usfirst.frc.team4592.robot.Lib.SubsystemFramework;
import org.usfirst.frc.team4592.robot.Subsystems.Elevator.ElevatorState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;

public class Climber extends SubsystemFramework {
	private VictorSPX leftClimberMotor;
	private VictorSPX leftClimberMotor2;
	private VictorSPX rightClimberMotor;
	private VictorSPX rightClimberMotor2;
	private DigitalInput climberLimitSwitch;
	
	public static ClimberState state = ClimberState.Off;

	public Climber(VictorSPX rightClimberMotor, VictorSPX rightClimberMotor2, VictorSPX leftClimberMotor,
			VictorSPX leftClimberMotor2, DigitalInput climberLimitSwitch) {
		this.rightClimberMotor = rightClimberMotor;
		this.rightClimberMotor2 = rightClimberMotor2;
		this.leftClimberMotor = leftClimberMotor;
		this.leftClimberMotor2 = leftClimberMotor2;
		
		this.climberLimitSwitch = climberLimitSwitch;
	}

	public enum ClimberState {
		Off, Climb;
	}

	@Override
	public void update() {
		ClimberState newState = state;

		switch (state) {
		case Off:
			rightClimberMotor.set(ControlMode.PercentOutput, 0);
			rightClimberMotor2.set(ControlMode.PercentOutput, 0);
			leftClimberMotor.set(ControlMode.PercentOutput, 0);
			leftClimberMotor2.set(ControlMode.PercentOutput, 0);
			
			if (Hardware.driverPad.getRawButton(Constants.CLIMB) && Elevator.state == ElevatorState.ClimbDown) {
				newState = ClimberState.Climb;
			}
	break;

		case Climb:
			// Comp Bot
			rightClimberMotor.set(ControlMode.PercentOutput, -1);
			rightClimberMotor2.set(ControlMode.PercentOutput, 1);
			leftClimberMotor.set(ControlMode.PercentOutput, -1);
			leftClimberMotor2.set(ControlMode.PercentOutput, 1);
			
			// Practice Bot
			//rightClimberMotor.set(ControlMode.PercentOutput, 0.75);
			//rightClimberMotor2.set(ControlMode.PercentOutput, -0.75);
			//leftClimberMotor.set(ControlMode.PercentOutput, -0.75);
			//leftClimberMotor2.set(ControlMode.PercentOutput, -0.75);

			if (Hardware.driverPad.getRawButtonReleased(Constants.CLIMB)) {
				newState = ClimberState.Off;
			}
	break;

		default:
			newState = ClimberState.Off;
	break;
		}

		if (newState != state) {
			state = newState;
		}
	}

	@Override
	public void outputToSmartDashboard() {
	}

	@Override
	public void setupSensors() {
	}

}
