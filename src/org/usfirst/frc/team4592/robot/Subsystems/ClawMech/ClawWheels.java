package org.usfirst.frc.team4592.robot.Subsystems.ClawMech;

import org.usfirst.frc.team4592.robot.Constants;
import org.usfirst.frc.team4592.robot.Hardware;
import org.usfirst.frc.team4592.robot.Lib.SubsystemFramework;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;

public class ClawWheels extends SubsystemFramework{
	//Maybe VictorSPX
	private VictorSP clawRightMotor;
	private VictorSP clawLeftMotor;
	private SpeedControllerGroup wheels;
	
	private int counter = 0;
	
	public static ClawWheelsState state = ClawWheelsState.Off;
	
	public ClawWheels(VictorSP clawRightMotor, VictorSP clawLeftMotor){
		this.clawRightMotor = clawRightMotor;
		this.clawLeftMotor = clawLeftMotor;
		this.clawRightMotor.setInverted(true);
		
		this.wheels = new SpeedControllerGroup(this.clawRightMotor, this.clawLeftMotor);
	}
	
	public enum ClawWheelsState{
		Off, Intake, ClawRotation, Spit;
	}
	
	@Override
	public void update() {
		ClawWheelsState newState = state;
		
		switch(state){
			case Off:
				wheels.set(0);
				
				if(Hardware.driverPad.getRawAxis(Constants.CLAW_INTAKE) > 0.2){
					newState = ClawWheelsState.Intake;
				}else if(Hardware.driverPad.getRawAxis(Constants.CLAW_SPIT) > 0.2){
					newState = ClawWheelsState.Spit;
				}/*if(Hardware.driverPad.getRawButton(Constants.SWITCH)
						|| Hardware.driverPad.getRawButton(Constants.SCALE)
						|| Hardware.driverPad.getRawButton(Constants.HIGH_SCALE)
						|| Hardware.operatorPad.getRawButton(Constants.Button1)){
					newState = ClawWheelsState.ClawRotation;
				}*/
	break;
			case Intake:
				wheels.set(-0.75);
				
				if(Hardware.driverPad.getRawAxis(Constants.CLAW_INTAKE) <= 0.2) {
					newState = ClawWheelsState.Off;
				}
	break;
		
			case ClawRotation:
				wheels.set(-0.5);
				
				if(counter > 200) {
					newState = ClawWheelsState.Off;
				}
				
				counter++;
	break;
	
			case Spit:
				wheels.set(1);
				
				if(Hardware.driverPad.getRawAxis(Constants.CLAW_SPIT) <= 0.2) {
					newState = ClawWheelsState.Off;
				}
	break;
			default:
				newState = ClawWheelsState.Off;
	break;
		}
		
		if(newState != state) {
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