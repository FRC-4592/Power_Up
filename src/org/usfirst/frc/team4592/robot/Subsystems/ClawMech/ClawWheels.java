package org.usfirst.frc.team4592.robot.Subsystems.ClawMech;

import org.usfirst.frc.team4592.robot.Constants;
import org.usfirst.frc.team4592.robot.Hardware;
import org.usfirst.frc.team4592.robot.Lib.SubsystemFramework;
import org.usfirst.frc.team4592.robot.Subsystems.ClawMech.ClawPistons.ClawPistonsState;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;

public class ClawWheels extends SubsystemFramework{
	//Double Motor
	private VictorSP clawRightMotor;
	private VictorSP clawLeftMotor;
	private SpeedControllerGroup wheels;
	
	//Single Motor
	private VictorSP clawWheelsMotor;
	
	private int counter = 0;
	
	public static ClawWheelsState state = ClawWheelsState.Off;
	
	public ClawWheels(VictorSP clawRightMotor, VictorSP clawLeftMotor){
		this.clawRightMotor = clawRightMotor;
		this.clawLeftMotor = clawLeftMotor;
		this.clawRightMotor.setInverted(true);
		
		this.wheels = new SpeedControllerGroup(this.clawRightMotor, this.clawLeftMotor);
	}
	
	public ClawWheels (VictorSP clawWheelsMotor) {
		this.clawWheelsMotor = clawWheelsMotor;
	}
	
	public enum ClawWheelsState{
		Off, Intake, Spit;
	}
	
	@Override
	public void update() {
		ClawWheelsState newState = state;
		
		switch(state){
			case Off:
				clawWheelsMotor.set(0);
				
				if(Hardware.driverPad.getRawAxis(Constants.CLAW_INTAKE) > 0.2){
					newState = ClawWheelsState.Intake;
				}else if(Hardware.driverPad.getRawAxis(Constants.CLAW_SPIT) > 0.2){
					newState = ClawWheelsState.Spit;
				}
	break;
			case Intake:
				clawWheelsMotor.set(1);
				
				if(Hardware.driverPad.getRawAxis(Constants.CLAW_INTAKE) <= 0.2) {
					newState = ClawWheelsState.Off;
				}
	break;	
			case Spit:
				clawWheelsMotor.set(-1);
				
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