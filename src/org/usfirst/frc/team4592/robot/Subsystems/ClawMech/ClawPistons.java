package org.usfirst.frc.team4592.robot.Subsystems.ClawMech;

import org.usfirst.frc.team4592.robot.Constants;
import org.usfirst.frc.team4592.robot.Hardware;
import org.usfirst.frc.team4592.robot.Lib.SubsystemFramework;
import org.usfirst.frc.team4592.robot.Util.doubleSolenoid;

public class ClawPistons extends SubsystemFramework{
	private doubleSolenoid clawPistons;
	
	public static ClawPistonsState state = ClawPistonsState.Close;
	
	public ClawPistons(doubleSolenoid clawPistons) {
		this.clawPistons = clawPistons;
	}
	
	public enum ClawPistonsState{
		Open, Close;
	}
	
	@Override
	public void update() {
		ClawPistonsState newState = state;
		
		switch(state) {
			case Open:
				clawPistons.open();
				
				if(Hardware.driverPad.getRawButtonReleased(Constants.CLAW_PISTON)) {
					newState = ClawPistonsState.Close;
				}
	break;
			case Close:
				clawPistons.close();
				
				if(Hardware.driverPad.getRawButton(Constants.CLAW_PISTON)) {
					newState = ClawPistonsState.Open;
				}
	break;
			default:
				newState = ClawPistonsState.Close;
	break;
		}
		
		if(state != newState) {
			state = newState;
		}
	}

	@Override
	public void outputToSmartDashboard() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void setupSensors() {
		// TODO Auto-generated method stub
		
	}

}
