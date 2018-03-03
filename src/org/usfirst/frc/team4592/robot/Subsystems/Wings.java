package org.usfirst.frc.team4592.robot.Subsystems;

import org.usfirst.frc.team4592.robot.Constants;
import org.usfirst.frc.team4592.robot.Hardware;
import org.usfirst.frc.team4592.robot.Lib.SubsystemFramework;
import org.usfirst.frc.team4592.robot.Subsystems.Climber.ClimberState;
import org.usfirst.frc.team4592.robot.Util.doubleSolenoid;

public class Wings extends SubsystemFramework{
	private doubleSolenoid wingRelease;
	
	private WingsState state = WingsState.Loaded;
	
	public Wings(doubleSolenoid wingRelease) {
		this.wingRelease = wingRelease;
	}
	
	public enum WingsState{
		Loaded, Released;
	}
	
	@Override
	public void update() {
		WingsState newState = state;
		
		switch(state) {
			case Loaded:
				wingRelease.open();
				
				if(Hardware.operatorPad.getRawButton(Constants.Button9)) {
					newState = WingsState.Released;
				}
	break;
			case Released:
				wingRelease.close();
				
				if(Hardware.operatorPad.getRawButtonReleased(Constants.Button9)) {
					newState = WingsState.Loaded;
				}
	break;
	
			default:
				newState = WingsState.Loaded;
	break;
			}

		if (newState != state) {
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
