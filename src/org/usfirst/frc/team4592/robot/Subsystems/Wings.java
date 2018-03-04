package org.usfirst.frc.team4592.robot.Subsystems;

import org.usfirst.frc.team4592.robot.Constants;
import org.usfirst.frc.team4592.robot.Hardware;
import org.usfirst.frc.team4592.robot.Lib.SubsystemFramework;
import org.usfirst.frc.team4592.robot.Util.doubleSolenoid;

import edu.wpi.first.wpilibj.DigitalInput;

public class Wings extends SubsystemFramework{
	private doubleSolenoid wingRelease;
	
	private int counter = 0;
	private int counter2 = 0;
	private boolean cycle = false;
	
	private DigitalInput climberLimitSwitch;
	
	private WingsState state = WingsState.Loaded;
	
	public Wings(doubleSolenoid wingRelease, DigitalInput climberLimitSwitch) {
		this.wingRelease = wingRelease;
		this.climberLimitSwitch = climberLimitSwitch;
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
				
				if(cycle && counter >= 40) {
					newState = WingsState.Released;
					counter2++;
					counter = 0;
				}
				
				if(Hardware.driverPad.getRawButton(Constants.WING_DEPLOY) && !cycle && !climberLimitSwitch.get()) {
					cycle = true;
					newState = WingsState.Released;
				}
				
				
				
				counter++;
	break;
			case Released:
				wingRelease.close();
				
				 if(cycle && counter >= 40) {
						newState = WingsState.Loaded;
						counter = 0;
				}
				
				if(Hardware.driverPad.getRawButtonReleased(Constants.WING_DEPLOY) || counter2 >= 10) {
					newState = WingsState.Loaded;
					cycle = false;
				}
				
				counter++;
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
