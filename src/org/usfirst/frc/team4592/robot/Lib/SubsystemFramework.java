package org.usfirst.frc.team4592.robot.Lib;

import org.usfirst.frc.team4592.robot.Lib.Loop.Loopable;

public abstract class SubsystemFramework implements Loopable{
	public abstract void outputToSmartDashboard();
	public abstract void setupSensors();
}