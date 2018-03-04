package org.usfirst.frc.team4592.robot.Lib;

import org.usfirst.frc.team4592.robot.Lib.Loop.Loopable;

public abstract class AutoFramework implements Loopable{
	public int counter = 0;
	public abstract void outputToSmartDashboard();
}
