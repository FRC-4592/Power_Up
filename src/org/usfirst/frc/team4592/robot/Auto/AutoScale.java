package org.usfirst.frc.team4592.robot.Auto;

import org.usfirst.frc.team4592.robot.Lib.AutoFramework;
import org.usfirst.frc.team4592.robot.Subsystems.Drivetrain;
import org.usfirst.frc.team4592.robot.Subsystems.Elevator;
import org.usfirst.frc.team4592.robot.Subsystems.Elevator.ElevatorState;
import org.usfirst.frc.team4592.robot.Subsystems.ClawMech.ClawWheels;
import org.usfirst.frc.team4592.robot.Subsystems.ClawMech.ClawWheels.ClawWheelsState;

public class AutoScale extends AutoFramework{
	private Drivetrain myDrive;
	private Elevator elevator;
	private ClawWheels clawWheels;
	
	private int counter2;
	
	public AutoScale(Drivetrain myDrive, Elevator elevator, ClawWheels clawWheels) {
		this.myDrive = myDrive;
		this.elevator = elevator;
		this.clawWheels = clawWheels;
	}
	
	@Override
	public void update() {
		if(counter >= 0 && counter <= 500) {
			myDrive.autoDriveStraight(15.5);	
		}
		
		if(gameData.charAt(1) == 'L') {
			//Create PID loop for auto scale to fix turn
			if(counter >= 505 && counter <= 605) {
				myDrive.autoTurn(90);
			}else if(counter >= 606 && counter <= 610) {
				myDrive.zeroSensors();
			//Drive more
			}else if(counter >= 615 && counter <= 2000) {
				myDrive.autoDriveStraight(25);
			}else if(counter >= 2005 && counter <= 2105) {
				myDrive.autoTurn(0);
			}else if(counter >= 2106 && counter <= 2110) {
				myDrive.zeroSensors();
			}else if(counter >= 2115 && counter <= 2155) {
				myDrive.autoDriveStraight(3);
			}else if(counter >= 2160 && counter <= 2610) {
				elevator.state = ElevatorState.AutoScalePosition;
			}else if(counter >= 2615 && counter <= 2715) {
				//make sure it spits out
					clawWheels.state = ClawWheelsState.Spit;
			}else if(counter >= 2720) {
				clawWheels.state = ClawWheelsState.Off;
			}
		}else if(gameData.charAt(1) == 'R') {
			//Drive less
			if(counter >= 505 && counter <= 725) {
				myDrive.autoDriveStraight(20);
			//Turn less may 45.92 or 60
			}else if(counter >= 730 && counter <= 1200) {
				myDrive.autoTurn(60);
				
				if(counter >= 750) {
					elevator.state = ElevatorState.AutoScalePosition;
				}
			}else if(counter >= 1205  && counter <= 1305) {
				clawWheels.state = ClawWheelsState.Spit;
			}else if(counter >= 1310) {
				clawWheels.state = ClawWheelsState.Off;
			}
		}else{
			//If not data is received do nothing
		}
		
		counter++;
	}

	@Override
	public void outputToSmartDashboard() {
		// TODO Auto-generated method stub
		
	}
}