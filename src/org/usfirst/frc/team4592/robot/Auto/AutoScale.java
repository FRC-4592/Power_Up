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
			if(counter >= 505 && counter <= 605) {
				myDrive.autoTurn(90);
			}else if(counter >= 606 && counter <= 610) {
				myDrive.zeroSensors();
			}else if(counter >= 615 && counter <= 1500) {
				myDrive.autoDriveStraight(18);
			}else if(counter >= 1505 && counter <= 1605) {
				myDrive.autoTurn(0);
			}else if(counter >= 1606 && counter <= 1610) {
				myDrive.zeroSensors();
			}else if(counter >= 1615 && counter <= 1755) {
				myDrive.autoDriveStraight(3);
			}else if(counter >= 1760 && counter <= 2605) {
				elevator.state = ElevatorState.AutoScalePosition;
			}else if(counter >= 2610) {
					clawWheels.state = ClawWheelsState.Spit;
			}/*else if(counter >= 2715) {
				clawWheels.state = ClawWheelsState.Off;
			}*/
		}else if(gameData.charAt(1) == 'R') {
			if(counter >= 505 && counter <= 800) {
				myDrive.autoDriveStraight(23.53);
			}else if(counter >= 805 && counter <= 1725) {
				myDrive.autoTurn(90);
				
				if(counter >= 855 && counter <= 1700) {
					elevator.state = ElevatorState.AutoScalePosition;
				}
			}else if(counter >= 1705) {
				clawWheels.state = ClawWheelsState.Spit;
			}
		}else {
			
		}
		
		counter++;
	}

	@Override
	public void outputToSmartDashboard() {
		// TODO Auto-generated method stub
		
	}
}