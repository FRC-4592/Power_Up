package org.usfirst.frc.team4592.robot.Auto;

import org.usfirst.frc.team4592.robot.Lib.AutoFramework;
import org.usfirst.frc.team4592.robot.Subsystems.Drivetrain;
import org.usfirst.frc.team4592.robot.Subsystems.Elevator;
import org.usfirst.frc.team4592.robot.Subsystems.Elevator.ElevatorState;
import org.usfirst.frc.team4592.robot.Subsystems.ClawMech.ClawWheels;
import org.usfirst.frc.team4592.robot.Subsystems.ClawMech.ClawWheels.ClawWheelsState;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoSwitch extends AutoFramework{
	private Drivetrain myDrive;
	private Elevator elevator;
	private ClawWheels clawWheels;
	
	public AutoSwitch(Drivetrain myDrive, Elevator elevator, ClawWheels clawWheels) {
		this.myDrive = myDrive;
		this.elevator = elevator;
		this.clawWheels = clawWheels;
	}
	
	@Override
	public void update() {
		if(counter >= 0 && counter <= 275) {
			myDrive.autoDriveStraight(2);
		}
		
		if(gameData.charAt(0) == 'L') {
			if(counter >= 280 && counter <= 400) {
				myDrive.autoTurn(60);
			}else if(counter >= 401 && counter <= 405) {
				myDrive.zeroSensors();
			}else if(counter >= 410 && counter <= 800) {
				myDrive.autoDriveStraight(5);
			}else if(counter >= 805 && counter <= 1000) {
				elevator.state = ElevatorState.SwitchPosition;
				myDrive.autoTurn(0);
			}else if(counter >= 1001 && counter <= 1005) {
				myDrive.zeroSensors();
			}else if(counter >= 1010 && counter <= 1400) {
				myDrive.autoDriveStraight(3.5);
			}else if(counter >= 1405 && counter <= 1465) {
				clawWheels.state = ClawWheelsState.Spit;
			}else if(counter >= 1470) {
				clawWheels.state = ClawWheelsState.Off;
			}
		}else if(gameData.charAt(0) == 'R') {
			if(counter >= 280 && counter <= 400) {
				myDrive.autoTurn(-60);
			}else if(counter >= 401 && counter <= 405) {
				myDrive.zeroSensors();
			}else if(counter >= 410 && counter <= 750) {
				myDrive.autoDriveStraight(4.25);
			}else if(counter >= 755 && counter <= 950) {
				elevator.state = ElevatorState.SwitchPosition;
				myDrive.autoTurn(0);
			}else if(counter >= 951 && counter <= 955) {
				myDrive.zeroSensors();
			}else if(counter >= 960 && counter <= 1450) {
				myDrive.autoDriveStraight(3.75);
			}else if(counter >= 1455 && counter <= 1515) {
				clawWheels.state = ClawWheelsState.Spit;
			}else if(counter >= 1520) {
				clawWheels.state = ClawWheelsState.Off;
			}
		}else{
			if(counter >= 280 && counter <= 400) {
				myDrive.autoTurn(60);
			}else if(counter >= 401 && counter <= 405) {
				myDrive.zeroSensors();
			}else if(counter >= 410 && counter <= 750) {
				myDrive.autoDriveStraight(4.5);
			}else if(counter >= 755 && counter <= 950) {
				elevator.state = ElevatorState.SwitchPosition;
				myDrive.autoTurn(0);
			}else if(counter >= 951 && counter <= 955) {
				myDrive.zeroSensors();
			}else if(counter >= 960 && counter <= 1260) {
				myDrive.autoDriveStraight(3);
			}
		}
		
		counter++;
	}

	@Override
	public void outputToSmartDashboard() {
		// TODO Auto-generated method stub
		
	}
}