/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4592.robot;

import org.usfirst.frc.team4592.robot.Auto.AutoScale;
import org.usfirst.frc.team4592.robot.Lib.Loop.MultiLooper;
import org.usfirst.frc.team4592.robot.Subsystems.Drivetrain;
import org.usfirst.frc.team4592.robot.Subsystems.Elevator;
import org.usfirst.frc.team4592.robot.Subsystems.Elevator.ElevatorState;
import org.usfirst.frc.team4592.robot.Subsystems.ClawMech.ClawPistons;
import org.usfirst.frc.team4592.robot.Subsystems.ClawMech.ClawRotation;
import org.usfirst.frc.team4592.robot.Subsystems.ClawMech.ClawWheels;

import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	// Loopers
	private MultiLooper DriveLooper = new MultiLooper("DriveLooper", 1 / 200.0, false); // Drivetrain looper
	private MultiLooper SSLooper = new MultiLooper("SSLooper", 1 / 200.0, false); // Subsystems looper
	// Subsystems
	private Drivetrain myDrive = new Drivetrain(Hardware.rightMasterMotor, Hardware.rightSlaveMotor,
												Hardware.rightSlaveMotor2, Hardware.leftMasterMotor, 
												Hardware.leftSlaveMotor, Hardware.leftSlaveMotor2,
												Hardware.shifter, Hardware.MXP, Constants.Average_Ticks_Per_Foot,
												Constants.Turn_Angle_Kp, Constants.Turn_Angle_Ki, Constants.Drive_Angle_Kp,
												Constants.Drive_Angle_Ki, Constants.Drive_Kp, Constants.Drive_Ki);
	private ClawWheels clawWheels = new ClawWheels(Hardware.clawLeftMotor);
	private MultiLooper AutoLooper = new MultiLooper("AutoLooper", 1 / 200.0, false); // Auto looper
	private Elevator elevator = new Elevator(Hardware.elevatorMotor, Constants.Average_Ticks_Per_Inch,
												Constants.Elevator_Kf, Constants.Elevator_Kp, 
												Constants.Elevator_Ki, Constants.Elevator_Kd);
	private ClawRotation clawRotation = new ClawRotation (Hardware.clawRotationMotor, Constants.Average_Ticks_Per_Degree,
												Constants.Claw_Rotation_Kf, Constants.Claw_Rotation_Kp,
												Constants.Claw_Rotation_Ki, Constants.Claw_Rotation_Kd);
	private ClawPistons clawPistons = new ClawPistons(Hardware.clawPistons);
	/*private Climber climber = new Climber(Hardware.rightClimberMotor, Hardware.rightClimberMotor2,
											Hardware.leftClimberMotor, Hardware.leftClimberMotor2);
	private Wings wings = new Wings(Hardware.wingRelease);*/
	
	
	//Auto
	//private AutoSwitch autoSwitch = new AutoSwitch(myDrive, elevator, clawWheels);
	private AutoScale autoScale = new AutoScale(myDrive, elevator, clawWheels);
	
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		myDrive.setupSensors();
		clawRotation.setupSensors();
		elevator.setupSensors();
		
		DriveLooper.addLoopable(myDrive);
		
		SSLooper.addLoopable(elevator);
		SSLooper.addLoopable(clawWheels);
		SSLooper.addLoopable(clawRotation);
		SSLooper.addLoopable(clawPistons);
		//SSLooper.addLoopable(climber);
		//SSLooper.addLoopable(wings); 
		
		
		//AutoLooper.addLoopable(autoSwitch);
		AutoLooper.addLoopable(autoScale);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString line to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the SendableChooser
	 * make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		//autoSwitch.getGameData();
		autoScale.getGameData();
		AutoLooper.start();
		AutoLooper.update();
		SSLooper.start();
		SSLooper.update();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		//myDrive.outputToSmartDashboard();
	}

	public void teleopInit() {
		elevator.state = ElevatorState.StowPosition;
		
		AutoLooper.stop();
		DriveLooper.start();
		DriveLooper.update();
		SSLooper.start();
		SSLooper.update();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
	
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}