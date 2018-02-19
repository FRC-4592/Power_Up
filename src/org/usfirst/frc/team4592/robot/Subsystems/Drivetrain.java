package org.usfirst.frc.team4592.robot.Subsystems;

import org.usfirst.frc.team4592.robot.Constants;
import org.usfirst.frc.team4592.robot.Hardware;
import org.usfirst.frc.team4592.robot.Lib.SubsystemFramework;
import org.usfirst.frc.team4592.robot.Util.PID;
import org.usfirst.frc.team4592.robot.Util.doubleSolenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemFramework {
	// Hardware
	private DifferentialDrive myRobot;
	private WPI_TalonSRX rightMasterMotor;
	private WPI_TalonSRX leftMasterMotor;
	private WPI_VictorSPX rightSlaveMotor;
	private WPI_VictorSPX leftSlaveMotor;
	private WPI_VictorSPX rightSlaveMotor2;
	private WPI_VictorSPX leftSlaveMotor2;
	private doubleSolenoid shifter;
	private ADXRS450_Gyro SpartanBoard;

	// PID
	private PID Turn_Angle_PI;
	private PID Drive_Angle_PI;
	private PID Drive_PI;

	// Constants
	private double Average_Ticks_Per_Feet;
	private double goal_Ticks;
	private double goal_Angle = 0;
	private double goal_Ticks_Error;
	private double goal_Angle_Error;

	// State
	public DrivetrainStates state = DrivetrainStates.HighGear;

	public Drivetrain(WPI_TalonSRX rightMasterMotor, WPI_VictorSPX rightSlaveMotor, WPI_VictorSPX rightSlaveMotor2,
			WPI_TalonSRX leftMasterMotor, WPI_VictorSPX leftSlaveMotor, WPI_VictorSPX leftSlaveMotor2,
			doubleSolenoid shifter, ADXRS450_Gyro SpartanBoard, double Average_Ticks_Per_Feet, double Turn_ANGLE_Kp,
			double Turn_ANGLE_Ki, double Drive_Angle_Kp, double Drive_Angle_Ki, double Drive_Kp, double Drive_Ki) {
		// Setup Drivetrain
		myRobot = new DifferentialDrive(rightMasterMotor, leftMasterMotor);

		// Motor Controllers
		this.rightMasterMotor = rightMasterMotor;
		this.rightSlaveMotor = rightSlaveMotor;
		this.rightSlaveMotor2 = rightSlaveMotor2;
		this.leftMasterMotor = leftMasterMotor;
		this.leftSlaveMotor = leftSlaveMotor;
		this.leftSlaveMotor2 = leftSlaveMotor2;

		// Shifter
		this.shifter = shifter;

		// Spartan Board (Gyro)
		this.SpartanBoard = SpartanBoard;

		// Constants
		this.Average_Ticks_Per_Feet = Average_Ticks_Per_Feet;

		// Setup PID
		this.Turn_Angle_PI = new PID(Turn_ANGLE_Kp, Turn_ANGLE_Ki);
		this.Drive_Angle_PI = new PID(Drive_Angle_Kp, Drive_Angle_Ki);
		this.Drive_PI = new PID(Drive_Kp, Drive_Ki);

	}

	public Drivetrain(WPI_TalonSRX rightMasterMotor, WPI_VictorSPX rightSlaveMotor, WPI_VictorSPX rightSlaveMotor2,
			WPI_TalonSRX leftMasterMotor, WPI_VictorSPX leftSlaveMotor, WPI_VictorSPX leftSlaveMotor2,
			doubleSolenoid shifter) {
		// Setup Drivetrain
		myRobot = new DifferentialDrive(rightMasterMotor, leftMasterMotor);

		// Motor Controllers
		this.rightMasterMotor = rightMasterMotor;
		this.rightSlaveMotor = rightSlaveMotor;
		this.rightSlaveMotor2 = rightSlaveMotor2;
		this.leftMasterMotor = leftMasterMotor;
		this.leftSlaveMotor = leftSlaveMotor;
		this.leftSlaveMotor2 = leftSlaveMotor2;

		// Shifter
		this.shifter = shifter;
	}

	/*
	 * Drivetrain States Low Gear State Puts Robot In Lowest Speed Best For Pushing
	 * High Gear State Puts Robot In Highest Speed Best For Long Distance Travel
	 */
	public enum DrivetrainStates {
		LowGear, HighGear;
	}

	// Drive Forward at Indicated Speed
	public void autoDrivePower(double speed) {
		myRobot.arcadeDrive(speed, 0);
	}

	// Drive Straight
	public void autoDriveStraight(double amtToDrive) {
		shifter.close();

		goal_Ticks = (amtToDrive * Average_Ticks_Per_Feet);

		goal_Ticks_Error = (goal_Ticks - getPosition());
		goal_Angle_Error = goal_Angle + SpartanBoard.getAngle();

		myRobot.arcadeDrive(Drive_PI.getControlledOutputP(goal_Ticks_Error),
				Drive_Angle_PI.getOutputP(goal_Angle_Error));
	}

	// Turn to wanted angle
	public void autoTurn(double wantedDegree) {
		goal_Angle_Error = wantedDegree + SpartanBoard.getAngle();

		System.out.println(goal_Angle_Error);

		myRobot.arcadeDrive(0, Turn_Angle_PI.getOutputP(goal_Angle_Error));
	}

	// Get Average Position
	public double getPosition() {
		return ((-1 * rightMasterMotor.getSelectedSensorPosition(0) + leftMasterMotor.getSelectedSensorPosition(0))
				/ 2);
	}

	// Get Motor Positions
	public double getRightPosition() {
		return rightMasterMotor.getSelectedSensorPosition(0);
	}

	public double getLeftPosition() {
		return leftMasterMotor.getSelectedSensorPosition(0);
	}

	// Get Goal Constants
	public double get_GoalTicks() {
		return goal_Ticks;
	}

	public double get_GoalAngle() {
		return goal_Angle_Error;
	}

	public double get_Angle() {
		return SpartanBoard.getAngle();
	}

	@Override
	public void update() {
		DrivetrainStates newState = state;

		switch (state) {
			case LowGear:
				// Shift Into LowGear
				shifter.close();
	
				// Joystick Control
				myRobot.arcadeDrive((Hardware.driverPad.getRawAxis(1) * -1), (Hardware.driverPad.getRawAxis(4) * -1),false);
	
				// Switch To HighGear When Asked
				if (Hardware.driverPad.getRawButton(Constants.DRIVETRAIN_HIGHGEAR)) {
					newState = DrivetrainStates.HighGear;
				}
			break;
			
			case HighGear:
				// Shift Into HighGear
				shifter.open();
	
				// Joystick Control
				myRobot.arcadeDrive((Hardware.driverPad.getRawAxis(1) * -1), (Hardware.driverPad.getRawAxis(4) * -1),
						false);
	
				// Switch To LowGear When Asked
				if (Hardware.driverPad.getRawButton(Constants.DRIVETRAIN_LOWGEAR)) {
					newState = DrivetrainStates.LowGear;
				}
			break;
	
			default:
				newState = DrivetrainStates.HighGear;
			break;
		}

		if (newState != state) {
			state = newState;
		}

		outputToSmartDashboard();
	}

	@Override
	public void outputToSmartDashboard() {
		// Robot Angle
		// SmartDashboard.putNumber("Angle", SpartanBoard.getAngle());

		// Robot Position
		SmartDashboard.putNumber("Right Position", rightMasterMotor.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Left Position", leftMasterMotor.getSelectedSensorPosition(0));
		System.out.println("Right Position: " + rightMasterMotor.getSelectedSensorPosition(0));
		System.out.println("Left Position: " + leftMasterMotor.getSelectedSensorPosition(0));
	}

	/*
	 * Reset SpartanBoard When Current Robot Angle is outside (-360, 360)
	 */
	public void resetSpartanBoard() {
		if (Math.abs(SpartanBoard.getAngle()) > 360) {
			SpartanBoard.reset();
		}
	}

	public void zeroSpartanBoard() {
		SpartanBoard.reset();
	}

	@Override
	public void setupSensors() {
		// Reset SpartanBoard
		// SpartanBoard.reset();

		// Setup Master Encoders
		rightMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		leftMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

		rightMasterMotor.setSelectedSensorPosition(0, 0, 10);
		leftMasterMotor.setSelectedSensorPosition(0, 0, 10);

		/*
		 * MAY NEED TO REVERSE SENSORS rightMasterMotor.setSensorPhase(false);
		 * leftMasterMotor.setSensorPhase(false);
		 */

		// Setup Master Slave Relationship
		leftSlaveMotor.set(ControlMode.Follower, leftMasterMotor.getDeviceID());
		leftSlaveMotor2.set(ControlMode.Follower, leftMasterMotor.getDeviceID());
		rightSlaveMotor.follow(rightMasterMotor);
		rightSlaveMotor2.follow(rightMasterMotor);
		leftSlaveMotor.follow(leftMasterMotor);
		leftSlaveMotor2.follow(leftMasterMotor);
	}
}