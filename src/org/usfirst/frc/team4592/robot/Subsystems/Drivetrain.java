package org.usfirst.frc.team4592.robot.Subsystems;

import org.usfirst.frc.team4592.robot.Constants;
import org.usfirst.frc.team4592.robot.Hardware;
import org.usfirst.frc.team4592.robot.Lib.SubsystemFramework;
import org.usfirst.frc.team4592.robot.Util.PID;
import org.usfirst.frc.team4592.robot.Util.doubleSolenoid;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

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
	private AHRS MXP;

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
			doubleSolenoid shifter, AHRS MXP, double Average_Ticks_Per_Feet, double Turn_ANGLE_Kp,
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

		// MXP (Gyro)
		this.MXP = MXP;

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
		LowGear, HighGear, Off;
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
		//goal_Angle_Error = goal_Angle + SpartanBoard.getAngle();

		myRobot.arcadeDrive(Drive_PI.getControlledOutputP(goal_Ticks_Error),
				Drive_Angle_PI.getOutputP(goal_Angle_Error));
	}

	// Turn to wanted angle
	public void autoTurn(double wantedDegree) {
		//goal_Angle_Error = wantedDegree + SpartanBoard.getAngle();

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

	@Override
	public void update() {
		DrivetrainStates newState = state;

		switch (state) {
			case LowGear:
				// Shift Into LowGear
				shifter.close();
	
				// Joystick Control
				myRobot.arcadeDrive((Hardware.driverPad.getRawAxis(1) * -1), (Hardware.driverPad.getRawAxis(4) * -1), false);
	
				// Switch To HighGear When Asked
				if (Hardware.driverPad.getRawButton(Constants.DRIVETRAIN_HIGHGEAR)) {
					newState = DrivetrainStates.HighGear;
				}else if(Hardware.driverPad.getRawButton(Constants.CLIMB_DOWN)) {
					newState = DrivetrainStates.Off;
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
				}else if(Hardware.driverPad.getRawButton(Constants.CLIMB_DOWN)) {
					newState = DrivetrainStates.Off;
				}
				
	break;
			case Off:
				//I do nothing but shut off drivetrain
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

		// Robot Position
		/*SmartDashboard.putNumber("Right Position", rightMasterMotor.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Left Position", leftMasterMotor.getSelectedSensorPosition(0));
		System.out.println("Right Position: " + rightMasterMotor.getSelectedSensorPosition(0));
		System.out.println("Left Position: " + leftMasterMotor.getSelectedSensorPosition(0));
		
		/* Display 6-axis Processed Angle Data                                      
        SmartDashboard.putBoolean(  "IMU_Connected", MXP.isConnected());
        SmartDashboard.putBoolean(  "IMU_IsCalibrating", MXP.isCalibrating());
        SmartDashboard.putNumber(   "IMU_Yaw", MXP.getYaw());
        SmartDashboard.putNumber(   "IMU_Pitch", MXP.getPitch());
        SmartDashboard.putNumber(   "IMU_Roll", MXP.getRoll());
        
        /* Display tilt-corrected, Magnetometer-based heading (requires             
        /* magnetometer calibration to be useful)                                   
        
        SmartDashboard.putNumber(   "IMU_CompassHeading",   MXP.getCompassHeading());
        
        /* Display 9-axis Heading (requires magnetometer calibration to be useful)  
        SmartDashboard.putNumber(   "IMU_FusedHeading",     MXP.getFusedHeading());

        /* These functions are compatible w/the WPI Gyro Class, providing a simple  
        /* path for upgrading from the Kit-of-Parts gyro to the navx MXP            
        
        SmartDashboard.putNumber(   "IMU_TotalYaw",         MXP.getAngle());
        SmartDashboard.putNumber(   "IMU_YawRateDPS",       MXP.getRate());

        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) 
        
        SmartDashboard.putNumber(   "IMU_Accel_X",          MXP.getWorldLinearAccelX());
        SmartDashboard.putNumber(   "IMU_Accel_Y",          MXP.getWorldLinearAccelY());
        SmartDashboard.putBoolean(  "IMU_IsMoving",         MXP.isMoving());
        SmartDashboard.putBoolean(  "IMU_IsRotating",       MXP.isRotating());

        /* Display estimates of velocity/displacement.  Note that these values are  
        /* not expected to be accurate enough for estimating robot position on a    
        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding 
        /* of these errors due to single (velocity) integration and especially      
        /* double (displacement) integration.                                       
        
        SmartDashboard.putNumber(   "Velocity_X",           MXP.getVelocityX());
        SmartDashboard.putNumber(   "Velocity_Y",           MXP.getVelocityY());
        SmartDashboard.putNumber(   "Displacement_X",       MXP.getDisplacementX());
        SmartDashboard.putNumber(   "Displacement_Y",       MXP.getDisplacementY());
        
        /* Display Raw Gyro/Accelerometer/Magnetometer Values                       
        /* NOTE:  These values are not normally necessary, but are made available   
        /* for advanced users.  Before using this data, please consider whether     
        /* the processed data (see above) will suit your needs.                     
        
        SmartDashboard.putNumber(   "RawGyro_X",            MXP.getRawGyroX());
        SmartDashboard.putNumber(   "RawGyro_Y",            MXP.getRawGyroY());
        SmartDashboard.putNumber(   "RawGyro_Z",            MXP.getRawGyroZ());
        SmartDashboard.putNumber(   "RawAccel_X",           MXP.getRawAccelX());
        SmartDashboard.putNumber(   "RawAccel_Y",           MXP.getRawAccelY());
        SmartDashboard.putNumber(   "RawAccel_Z",           MXP.getRawAccelZ());
        SmartDashboard.putNumber(   "RawMag_X",             MXP.getRawMagX());
        SmartDashboard.putNumber(   "RawMag_Y",             MXP.getRawMagY());
        SmartDashboard.putNumber(   "RawMag_Z",             MXP.getRawMagZ());
        SmartDashboard.putNumber(   "IMU_Temp_C",           MXP.getTempC());
        SmartDashboard.putNumber(   "IMU_Timestamp",        MXP.getLastSensorTimestamp());
        
        /* Omnimount Yaw Axis Information                                           
        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  
        AHRS.BoardYawAxis yaw_axis = MXP.getBoardYawAxis();
        SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
        SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
        
        /* Sensor Board Information                                                 
        SmartDashboard.putString(   "FirmwareVersion",      MXP.getFirmwareVersion());
        
        /* Quaternion Data                                                          
        /* Quaternions are fascinating, and are the most compact representation of  
        /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  
        /* from the Quaternions.  If interested in motion processing, knowledge of  
        /* Quaternions is highly recommended.                                       
        SmartDashboard.putNumber(   "QuaternionW",          MXP.getQuaternionW());
        SmartDashboard.putNumber(   "QuaternionX",          MXP.getQuaternionX());
        SmartDashboard.putNumber(   "QuaternionY",          MXP.getQuaternionY());
        SmartDashboard.putNumber(   "QuaternionZ",          MXP.getQuaternionZ());
        
        /* Connectivity Debugging Support                                           
        SmartDashboard.putNumber(   "IMU_Byte_Count",       MXP.getByteCount());
        SmartDashboard.putNumber(   "IMU_Update_Count",     MXP.getUpdateCount());*/
	}

	@Override
	public void setupSensors() {
		// Setup MXP
			//MXP.reset();
		
		// Setup Master Slave Relationship
				rightSlaveMotor.follow(rightMasterMotor);
				rightSlaveMotor2.follow(rightMasterMotor);
				leftSlaveMotor.follow(leftMasterMotor);
				leftSlaveMotor2.follow(leftMasterMotor);
				
		// Setup Master Encoders
		rightMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		leftMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

		rightMasterMotor.setSelectedSensorPosition(0, 0, 10);
		leftMasterMotor.setSelectedSensorPosition(0, 0, 10);
		
		rightMasterMotor.configNominalOutputForward(0, 10);
		leftMasterMotor.configNominalOutputForward(0, 10);
		rightMasterMotor.configNominalOutputReverse(0, 10);
		leftMasterMotor.configNominalOutputReverse(0, 10);
		rightMasterMotor.configPeakOutputForward(1, 10);
		leftMasterMotor.configPeakOutputForward(1, 10);
		rightMasterMotor.configPeakOutputReverse(-1, 10);
		leftMasterMotor.configPeakOutputReverse(-1, 10);
	}
}