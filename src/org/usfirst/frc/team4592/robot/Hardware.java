package org.usfirst.frc.team4592.robot;

import org.usfirst.frc.team4592.robot.Util.doubleSolenoid;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;

//This file holds all initialization for hardware and buttons

public class Hardware {
	//Drivetrain Motors
	public static final WPI_TalonSRX rightMasterMotor =
			new WPI_TalonSRX(Constants.RIGHT_MASTER_MOTOR_CAN);
	public static final WPI_VictorSPX rightSlaveMotor = 
			new WPI_VictorSPX(Constants.RIGHT_SLAVE_MOTOR_CAN);
	public static final WPI_VictorSPX rightSlaveMotor2 =
			new WPI_VictorSPX(Constants.RIGHT_SLAVE_MOTOR_2_CAN);
	public static final WPI_TalonSRX leftMasterMotor =
			new WPI_TalonSRX(Constants.LEFT_MASTER_MOTOR_CAN);
	public static final WPI_VictorSPX leftSlaveMotor =
			new WPI_VictorSPX(Constants.LEFT_SLAVE_MOTOR_CAN);
	public static final WPI_VictorSPX leftSlaveMotor2 = 
			new WPI_VictorSPX(Constants.LEFT_SLAVE_MOTOR_2_CAN);
	
	//Elevator
	public static final TalonSRX elevatorMotor = 
			new TalonSRX(Constants.ELEVATOR_MOTOR_CAN);
	
	//Claw Rotation
	public static final TalonSRX clawRotationMotor =
			new TalonSRX(Constants.CLAW_MOTOR_CAN);
	
	//Claw Wheel
	public static final VictorSP clawRightMotor = 
			new VictorSP(Constants.RIGHT_CLAW_WHEEL_MOTOR_PWM);
	public static final VictorSP clawLeftMotor =
			new VictorSP(Constants.LEFT_CLAW_WHEEL_MOTOR_PWM);
	
	//Climber
	public static final VictorSPX rightClimberMotor =
			new VictorSPX(Constants.RIGHT_CLIMBER_MOTOR_CAN);
	public static final VictorSPX rightClimberMotor2 = 
			new VictorSPX(Constants.RIGHT_CLIMBER_MOTOR_2_CAN);
	public static final VictorSPX leftClimberMotor =
			new VictorSPX(Constants.LEFT_CLIMBER_MOTOR_CAN);
	public static final VictorSPX leftClimberMotor2 =
			new VictorSPX(Constants.LEFT_CLIMBER_MOTOR_2_CAN);
	
	//Climber Sensor
	public static final DigitalInput climberLimitSwitch =
			new DigitalInput(Constants.CLIMBER_LIMIT_SWITCH);
	
	//Gyro
	public static final AHRS MXP = new AHRS(Constants.MXP_PORT); 
		
	//Shifter
		public static final doubleSolenoid shifter = 
				new doubleSolenoid(Constants.SHIFTER_OPEN, Constants.SHIFTER_CLOSE);
		
	//Wings
		public static final doubleSolenoid wingRelease =
				new doubleSolenoid(Constants.WING_OPEN, Constants.WING_CLOSE);
		
		//Sticks
		public static final Joystick driverPad = 
				new Joystick(Constants.DRIVE_USB_PORT);
		public static final Joystick operatorPad = 
				new Joystick(Constants.OPERATOR_USB_PORT);
		/*public static final XboxController driverPad = 
				new XboxController(Constants.DRIVE_USB_PORT);*/
}
