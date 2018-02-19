package org.usfirst.frc.team4592.robot;

import org.usfirst.frc.team4592.robot.Util.doubleSolenoid;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

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
	//Gyro
		/*public static final ADXRS450_Gyro SpartanBoard =
				new ADXRS450_Gyro();*/
		
	//Shifter
		public static final doubleSolenoid shifter = 
				new doubleSolenoid(Constants.SHIFTER_OPEN, Constants.SHIFTER_CLOSE);
		
		//Sticks
		public static final Joystick driverPad = 
				new Joystick(Constants.DRIVE_USB_PORT);
		public static final Joystick operatorPad = 
				new Joystick(Constants.OPERATOR_USB_PORT);
		/*public static final XboxController driverPad = 
				new XboxController(Constants.DRIVE_USB_PORT);*/
}
