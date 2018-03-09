package org.usfirst.frc.team4592.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

/* This file is used to hold constant robot values.
 * DO NOT DEFINE MOTOR PWM OR PID VALUES OUTSIDE OF THIS FILE!!
 */

public class Constants {
	//Drivetrain CAN Values
	public static final int RIGHT_MASTER_MOTOR_CAN = 2;
	public static final int RIGHT_SLAVE_MOTOR_CAN = 14;
	public static final int RIGHT_SLAVE_MOTOR_2_CAN = 15;
	public static final int LEFT_MASTER_MOTOR_CAN = 3;
	public static final int LEFT_SLAVE_MOTOR_CAN = 1;
	public static final int LEFT_SLAVE_MOTOR_2_CAN = 0;
	
	//Drivetrain Pnuematics
	public static final int SHIFTER_OPEN = 3;
	public static final int SHIFTER_CLOSE = 2;
	
	//Elevator
	public static final int ELEVATOR_MOTOR_CAN = 10;
		
	//Claw Rotation
	public static final int CLAW_MOTOR_CAN = 11;
	
	//Claw Wheels
	public static final int RIGHT_CLAW_WHEEL_MOTOR_PWM = 8;
	public static final int LEFT_CLAW_WHEEL_MOTOR_PWM = 9;
	
	//Climber
	public static final int RIGHT_CLIMBER_MOTOR_CAN = 5;
	public static final int RIGHT_CLIMBER_MOTOR_2_CAN = 4;
	public static final int LEFT_CLIMBER_MOTOR_CAN = 7;
	public static final int LEFT_CLIMBER_MOTOR_2_CAN = 6;
	
	//Climber Sensor
	public static final int CLIMBER_LIMIT_SWITCH = 0;
	
	//Wing Pneumatics
	public static final int WING_OPEN = 0;
	public static final int WING_CLOSE = 1;
	
	//Gyro
	public static final Port MXP_PORT = SPI.Port.kMXP;
	//Stick USB Values
	public static final int DRIVE_USB_PORT = 0;
	public static final int OPERATOR_USB_PORT = 1;
	
	//Driver Buttons
		//Drivetrain Buttons
		public static final int DRIVETRAIN_LOWGEAR = 1;
		public static final int DRIVETRAIN_HIGHGEAR = 2;
	
		//Elevator Buttons
		public static final int INTAKE = 3;
		public static final int SWITCH = 4;
		public static final int SCALE = 5;
		public static final int HIGH_SCALE = 6;
		public static final int CLIMB_UP = 7;
		public static final int CLIMB_DOWN = 8;
		
		//Claw Buttons
		public static final int CLAW_INTAKE = 3;
		public static final int CLAW_SPIT = 2;
		
		//Wing Buttons
		public static final int WING_DEPLOY = 9;
		
		//Climber Buttons
		public static final int CLIMB = 10;
		
		
	//Operator Buttons		
		//Buttons
		public static final int Button1 = 1;
		public static final int Button2 = 2;
		public static final int Button3 = 3;
		public static final int Button4 = 4;
		public static final int Button5 = 5;
		public static final int Button6 = 6;
		public static final int Button8 = 8;
		public static final int Button9 = 9;
		public static final int Button12 = 12;
		public static final int Button13 = 13;
		public static final int Button14 = 14;
		public static final int Button16 = 16;
		
	//Elevator PID Values
		public static final double Average_Ticks_Per_Inch = -1052.22;
		public static final double Elevator_Kf = 0;
		public static final double Elevator_Kp = 1.1;
		public static final double Elevator_Ki = 0;
		public static final double Elevator_Kd = 0;
		public static final double Safe_Position = 2;
		
	//Claw Rotation PID Values
		public static final double Average_Ticks_Per_Degree = -41.064957;
		public static final double Claw_Rotation_Kf = 0;
		public static final double Claw_Rotation_Kp = 1;
		public static final double Claw_Rotation_Ki = 0.;
		public static final double Claw_Rotation_Kd = 0;
		public static final double Safe_Angle = 70;
		
	//Drivetrain PID Values
		public static final double Average_Ticks_Per_Foot = 4032.97;
		public static final double Turn_Angle_Kp = 0.0124;
		public static final double Turn_Angle_Ki = 0;
		public static final double Drive_Angle_Kp = 0.06;
		public static final double Drive_Angle_Ki = 0;
		public static final double Drive_Kp = 0.000115;
		public static final double Drive_Ki = 0;
}