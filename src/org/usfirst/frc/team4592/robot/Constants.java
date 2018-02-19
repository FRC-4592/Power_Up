package org.usfirst.frc.team4592.robot;

/* This file is used to hold constant robot values.
 * DO NOT DEFINE MOTOR PWM OR PID VALUES OUTSIDE OF THIS FILE!!
 */

public class Constants {
	//Drivetrain CAN Values
	public static final int RIGHT_MASTER_MOTOR_CAN = 2;
	public static final int RIGHT_SLAVE_MOTOR_CAN = 14;
	public static final int RIGHT_SLAVE_MOTOR_2_CAN = 15;
	public static final int LEFT_MASTER_MOTOR_CAN = 3;
	public static final int LEFT_SLAVE_MOTOR_CAN = 12;
	public static final int LEFT_SLAVE_MOTOR_2_CAN = 13;
	
	//Drivetrain Pnuematics
	public static final int SHIFTER_OPEN = 6;
	public static final int SHIFTER_CLOSE = 7;
	
	//Elevator
	public static final int ELEVATOR_MOTOR_CAN = 0;
		
	//Claw Rotation
	public static final int CLAW_MOTOR_CAN = 1;
	
	//Claw Wheels
	public static final int RIGHT_CLAW_WHEEL_MOTOR_PWM = 0;
	public static final int LEFT_CLAW_WHEEL_MOTOR_PWM = 1;
	
	//Climber
	public static final int RIGHT_CLIMBER_MOTOR_PWM = 2;
	public static final int LEFT_CLIMBER_MOTOR_PWM = 3;
	
	//Wings
	
	//Stick USB Values
	public static final int DRIVE_USB_PORT = 0;
	public static final int OPERATOR_USB_PORT = 1;
	
	//Driver Buttons
		//Drivetrain Buttons
		public static final int DRIVETRAIN_LOWGEAR = 1;
		public static final int DRIVETRAIN_HIGHGEAR = 2;
	
		//Elevator Buttons
		public static final int ELEVATOR_STOW = 3;
		public static final int ELEVATOR_INTAKE = 4;
		public static final int ELEVATOR_SWITCH = 5;
		public static final int ELEVATOR_LOW_SCALE = 6;
		public static final int ELEVATOR_MID_SCALE = 7;
		public static final int ELEVATOR_HIGH_SCALE = 8;
		
		//Claw Buttons
		public static final int CLAW_INTAKE = 2;
		public static final int CLAW_SPIT = 3;
		
	//Operator Buttons	
		//Claw Buttons
		public static final int CLAW_INTAKE_POS = 3;
		public static final int CLAW_90 = 2;
		public static final int CLAW_STOW_POS = 3;
		
		//Elevator Button
		public static final int ELEVATOR_CLIMB_POS = 4;
		
		//Climber Buttons
		public static final int CLIMB = 5;
		
		//Wing Buttons
		public static final int RIGHT_WING_DEPLOY = 6;
		public static final int LEFT_WING_DEPLOY = 7;
		public static final int WINGS_DEPLOY = 8;
		public static final int RIGHT_WING_LIFT = 9;
		public static final int LEFT_WING_LIFT = 10;
		
	//Elevator PID Values
		public static final double Average_Ticks_Per_Inch = -844.34;
		public static final double Elevator_Kf = 0;
		public static final double Elevator_Kp = 0.00175;
		public static final double Elevator_Ki = 0;
		public static final double Elevator_Kd = 0;
		
	//Claw Rotation PID Values
		public static final double Average_Ticks_Per_Degree = -41.064957;
		public static final double Claw_Rotation_Kf = 0;
		public static final double Claw_Rotation_Kp = 0.00175;
		public static final double Claw_Rotation_Ki = 0.;
		public static final double Claw_Rotation_Kd = 0;
}