package org.usfirst.frc.team4592.robot;

/* This file is used to hold constant robot values.
 * DO NOT DEFINE MOTOR PWM OR PID VALUES OUTSIDE OF THIS FILE!!
 */

public class Constants {
	//Drivetrain CAN Values
	public static final int RIGHT_MASTER_MOTOR_CAN = 0;
	public static final int RIGHT_SLAVE_MOTOR_CAN = 1;
	public static final int RIGHT_SLAVE_MOTOR_2_CAN = 2;
	public static final int LEFT_MASTER_MOTOR_CAN = 3;
	public static final int LEFT_SLAVE_MOTOR_CAN = 4;
	public static final int LEFT_SLAVE_MOTOR_2_CAN = 5;
	
	//Drivetrain Pnuematics
	public static final int SHIFTER_OPEN = 0;
	public static final int SHIFTER_CLOSE = 1;
	
	//Elevator
	public static final int ELEVATOR_MASTER_MOTOR_CAN = 6;
	public static final int ELEVATOR_SLAVE_MOTOR_CAN = 7;
		
	//Claw Rotation
	public static final int CLAW_MOTOR_CAN = 8;
	
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
		public static final int DRIVETRAIN_LOWGEAR = 0;
		public static final int DRIVETRAIN_HIGHGEAR = 1;
		
		//Elevator Buttons
		public static final int ELEVATOR_STOW = 2;
		public static final int ELEVATOR_INTAKE = 3;
		public static final int ELEVATOR_SWITCH = 4;
		public static final int ELEVATOR_LOW_SCALE = 5;
		public static final int ELEVATOR_MID_SCALE = 6;
		public static final int ELEVATOR_HIGH_SCALE = 7;
		
		//Claw Buttons
		public static final int CLAW_INTAKE = 8;
		public static final int CLAW_SPIT = 9;
		
	//Operator Buttons	
		//Claw Buttons
		public static final int CLAW_INTAKE_POS = 0;
		public static final int CLAW_90 = 1;
		public static final int CLAW_STOW_POS = 2;
		
		//Elevator Button
		public static final int ELEVATOR_CLIMB_POS = 3;
		
		//Climber Buttons
		public static final int CLIMB = 4;
		
		//Wing Buttons
		public static final int RIGHT_WING_DEPLOY = 5;
		public static final int LEFT_WING_DEPLOY = 6;
		public static final int WINGS_DEPLOY = 7;
		public static final int RIGHT_WING_LIFT = 8;
		public static final int LEFT_WING_LIFT = 9;
}