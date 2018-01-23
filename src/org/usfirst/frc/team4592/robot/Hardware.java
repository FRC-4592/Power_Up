package org.usfirst.frc.team4592.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;

//This file holds all initialization for hardware and buttons

public class Hardware {
	//Drivetrain Motors
	public static final WPI_TalonSRX rightMasterMotor =
			new WPI_TalonSRX(Constants.RIGHT_MASTER_MOTOR_CAN);
	public static final VictorSPX rightSlaveMotor = 
			new VictorSPX(Constants.RIGHT_SLAVE_MOTOR_CAN);
	public static final VictorSPX rightSlaveMotor2 =
			new VictorSPX(Constants.RIGHT_SLAVE_MOTOR_2_CAN);
	public static final WPI_TalonSRX leftMasterMotor =
			new WPI_TalonSRX(Constants.LEFT_MASTER_MOTOR_CAN);
	public static final VictorSPX leftSlaveMotor =
			new VictorSPX(Constants.LEFT_SLAVE_MOTOR_CAN);
	public static final VictorSPX leftSlaveMotor2 = 
			new VictorSPX(Constants.LEFT_SLAVE_MOTOR_2_CAN);
	
	//Elevator
	
	//Gyro
		public static final ADXRS450_Gyro SpartanBoard =
				new ADXRS450_Gyro();
		
		//Sticks
		public static final Joystick driverPad = 
				new Joystick(Constants.DRIVE_USB_PORT);
		public static final Joystick operatorPad = 
				new Joystick(Constants.OPERATOR_USB_PORT);
}
