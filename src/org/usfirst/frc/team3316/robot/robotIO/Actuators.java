/**
 * Le robot actuators
 */
package org.usfirst.frc.team3316.robot.robotIO;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.config.Config;
import org.usfirst.frc.team3316.robot.config.Config.ConfigException;
import org.usfirst.frc.team3316.robot.logger.DBugLogger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.VictorSP;

public class Actuators 
{
	Config config = Robot.config;
	DBugLogger logger = Robot.logger;
	
	/*
	 * Chassis
	 */
	public SpeedController chassisMotorControllerLeft1, chassisMotorControllerLeft2;
	public SpeedController chassisMotorControllerRight1, chassisMotorControllerRight2;
	public SpeedController chassisMotorControllerCenter1, chassisMotorControllerCenter2;
	
	/*
	 * Stacker
	 */
	public DoubleSolenoid stackerSolenoidUpper;
	public DoubleSolenoid stackerSolenoidBottom;
	
	public DoubleSolenoid stackerSolenoidContainer;
	
	public DoubleSolenoid stackerSolenoidGripper;
	
	/*
	 * Anschluss
	 */
	public SpeedController anschlussMotorController;
	
	/*
	 * Roller-Gripper
	 */
	public SpeedController rollerGripperMotorControllerLeft;
	public SpeedController rollerGripperMotorControllerRight;
	
	public Actuators ()
	{
		try 
		{
			/*
			 * Chassis
			 */
			if (Robot.config.robotA)
			{
				chassisMotorControllerLeft1 = new VictorSP ((int) config.get("CHASSIS_MOTOR_CONTROLLER_LEFT_1"));
				chassisMotorControllerLeft2 = new VictorSP ((int) config.get("CHASSIS_MOTOR_CONTROLLER_LEFT_2"));
				
				chassisMotorControllerRight1 = new VictorSP ((int) config.get("CHASSIS_MOTOR_CONTROLLER_RIGHT_1"));
				chassisMotorControllerRight2 = new VictorSP ((int) config.get("CHASSIS_MOTOR_CONTROLLER_RIGHT_2"));
				
				chassisMotorControllerCenter1 = new VictorSP((int) config.get("CHASSIS_MOTOR_CONTROLLER_CENTER_1"));
				chassisMotorControllerCenter2 = new VictorSP((int) config.get("CHASSIS_MOTOR_CONTROLLER_CENTER_2"));
				/*
				 * Anschluss
				 */
				anschlussMotorController = new VictorSP ((int) config.get("ANSCHLUSS_MOTOR_CONTROLLER"));
				
				/*
				 * Roller-Gripper
				 */
				rollerGripperMotorControllerLeft = new VictorSP ((int) config.get("ROLLER_GRIPPER_MOTOR_CONTROLLER_LEFT"));
				rollerGripperMotorControllerRight = new VictorSP ((int) config.get("ROLLER_GRIPPER_MOTOR_CONTROLLER_RIGHT"));
			}
			else // if (!Robot.config.RobotA)
			{
				chassisMotorControllerLeft1 = new Talon ((int) config.get("CHASSIS_MOTOR_CONTROLLER_LEFT_1"));
				chassisMotorControllerLeft2 = new Talon ((int) config.get("CHASSIS_MOTOR_CONTROLLER_LEFT_2"));
				
				chassisMotorControllerRight1 = new Talon ((int) config.get("CHASSIS_MOTOR_CONTROLLER_RIGHT_1"));
				chassisMotorControllerRight2 = new Talon ((int) config.get("CHASSIS_MOTOR_CONTROLLER_RIGHT_2"));
				
				chassisMotorControllerCenter1 = new Talon((int) config.get("CHASSIS_MOTOR_CONTROLLER_CENTER_1"));
				chassisMotorControllerCenter2 = new Talon((int) config.get("CHASSIS_MOTOR_CONTROLLER_CENTER_2"));
				/*
				 * Anschluss
				 */
				anschlussMotorController = new Talon ((int) config.get("ANSCHLUSS_MOTOR_CONTROLLER"));
				
				/*
				 * Roller-Gripper
				 */
				rollerGripperMotorControllerLeft = new Talon ((int) config.get("ROLLER_GRIPPER_MOTOR_CONTROLLER_LEFT"));
				rollerGripperMotorControllerRight = new Talon ((int) config.get("ROLLER_GRIPPER_MOTOR_CONTROLLER_RIGHT"));
			}
			
			/*
			 * Stacker
			 */
			stackerSolenoidUpper = new DoubleSolenoid((int) config.get("STACKER_SOLENOID_UPPER_FORWARD"), 
														 (int) config.get("STACKER_SOLENOID_UPPER_REVERSE"));
			
			stackerSolenoidBottom = new DoubleSolenoid((int) config.get("STACKER_SOLENOID_BOTTOM_FORWARD"), 
														 (int) config.get("STACKER_SOLENOID_BOTTOM_REVERSE"));
			
			// Solenoid used to control the pistons connected to the gripper
			stackerSolenoidGripper = new DoubleSolenoid((int) config.get("STACKER_SOLENOID_GRIPPER_FORWARD"),
														  (int) config.get("STACKER_SOLENOID_GRIPPER_REVERSE"));
			
			// Solenoid used to control the pistons holding the container
			stackerSolenoidContainer = new DoubleSolenoid((int) config.get("STACKER_SOLENOID_CONTAINER_FORWARD"),
														  (int) config.get("STACKER_SOLENOID_CONTAINER_REVERSE"));
		}
		catch (ConfigException e) 
    	{
			logger.severe(e);
		}
	}
}
