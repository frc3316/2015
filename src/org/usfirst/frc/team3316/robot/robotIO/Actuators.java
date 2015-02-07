/**
 * Le robot actuators
 */
package org.usfirst.frc.team3316.robot.robotIO;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.config.Config;
import org.usfirst.frc.team3316.robot.config.Config.ConfigException;
import org.usfirst.frc.team3316.robot.logger.DBugLogger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.VictorSP;

public class Actuators 
{
	Config config = Robot.config;
	DBugLogger logger = Robot.logger;
	
	/*
	 * Chassis
	 */
	public VictorSP chassisMotorControllerLeft1, chassisMotorControllerLeft2;
	public VictorSP chassisMotorControllerRight1, chassisMotorControllerRight2;
	public VictorSP chassisMotorControllerCenter1, chassisMotorControllerCenter2;
	
	/*
	 * Stacker
	 */
	public DoubleSolenoid stackerSolenoidUpperLeft;
	public DoubleSolenoid stackerSolenoidUpperRight;
	
	public DoubleSolenoid stackerSolenoidBottomLeft;
	public DoubleSolenoid stackerSolenoidBottomRight;
	
	public DoubleSolenoid stackerSolenoidContainer;
	public DoubleSolenoid stackerSolenoidGripper;
	
	/*
	 * Anschluss
	 */
	public VictorSP anschlussMotorController;
	
	/*
	 * Roller-Gripper
	 */
	public VictorSP rollerGripperMotorControllerLeft;
	public VictorSP rollerGripperMotorControllerRight;
	
	public Actuators ()
	{
		try 
		{
			/*
			 * Chassis
			 */
			chassisMotorControllerLeft1 = new VictorSP ((int) config.get("CHASSIS_MOTOR_CONTROLLER_LEFT_1"));
			chassisMotorControllerLeft2 = new VictorSP ((int) config.get("CHASSIS_MOTOR_CONTROLLER_LEFT_2"));
			
			chassisMotorControllerRight1 = new VictorSP ((int) config.get("CHASSIS_MOTOR_CONTROLLER_RIGHT_1"));
			chassisMotorControllerRight2 = new VictorSP ((int) config.get("CHASSIS_MOTOR_CONTROLLER_RIGHT_2"));
			
			chassisMotorControllerCenter1 = new VictorSP((int) config.get("CHASSIS_MOTOR_CONTROLLER_CENTER_1"));
			chassisMotorControllerCenter2 = new VictorSP((int) config.get("CHASSIS_MOTOR_CONTROLLER_CENTER_2"));
			
			/*
			 * Stacker
			 */
			stackerSolenoidUpperLeft = new DoubleSolenoid((int) config.get("STACKER_SOLENOID_UPPER_LEFT_FORWARD"), 
														 (int) config.get("STACKER_SOLENOID_UPPER_LEFT_REVERSE"));
			
			stackerSolenoidUpperRight = new DoubleSolenoid((int) config.get("STACKER_SOLENOID_UPPER_RIGHT_FORWARD"), 
					 									  (int) config.get("STACKER_SOLENOID_UPPER_RIGHT_REVERSE"));
			
			stackerSolenoidBottomLeft = new DoubleSolenoid((int) config.get("STACKER_SOLENOID_BOTTOM_LEFT_FORWARD"), 
														 (int) config.get("STACKER_SOLENOID_BOTTOM_LEFT_REVERSE"));
			
			stackerSolenoidBottomRight = new DoubleSolenoid((int) config.get("STACKER_SOLENOID_BOTTOM_RIGHT_FORWARD"), 
														  (int) config.get("STACKER_SOLENOID_BOTTOM_RIGHT_REVERSE"));
			
			// Solenoid used to control the pistons connected to the gripper
			stackerSolenoidGripper = new DoubleSolenoid((int) config.get("STACKER_SOLENOID_GRIPPER_FORWARD"),
														  (int) config.get("STACKER_SOLENOID_GRIPPER_REVERSE"));
			
			// Solenoid used to control the pistons holding the container
			stackerSolenoidContainer = new DoubleSolenoid((int) config.get("STACKER_SOLENOID_CONTAINER_FORWARD"),
														  (int) config.get("STACKER_SOLENOID_CONTAINER_REVERSE"));
			
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
		catch (ConfigException e) 
    	{
			logger.severe(e);
		}
	}
}
