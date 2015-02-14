package org.usfirst.frc.team3316.robot.config;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.BufferedReader;
import java.io.FileReader;
import java.util.Hashtable;
import java.util.Map.Entry;
import java.util.Set;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.logger.DBugLogger;

public class Config 
{
	DBugLogger logger = Robot.logger;
	
	public boolean robotA; //true if it's robot A, false if it's robot B
	
	/**
	 * Exception that is raised when a certain key is not found in config
	 */
	public class ConfigException extends Exception
	{
		private static final long serialVersionUID = -658181374612523772L;
		
		public ConfigException (String key)
		{
			super(key);
		}
	}
	
	private static Hashtable <String, Object> variables;
	private static Hashtable <String, Object> constants;
	
	public Config ()
	{
		determineRobotA();
		deserializationInit();	
	}
	
	/*
	 * Reads the file on the roborio that says whether it is robot A or B
	 * Default is Robot B
	 */
	private void determineRobotA ()
	{
		String line;
		BufferedReader in;
		
		try 
		{
			in = new BufferedReader(new FileReader("/home/lvuser/RobotName/robot.txt"));
			line = in.readLine();
			
			if (line.equals("Robot A"))
			{
				robotA = true;
				logger.info(" This is Robot A");
			}
			else
			{
				robotA = false;
				logger.info(" This is Robot B");
			}
		} 
		catch (FileNotFoundException e) 
		{
			logger.severe(e);
			System.exit(1);
		} 
		catch (IOException e) 
		{
			logger.severe(e);
		}
	}
	
	public void add (String key, Object value)
	{
		addToVariables(key, value);
	}
	
	/**
	 * Returns the value attached to a requested key
	 * @param key the key to look for
	 * @return returns the corresponding value
	 * @throws ConfigException if the key does not exist
	 */
	public Object get (String key) throws ConfigException
	{
		if (constants.containsKey(key))
		{
			return constants.get(key);
		}
		else if (variables.containsKey(key))
		{
			return variables.get(key);
		}
		else
		{
			throw new ConfigException(key);
		}
	}
	
	private void addToVariables (String key, Object value)
	{
		if (variables.containsKey(key))
		{
			variables.replace(key, value);
			logger.info("replaced " + key + " in variables hashtable");
		}
		else
		{
			variables.put(key, value);
			logger.info("added " + key + " in variables hashtable");
		}
	}
	
	@SuppressWarnings("unchecked")
	private void deserializationInit () 
	{
<<<<<<< HEAD
		String configPath;
		
		if (robotA)
		{
			configPath = "/home/lvuser/config/configFileA.ser";
		}
		else
		{
			configPath = "/home/lvuser/config/configFileB.ser";
		}
		
		try 
		{
			FileInputStream in = new FileInputStream(configPath);
			ObjectInputStream input = new ObjectInputStream(in);
			
			constants = (Hashtable <String, Object>) input.readObject();
			variables = (Hashtable <String, Object>) input.readObject();
			
			Set <Entry <String, Object> > set;
			
			set = constants.entrySet();
			logger.info(" Logging Constants");
			for (Entry <String, Object> entry : set)
			{
				logger.info(" Key = " + entry.getKey() + " Value = " + entry.getValue());
			}
			
			set = variables.entrySet();
			logger.info(" Logging Variables");
			for (Entry <String, Object> entry : set)
			{
				logger.info(" Key = " + entry.getKey() + " Value = " + entry.getValue());
			}
			
			input.close();
		} 
		catch (Exception e) 
		{
			logger.severe(e);
		}
=======
		/*
		 * Human IO
		 */
			/*
			 * Constants
			 */
			addToConstants("JOYSTICK_LEFT", 0);
			addToConstants("JOYSTICK_RIGHT", 1);
			addToConstants("JOYSTICK_OPERATOR", 2);
			
			//subsystem
				/*
				 * Stacker
				 */
				addToConstants("BUTTON_MOVE_STACKER_TO_FLOOR", 1);
				addToConstants("BUTTON_MOVE_STACKER_TO_STEP", 2);
				addToConstants("BUTTON_MOVE_STACKER_TO_TOTE", 4);
				
				addToConstants("BUTTON_HOLD_CONTAINER", 10);
				addToConstants("BUTTON_RELEASE_CONTAINER", 9);
				
				addToConstants("BUTTON_OPEN_GRIPPER", 5);
				addToConstants("BUTTON_CLOSE_GRIPPER", 6);
				
				/*
				 * Roller-Gripper
				 */
				addToConstants("BUTTON_ROLL_IN", 180);
				addToConstants("BUTTON_ROLL_OUT", 0);
				addToConstants("BUTTON_ROLL_TURN_CLOCKWISE", 90);
				addToConstants("BUTTON_ROLL_TURN_COUNTER_CLOCKWISE", 270);
				
				/*
				 * Anschluss
				 */
				addToConstants("BUTTON_OPEN_ANSCHLUSS", 8);
				addToConstants("BUTTON_CLOSE_ANSCHLUSS", 7);
		/*
		 * Chassis
		 */
			/*
			 * Constants
			 */
			addToConstants("CHASSIS_MOTOR_CONTROLLER_LEFT_1", 6);
			addToConstants("CHASSIS_MOTOR_CONTROLLER_LEFT_2", 7);
			
			addToConstants("CHASSIS_MOTOR_CONTROLLER_RIGHT_1", 1);
			addToConstants("CHASSIS_MOTOR_CONTROLLER_RIGHT_2", 2);
			
			addToConstants("CHASSIS_MOTOR_CONTROLLER_CENTER_1", 0);
			addToConstants("CHASSIS_MOTOR_CONTROLLER_CENTER_2", 5);
			
			addToConstants("CHASSIS_ENCODER_LEFT_A", 4);
			addToConstants("CHASSIS_ENCODER_LEFT_B", 5);
			
			addToConstants("CHASSIS_ENCODER_RIGHT_A", 2);
			addToConstants("CHASSIS_ENCODER_RIGHT_B", 3);
			
			addToConstants("CHASSIS_ENCODER_CENTER_A", 0);
			addToConstants("CHASSIS_ENCODER_CENTER_B", 1);
			
			addToConstants("CHASSIS_ENCODER_LEFT_DISTANCE_PER_PULSE", -0.0018702293765902);
			addToConstants("CHASSIS_ENCODER_RIGHT_DISTANCE_PER_PULSE", 0.0018702293765902);
			addToConstants("CHASSIS_ENCODER_CENTER_DISTANCE_PER_PULSE", 0.0012468195843934);
			
			addToConstants("CHASSIS_WIDTH", 0.5322); //in meters
			/*
			 * Variables
			 */

			 //Subsystem
			addToVariables("chassis_LeftScale", 1.0);
			addToVariables("chassis_RightScale", -1.0);
			addToVariables("chassis_CenterScale", 1.0);
			
			addToVariables("chassis_HeadingToSet", 0.0); // This is the heading that the SetHeadingSDB command sets to
														 // It is configurable in the SDB (it should appear in initSDB())
			//TankDrive
			addToVariables("chassis_TankDrive_InvertX", false);
			addToVariables("chassis_TankDrive_InvertY", true);
			
			addToVariables("chassis_TankDrive_LowPass", 0.0);
			//RobotOrientedDrive
			addToVariables("chassis_RobotOrientedDrive_TurnScale", 1.0);
			
			//RobotOrientedNavigation
			addToVariables("chassis_RobotOrientedNavigation_PIDControllerX_KP", 0.0);
			addToVariables("chassis_RobotOrientedNavigation_PIDControllerX_KI", 0.0);
			addToVariables("chassis_RobotOrientedNavigation_PIDControllerX_KD", 0.0);
			
			addToVariables("chassis_RobotOrientedNavigation_PIDControllerY_KP", 0.0);
			addToVariables("chassis_RobotOrientedNavigation_PIDControllerY_KI", 0.0);
			addToVariables("chassis_RobotOrientedNavigation_PIDControllerY_KD", 0.0);
			
			addToVariables("chassis_RobotOrientedNavigation_PIDControllerHeading_KP", 0.0);
			addToVariables("chassis_RobotOrientedNavigation_PIDControllerHeading_KI", 0.0);
			addToVariables("chassis_RobotOrientedNavigation_PIDControllerHeading_KD", 0.0);
			
			addToVariables("chassis_RobotOrientedNavigation_PIDControllerX_AbsoluteTolerance", 0.0);
			addToVariables("chassis_RobotOrientedNavigation_PIDControllerY_AbsoluteTolerance", 0.0);
			addToVariables("chassis_RobotOrientedNavigation_PIDControllerHeading_AbsoluteTolerance", 0.0);
			
			addToVariables("chassis_RobotOrientedNavigation_PIDControllerX_MinimumOutput", -1.0);
			addToVariables("chassis_RobotOrientedNavigation_PIDControllerX_MaximumOutput", 1.0);
			
			addToVariables("chassis_RobotOrientedNavigation_PIDControllerY_MinimumOutput", -1.0);
			addToVariables("chassis_RobotOrientedNavigation_PIDControllerY_MaximumOutput", 1.0);
			
			addToVariables("chassis_RobotOrientedNavigation_PIDControllerHeading_MinimumOutput", -1.0);
			addToVariables("chassis_RobotOrientedNavigation_PIDControllerHeading_MaximumOutput", 1.0);
		/*
		 * Anschluss
		 */
			/*
			 * Constants
			 */
			addToConstants("ANSCHLUSS_MOTOR_CONTROLLER", 4);
			
			addToConstants("ANSCHLUSS_DIGITAL_INPUT_CLOSED", 11);
			addToConstants("ANSCHLUSS_DIGITAL_INPUT_OPENED", 10);
			
			
			
			/*
			 * Variables
			 */
			addToVariables("anschluss_CloseAnschluss_MotorSpeed", -1.0);
			addToVariables("anschluss_OpenAnschluss_MotorSpeed", 1.0);
		/*
		 * Roller Gripper
		 */
			
			/*
			 * Constants
			 */
			addToConstants("ROLLER_GRIPPER_GAME_PIECE_IR", 1);
			addToConstants("ROLLER_GRIPPER_SWITCH_GAME_PIECE", 7);
			
			addToConstants("ROLLER_GRIPPER_MOTOR_CONTROLLER_LEFT", 8);
			addToConstants("ROLLER_GRIPPER_MOTOR_CONTROLLER_RIGHT", 3);
			
			/*
			 * Variables
			 */
				//Subsystem
					addToVariables("rollerGripper_LeftScale", 1.0);
					addToVariables("rollerGripper_RightScale", -1.0);
					
					addToVariables("rollerGripper_ToteDistanceMinimum", 0.325);
					addToVariables("rollerGripper_ToteDistanceMaximum", 0.34);
					
					addToVariables("rollerGripper_ContainerDistanceMinimum", 0.39);
					addToVariables("rollerGripper_ContainerDistanceMaximum", 0.41);
					
					addToVariables("rollerGripper_GamePieceDistanceThreshold", 1.0);
					
				//RollIn
					addToVariables("rollerGripper_RollIn_SpeedLeft", 1.0);
					addToVariables("rollerGripper_RollIn_SpeedRight", 1.0);
			
				//RollOut
					addToVariables("rollerGripper_RollOut_SpeedLeft", -1.0);
					addToVariables("rollerGripper_RollOut_SpeedRight", -1.0);
			
				//RollTurnClockwise
					addToVariables("rollerGripper_RollTurnClockwise_SpeedLeft", -1.0);
					addToVariables("rollerGripper_RollTurnClockwise_SpeedRight", 1.0);
			
				//RollTurnCounterClockwise
					addToVariables("rollerGripper_RollTurnCounterClockwise_SpeedLeft", 1.0);
					addToVariables("rollerGripper_RollTurnCounterClockwise_SpeedRight", -1.0);
					
				//RollJoystick
					addToVariables("rollerGripper_RollJoystick_ChannelX", 0);
					addToVariables("rollerGripper_RollJoystick_ChannelY", 1);
					
					addToVariables("rollerGripper_RollJoystick_InvertX", true);
					addToVariables("rollerGripper_RollJoystick_InvertY", false);
					
					addToVariables("rollerGripper_RollJoystick_LowPass", 0.15);
					
		/*
		 * Stacker
		 */
			/*
			 * Constants
			 */
				//Subsystem
					addToConstants("STACKER_SOLENOID_UPPER_FORWARD", 6);
					addToConstants("STACKER_SOLENOID_UPPER_REVERSE", 7);
					
					addToConstants("STACKER_SOLENOID_BOTTOM_FORWARD", 4);
					addToConstants("STACKER_SOLENOID_BOTTOM_REVERSE", 5);
					
					addToConstants("STACKER_SOLENOID_CONTAINER_FORWARD", 3);
					addToConstants("STACKER_SOLENOID_CONTAINER_REVERSE", 2);
					
					addToConstants("STACKER_SOLENOID_GRIPPER_FORWARD", 0);
					addToConstants("STACKER_SOLENOID_GRIPPER_REVERSE", 1);
					
					addToConstants("STACKER_IR", 0);
					

					addToConstants("SWITCH_RATCHET_LEFT", 7);
					addToConstants("SWITCH_RATCHET_RIGHT", 8);
					
			/*
			 * Variables
			 */
				//Subsystem
					
					//TODO: re-determine these values 
					addToVariables("stacker_HeightFloorMinimum", 0.4);
					addToVariables("stacker_HeightFloorMaximum", 0.42);
					
					addToVariables("stacker_HeightToteMinimum", 1.45);
					addToVariables("stacker_HeightToteMaximum", 1.62);
					
					addToVariables("stacker_HeightStepMinimum", 0.95);
					addToVariables("stacker_HeightStepMaximum", 1.05);
					
					addToVariables("stacker_HeightStuckOnContainerMinimum", 1.05);
					addToVariables("stacker_HeightStuckOnContainerMaximum", 1.05);
>>>>>>> Finished adding PickupSequence and fixed MoveStacker commmands
	}
}
