package org.usfirst.frc.team3316.robot.config;

import java.util.Hashtable;

public class Config 
{
	public class ConfigException extends Exception
	{
		/**
		 * 
		 */
		private static final long serialVersionUID = 1L;

		public ConfigException (String key)
		{
			super(key);
		}
	}
	
	private static Hashtable<String, Object> variables;
	private static Hashtable<String, Object> constants;
	
	public Config ()
	{
		if (variables == null || constants == null)
		{
			variables = new Hashtable<String, Object>();
			constants = new Hashtable<String, Object>();
			initConfig();
		}
	}
	
	public void add (String key, Object value)
	{
		addToVariables(key, value);
	}
	
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
	
	private void addToConstants (String key, Object value)
	{
		if (constants.containsKey(key))
		{
			constants.replace(key, value);
		}
		else
		{
			constants.put(key, value);
		}
	}
	
	private void addToVariables (String key, Object value)
	{
		if (variables.containsKey(key))
		{
			variables.replace(key, value);
		}
		else
		{
			variables.put(key, value);
		}
	}
	
	private void initConfig ()
	{
		addToConstants("joystickLeft", 0);
		addToConstants("joystickRight", 1);
		addToConstants("joystickOperator", 2);
		addToConstants("chassisLeft", 0);
		addToConstants("chassisRight", 1);
		
		addToVariables("driveMaxSpeed", 1);
		addToVariables("driveMinSpeed", -1);
		
		addToVariables("chassisLeftScale", -1);
		addToVariables("chassisRightScale", 1);
		addToVariables("chassisCenterScale", 1);
		/*
		 * Roller Gripper
		 */
		
		/*
		 * Constants
		 */
			//Subsystem
				addToConstants("ROLLER_GRIPPER_IR_LEFT", 0);
				addToConstants("ROLLER_GRIPPER_IR_RIGHT", 1);
		
		/*
		 * Variables
		 */
			//Subsystem
				addToVariables("rollerGripper_LeftScale", -1);
				addToVariables("rollerGripper_RightScale", 1);
			//RollIn
				addToVariables("rollerGripper_RollIn_SpeedLeft", 0.5);
				addToVariables("rollerGripper_RollIn_SpeedRight", 0.5);
		
			//RollOut
				addToVariables("rollerGripper_RollOut_SpeedLeft", -0.5);
				addToVariables("rollerGripper_RollOut_SpeedRight", -0.5);
		
			//RollTurnClockwise
				addToVariables("rollerGripper_RollTurnClockwise_SpeedLeft", 0.5);
				addToVariables("rollerGripper_RollTurnClockwise_SpeedRight", -0.5);
		
			//RollTurnCounterClockwise
				addToVariables("rollerGripper_RollTurnCounterClockwise_SpeedLeft", 0.5);
				addToVariables("rollerGripper_RollTurnCounterClockwise_SpeedRight", -0.5);
	}
}
