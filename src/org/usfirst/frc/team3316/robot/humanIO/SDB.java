/**
 * Class for managing the SmartDashboard data
 */
package org.usfirst.frc.team3316.robot.humanIO;

import java.util.Hashtable;
import java.util.Map.Entry;
import java.util.Set;
import java.util.TimerTask;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.chassis.commands.StrafeDrive;
import org.usfirst.frc.team3316.robot.chassis.commands.StartIntegrator;
import org.usfirst.frc.team3316.robot.chassis.commands.TankDrive;
import org.usfirst.frc.team3316.robot.chassis.heading.SetHeadingPreset;
import org.usfirst.frc.team3316.robot.chassis.heading.SetHeadingSDB;
import org.usfirst.frc.team3316.robot.chassis.commands.RobotOrientedDrive;
import org.usfirst.frc.team3316.robot.config.Config;
import org.usfirst.frc.team3316.robot.config.Config.ConfigException;
import org.usfirst.frc.team3316.robot.logger.DBugLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SDB 
{
	/*
	 * Runnable that periodically updates values from the robot into the SmartDashboard
	 * This is the place where all of the robot data should be displayed from
	 */
	private class UpdateSDBTask extends TimerTask
	{
		public UpdateSDBTask ()
		{
			logger.info("Created UpdateSDBTask");
		}
		public void run ()
		{
			put ("Current Heading", Robot.chassis.getHeading());
			put ("Current Height", Robot.stacker.getHeight());
			
			put ("Current GP distance", Robot.rollerGripper.getGPIRDistance());
			
			put ("Angular Velocity", Robot.chassis.getAngularVelocity());
			put ("Angular Velocity Encoders", Robot.chassis.getAngularVelocityEncoders());
			
			put ("Left Ratchet", Robot.stacker.getSwitchLeft());
			put ("Right Ratchet", Robot.stacker.getSwitchRight());
			
			put ("Game Piece Switch", Robot.rollerGripper.getSwitchGP());
			
			put ("Distance Left", Robot.chassis.getDistanceLeft());
			put ("Distance Right", Robot.chassis.getDistanceRight());
			put ("Distance Center", Robot.chassis.getDistanceCenter());
			
			put ("Speed Left", Robot.chassis.getSpeedLeft());
			put ("Speed Right", Robot.chassis.getSpeedRight());
			put ("Speed Center", Robot.chassis.getSpeedCenter());
			
			/*
			 * Integrator testing
			 * should be removed
			 */
			put ("Integrator X", Robot.chassis.navigationIntegrator.getX());
			put ("Integrator Y", Robot.chassis.navigationIntegrator.getY());
			put ("Integrator Heading", Robot.chassis.navigationIntegrator.getHeading());
			
			logger.info("Finished UpdateSDBTask run");
		}
		
		private void put (String name, double d)
	    {
	    	SmartDashboard.putNumber(name, d);
	    }
	    
	    private void put (String name, int i)
	    {
	    	SmartDashboard.putInt(name, i);
	    }
	    
	    private void put (String name, boolean b)
	    {
	    	SmartDashboard.putBoolean(name, b);
	    }
	}
	
	DBugLogger logger = Robot.logger;
	Config config = Robot.config;
	
	private UpdateSDBTask updateSDBTask;
	
	private Hashtable <String, Class <?> > variablesInSDB;
	
	public SDB ()
	{
		variablesInSDB = new Hashtable <String, Class <?> > ();
		
		SmartDashboard.putData(new UpdateVariablesInConfig());
		SmartDashboard.putData(new SetHeadingSDB());
		
		initSDB();
		logger.info("Finished initSDB()");
	}
	
	public void timerInit ()
	{
		updateSDBTask = new UpdateSDBTask();
		Robot.timer.schedule(updateSDBTask, 0, 20);
	}
	
	/**
	 * Adds a certain key in the config to the SmartDashboard
	 * @param key the key required
	 * @return whether the value was put in the SmartDashboard
	 */
	public boolean putConfigVariableInSDB (String key)
	{
		try
		{
			Object value = config.get(key);
			Class <?> type = value.getClass();
			
			boolean constant = Character.isUpperCase(key.codePointAt(0));
			
			if (type == Double.class)
			{
				SmartDashboard.putNumber(key, (double) value);
			}
			else if (type == Integer.class)
			{
				SmartDashboard.putNumber(key, (int) value);
			}
			else if (type == Boolean.class)
			{
				SmartDashboard.putBoolean(key, (boolean) value);
			}
			
			if (!constant)
			{
				variablesInSDB.put(key, type);
				logger.info("Added to SDB " + key + " of type " + type + 
						"and allows for its modification");
			}
			else
			{
				logger.info("Added to SDB " + key + " of type " + type + 
						"BUT DOES NOT ALLOW for its modification");
			}
			
			return true;
		}
		catch (ConfigException e)
		{
			logger.severe(e);
		}
		return false;
	}
	
	public Set <Entry <String, Class <?> > > getVariablesInSDB ()
	{
		return variablesInSDB.entrySet();
	}
	
	private void initSDB ()
	{	
		SmartDashboard.putData(new UpdateVariablesInConfig()); // NEVER REMOVE THIS COMMAND
		
		SmartDashboard.putData(new TankDrive());
		SmartDashboard.putData(new StrafeDrive());
		SmartDashboard.putData(new RobotOrientedDrive());
		SmartDashboard.putData(new StartIntegrator()); //For integrator testing, should be removed
		
		SmartDashboard.putData(new TankDrive()); //should be removed
		SmartDashboard.putData(new RobotOrientedDrive()); //should be removed
		
		/*
		 * Set Heading SDB
		 */
		SmartDashboard.putData("Zero Heading", new SetHeadingPreset(0));
		
		SmartDashboard.putData(new SetHeadingSDB());
		putConfigVariableInSDB("chassis_HeadingToSet");
		
		/*
		 * Roller Gripper
		 */
		putConfigVariableInSDB("rollerGripper_RollJoystick_ChannelX");
		putConfigVariableInSDB("rollerGripper_RollJoystick_ChannelY");
		
		putConfigVariableInSDB("rollerGripper_RollJoystick_InvertX");
		putConfigVariableInSDB("rollerGripper_RollJoystick_InvertY");
		
		putConfigVariableInSDB("rollerGripper_RollJoystick_LowPass");
		
	}
}