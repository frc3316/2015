package org.usfirst.frc.team3316.robot.rollerGripper.commands;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.config.Config.ConfigException;

public class RollTurnCounterClockwise extends Roll{

	protected void setSpeed() 
    {
    	try
    	{
			speedLeft = (double) config.get("rollerGripper_RollTurnCounterClockwise_Speed_Left");
			speedRight = (double) config.get("rollerGripper_RollTurnCounterClockwise_Speed_Right");
		} 
    	catch (ConfigException e) 
    	{
			logger.severe(e);
		}
    }
}
