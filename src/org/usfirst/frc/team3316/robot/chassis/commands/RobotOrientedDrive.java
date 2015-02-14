package org.usfirst.frc.team3316.robot.chassis.commands;

import org.usfirst.frc.team3316.robot.config.Config.ConfigException;

public class RobotOrientedDrive extends StrafeDrive 
{	
	double turnScale;
	
	protected void set ()
	{
		setRobotVector(getRightX(), getRightY());
		setRotation(getLeftX());
	}
	
	protected void setRobotVector (double x, double y)
	{
		this.left = y;
		this.right = y;
		this.center = x;
	}
	
	protected void setRotation (double requiredTurn)
	{
		//NOTE: Required turn cannot be larger than yIn.
		
		/*
		 * the following code sets left and right so that:
		 * for turning clockwise, left > right
		 * for turning counter-clockwise, right > left
		 * left - right = requiredTurn*2
		 * -1 < left, right < 1 
		 */
		updateTurnScale();
		requiredTurn *= turnScale;
		
		this.left += requiredTurn;
		this.right -= requiredTurn;
		
		double max = Math.max(this.left, this.right);
		double min = Math.min(this.left, this.right);
		
		double excess = 0;
		if (max > 1)
		{
			excess = (max - 1);
		}
		else if (min < -1)
		{
			excess = (min + 1);
		}
		
		this.left -= excess;
		this.right -= excess;
	}
	
	private void updateTurnScale ()
	{
		try 
		{
			turnScale = (double)config.get("chassis_RobotOrientedDrive_TurnScale");
		}
		catch (ConfigException e)
		{
			logger.severe(e);
		}
	}
}
