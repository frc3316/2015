package org.usfirst.frc.team3316.robot.chassis.commands;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.config.Config;
import org.usfirst.frc.team3316.robot.logger.DBugLogger;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public abstract class Drive extends Command 
{
	protected static Config config = Robot.config;
	protected static DBugLogger logger = Robot.logger;
	
	protected double left = 0, right = 0, center = 0;
	
    public Drive () 
    {
       requires(Robot.chassis);
    }

    protected void initialize() {}

    protected void execute() 
    {
    	set();
    	Robot.chassis.set(left, right, center);
    }

    protected boolean isFinished() 
    {
        return false;
    }

    protected void end() 
    {
    	Robot.chassis.set(0, 0, 0);
    }

    protected void interrupted() 
    {
    	end();
    }
    
    protected abstract void set ();
}
