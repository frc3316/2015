package org.usfirst.frc.team3316.robot.anschluss.commands;

import java.io.PrintWriter;
import java.io.StringWriter;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.config.Config;
import org.usfirst.frc.team3316.robot.config.Config.ConfigException;
import org.usfirst.frc.team3316.robot.logger.DBugLogger;

import edu.wpi.first.wpilibj.command.Command;

/**
 *Command that closes the anschluss
 */
public class closeAnschluss extends Command 
{
	Config config = Robot.config;
	DBugLogger logger = Robot.logger;
	double motorSpeed;
	
    public closeAnschluss() 
    {
    	requires(Robot.anschluss);
    }

    protected void initialize() {}

    protected void execute() 
    {
    	updateMotorSpeed();
    	Robot.anschluss.set(motorSpeed);
    }

    protected boolean isFinished() {
        return Robot.anschluss.isClosed();
    }

    protected void end() {}

    protected void interrupted() {}
    
    private void updateMotorSpeed() 
    {
    	try 
    	{
			motorSpeed = (double) config.get("closeAnschlussMotorSpeed");
		}
    	catch (ConfigException e) {
    		logger.severe(e);			
		}
    }
}