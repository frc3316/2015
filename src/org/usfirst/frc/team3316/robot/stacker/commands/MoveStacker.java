package org.usfirst.frc.team3316.robot.stacker.commands;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.config.Config;
import org.usfirst.frc.team3316.robot.logger.DBugLogger;
import org.usfirst.frc.team3316.robot.utils.GamePieceCollected;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public abstract class MoveStacker extends Command 
{
	DBugLogger logger = Robot.logger;
	Config config = Robot.config;
	
	GamePieceCollected gp;
	
	boolean setSuccesful;
	int countUnsuccesful;
	
    public MoveStacker()
    {
        requires(Robot.stacker);
    }

    protected void initialize()
    {
    	logger.fine(this.getName() + " initialized");
    	
    	setSuccesful = false;
    	
    	gp = Robot.rollerGripper.getGamePieceCollected();
    	
    	prepareSolenoids();
    	
    }

    protected void execute() 
    {
    	setSuccesful = setSolenoids();
    }

    protected boolean isFinished()
    {
    	return setSuccesful;
    }
    
    protected void end() 
    {
    	logger.fine(this.getName() + "ended");
    }

    protected void interrupted() 
    {
    	logger.fine(this.getName() + "interrupted");
    }
    
    /**
     * This method should be used for any preparation that is needed for the set solenoids
     * It should only contain methods that are certain to not fail
     */
    protected abstract void prepareSolenoids();
    
    /**
     * This method should move the stacker
     * @return whether the solenoids were successfully set
     */
    protected abstract boolean setSolenoids(); 
}
