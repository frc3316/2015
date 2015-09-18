package org.usfirst.frc.team3316.robot.stacker.commands;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.logger.DBugLogger;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MoveStackerManually extends Command
{
	double v;
	DBugLogger logger = Robot.logger;

	public MoveStackerManually()
	{
		requires(Robot.stacker);
	}

	// Called just before this Command runs the first time
	protected void initialize()
	{
		logger.fine(this.getName() + " initialize");
		Robot.stacker.allowStackMovement();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute()
	{
		v = Robot.joysticks.joystickOperator.getRawAxis(5);
		Robot.stacker.setMotors(v);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished()
	{
		return false;
	}

	// Called once after isFinished returns true
	protected void end()
	{
		Robot.stacker.setMotors(0);
	 	Robot.stacker.disallowStackMovement();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted()
	{
	}


}
