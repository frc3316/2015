package org.usfirst.frc.team3316.robot.chassis.commands;

import edu.wpi.first.wpilibj.command.Command;

public class DisablePIDRotation extends Command
{

	@Override
	protected void initialize()
	{
		RobotOrientedDrive.activate(false);
	}

	@Override
	protected void execute()
	{
		// TODO Auto-generated method stub

	}

	@Override
	protected boolean isFinished()
	{
		// TODO Auto-generated method stub
		return true;
	}

	@Override
	protected void end()
	{
		// TODO Auto-generated method stub

	}

	@Override
	protected void interrupted()
	{
		// TODO Auto-generated method stub

	}

}
