package org.usfirst.frc.team3316.robot.stacker.commands;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.rollerGripper.GamePieceCollected;
import org.usfirst.frc.team3316.robot.stacker.StackerPosition;

/**
 * Moves stacker to tote position while checking
 * for mechanical safety constraints. 
 */
public class MoveStackerToTote extends MoveStacker
{
	protected void initialize()
	{	
		logger.fine("MoveStackerToTote command initialize");

		GamePieceCollected gp = Robot.rollerGripper.getGamePieceCollected();
		if (gp == GamePieceCollected.Container)
		{
			//If there is a container at floor position, it might colide with the roller gripper
			logger.fine("YES container in roller gripper");
			Robot.stacker.openSolenoidGripper();
			Robot.stacker.openSolenoidContainer();
		}
		else
		{
			logger.fine("NO container in the roller gripper");
		}
		super.initialize();
	}
	protected void setSolenoids() 
	{
		Robot.stacker.closeSolenoidUpper();
		Robot.stacker.closeSolenoidBottom();
	}
}
