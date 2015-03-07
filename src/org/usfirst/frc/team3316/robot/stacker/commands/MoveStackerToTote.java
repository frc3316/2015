package org.usfirst.frc.team3316.robot.stacker.commands;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.utils.GamePieceCollected;

/**
 * Moves stacker to tote position while checking
 * for mechanical safety constraints. 
 */
public class MoveStackerToTote extends MoveStacker
{
	protected void prepareSolenoids()
	{
		if (gp == GamePieceCollected.Container)
		{
			//If there is a container at floor position, it might colide with the roller gripper
			logger.fine("YES container in roller gripper");
			Robot.stacker.openSolenoidContainer();
		}
		else
		{
			logger.fine("NO container in the roller gripper");
		}
		
		// The stacker will colide with the roller gripper.
		Robot.stacker.openSolenoidGripper();
	}
	
	protected boolean setSolenoids() 
	{
		return (Robot.stacker.closeSolenoidUpper() && Robot.stacker.closeSolenoidBottom());
	}
}
