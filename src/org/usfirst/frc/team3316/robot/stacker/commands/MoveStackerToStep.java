package org.usfirst.frc.team3316.robot.stacker.commands;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.stacker.StackerPosition;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 *
 */
public class MoveStackerToStep extends MoveStacker
{
	public MoveStackerToStep()
    {
		super("STACKER_MOVE_STACKER_TO_STEP_HEIGHT_MAX", 
				"STACKER_MOVE_STACKER_TO_STEP_HEIGHT_MIN");
    }

	protected void initialize()
	{
		/*
		 * If one of the ratchets is not in place and they should be pressed, dont start
		 */
		if (	Robot.stacker.getSolenoidContainer() == DoubleSolenoid.Value.kReverse &&
				Robot.stacker.getPosition() == StackerPosition.Floor &&
				(!Robot.stacker.getSwitchLeft() || !Robot.stacker.getSwitchRight())) 
		{
			this.cancel();
		}
		else
		{
			super.initialize();
		}
	}
	
	protected void setSolenoids() 
	{
		if(Robot.stacker.getPosition() == StackerPosition.Tote)
		{
			Robot.stacker.openSolenoidGripper();
		}
		Robot.stacker.closeSolenoidUpper();
    	Robot.stacker.openSolenoidBottom();
	}
}
