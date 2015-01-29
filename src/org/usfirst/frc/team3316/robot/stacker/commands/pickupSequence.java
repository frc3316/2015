package org.usfirst.frc.team3316.robot.stacker.commands;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.rollerGripper.commands.RollIn;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class pickupSequence extends CommandGroup 
{    
    public pickupSequence() 
    {
        addSequential(new RollIn());
        addSequential(new MoveStackerToFloor());
        addSequential(new MoveStackerToTote());
        
    }
}
