package org.usfirst.frc.team3316.robot.sequences;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousSequenceWithoutDrive extends CommandGroup {
    
    public  AutonomousSequenceWithoutDrive() {
        addSequential(new AutoRollIn());
    }
}
