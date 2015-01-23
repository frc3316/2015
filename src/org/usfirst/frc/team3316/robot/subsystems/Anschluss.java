/**
 * Anschluss subsystem
 */
package org.usfirst.frc.team3316.robot.subsystems;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.config.Config;
import org.usfirst.frc.team3316.robot.logger.DBugLogger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Anschluss extends Subsystem {
    
	Config config = Robot.config;
	DBugLogger logger = Robot.logger;
	
	private VictorSP anschluss;
	
	DigitalInput hallEffectClosed;
	DigitalInput hallEffectOpened;
	
	public Anschluss ()	{
		anschluss = Robot.actuators.anschluss;
		hallEffectClosed = Robot.sensors.anschlussDigitalInputClosed;
		hallEffectOpened = Robot.sensors.anschlussDigitalInputOpened;
	}
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void set(double motorSpeed) {
		if(motorSpeed > 0) {
			if(!Robot.anschluss.isOpened()) {
				anschluss.set(motorSpeed);
			}
		}
		else if(motorSpeed < 0) {
			if(!Robot.anschluss.isClosed()) {
				anschluss.set(motorSpeed);
			}
		}
		anschluss.set(0);
    }
    
    public boolean isClosed() {
    	return hallEffectClosed.get();
    }
    
    public boolean isOpened() {
    	return hallEffectOpened.get();
    }
    
}
