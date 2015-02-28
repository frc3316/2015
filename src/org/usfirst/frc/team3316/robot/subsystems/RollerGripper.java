package org.usfirst.frc.team3316.robot.subsystems;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.config.Config;
import org.usfirst.frc.team3316.robot.config.Config.ConfigException;
import org.usfirst.frc.team3316.robot.logger.DBugLogger;
import org.usfirst.frc.team3316.robot.rollerGripper.GamePieceCollected;
import org.usfirst.frc.team3316.robot.rollerGripper.commands.Roll;
import org.usfirst.frc.team3316.robot.rollerGripper.commands.RollJoystick;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class RollerGripper extends Subsystem 
{	
	Config config = Robot.config;
	DBugLogger logger = Robot.logger;
	
	//TODO: fix all of the private names to not start with gripper
	private SpeedController gripperLeft, gripperRight;
	
	private DigitalInput gripperSwitchGP;
	
	private DigitalInput switchRight;
	private DigitalInput switchLeft;
	
	private double leftScale, rightScale;
	
	private Roll defaultRoll;
	
    public RollerGripper () 
    {
    	gripperLeft = Robot.actuators.rollerGripperMotorControllerLeft;
    	gripperRight = Robot.actuators.rollerGripperMotorControllerRight;
    	
    	gripperSwitchGP = Robot.sensors.rollerGripperSwitchGP;
    	
    	switchRight = Robot.sensors.switchRatchetRight;
    	switchLeft = Robot.sensors.switchRatchetLeft;
    }

    public void initDefaultCommand() 
    {
    	defaultRoll = new RollJoystick();
    	setDefaultCommand(defaultRoll);
    }
    
    public boolean set (double speedLeft, double speedRight) 
    {
    	updateScales();
     	gripperLeft.set(speedLeft*leftScale);
    	gripperRight.set(speedRight*rightScale);
    	return true;
    }
    
    public boolean getSwitchRight ()
    {
    	return switchRight.get();
    }
    
    public boolean getSwitchLeft ()
    {
    	return switchLeft.get();
    }
    
    public boolean getSwitchGamePiece ()
    {
    	return gripperSwitchGP.get();
    }
    
    /**
     * Returns which gamepiece there is in the roller gripper (if any)
     * @return Tote for tote, Container for container, Unsure if there 
     * is something in but can't figure out which and none if empty 
     */
    public GamePieceCollected getGamePieceCollected ()
    {
    	boolean gpSwitch = getSwitchGamePiece();
    	boolean gpSwitchRight = getSwitchRight();
    	boolean gpSwitchLeft = getSwitchLeft();
    	
    	if (!gpSwitch) {
    		return GamePieceCollected.None;
    	
    	} else if (!gpSwitchRight && !gpSwitchLeft) {
    		return GamePieceCollected.Container;
    	
    	} else {
    		return GamePieceCollected.Tote;
    	}
    }
    
    private void updateScales ()
    {
    	try
    	{
    		leftScale = (double)config.get("rollerGripper_LeftScale");
    		rightScale = (double)config.get("rollerGripper_RightScale");
    	}
    	catch (ConfigException e)
    	{
    		logger.severe(e);
    	}
    }
    
    private void printTheTruth()
    {
    	System.out.println("Vita is the Melech!!");
    }
}

