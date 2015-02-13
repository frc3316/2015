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
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class RollerGripper extends Subsystem 
{	
	Config config = Robot.config;
	DBugLogger logger = Robot.logger;
	
	//TODO: fix all of the private names to not start with gripper
	private VictorSP gripperLeft, gripperRight;
	
	private AnalogInput gripperGPIR;
	
	private DigitalInput gripperSwitchGP;
	
	private double leftScale, rightScale;
	
	// Variables for getGamePieceCollected()
	// They are set in updateDistanceVariables()
	private double toteDistanceMin, 
				   toteDistanceMax,
				   containerDistanceMin, 
				   containerDistanceMax,
				   gpDistanceThreshold;
	
	private Roll defaultRoll;
	
    public RollerGripper () 
    {
    	gripperLeft = Robot.actuators.rollerGripperMotorControllerLeft;
    	gripperRight = Robot.actuators.rollerGripperMotorControllerRight;
    	
    	gripperGPIR = Robot.sensors.rollerGripperGPIR;
    	
    	gripperSwitchGP = Robot.sensors.rollerGripperSwitchGP;
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
    
    //TODO: fix name to be getIRGPDistance
    public double getGPIRDistance ()
    {
    	return (1 / gripperGPIR.getVoltage());
    }
    
    public boolean getSwitchGamePiece ()
    {
    	return !gripperSwitchGP.get();
    }
    
    /**
     * Returns which gamepiece there is in the roller gripper (if any)
     * @return Tote for tote, Container for container, Unsure if there 
     * is something in but can't figure out which and none if empty 
     */
    public GamePieceCollected getGamePieceCollected ()
    {
    	updateDistanceVariables();
    	
    	double gpDistance = getGPIRDistance();
    	boolean gpSwitch = getSwitchGamePiece();
    	
    	if (gpSwitch)
    	{
    		if (gpDistance > toteDistanceMin && gpDistance < toteDistanceMax)
    		{
    			return GamePieceCollected.Tote;
    		}
    		else if (gpDistance > containerDistanceMin && gpDistance < containerDistanceMax)
    		{
    			return GamePieceCollected.Container;
    		}
    		else
    		{
    			return GamePieceCollected.Unsure;
    		}
    	}
    	else
    	{
    		if (gpDistance < gpDistanceThreshold)
    		{
    			return GamePieceCollected.Unsure;
    		}
    		else
    		{
    			return GamePieceCollected.None;
    		}
    	}
    }
    
    private void updateDistanceVariables () 
    {
    	try 
    	{
			toteDistanceMin = (double) config.get("rollerGripper_ToteDistanceMinimum");
			toteDistanceMax = (double) config.get("rollerGripper_ToteDistanceMaximum");
			
			containerDistanceMin = (double) config.get("rollerGripper_ContainerDistanceMinimum");
			containerDistanceMax = (double) config.get("rollerGripper_ContainerDistanceMaximum");
			
			gpDistanceThreshold = (double) config.get("rollerGripper_GamePieceDistanceThreshold");
		} 
    	catch (ConfigException e) 
    	{
			logger.severe(e);
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

