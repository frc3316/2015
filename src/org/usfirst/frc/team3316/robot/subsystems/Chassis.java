/**
 * Chassis subsystem
 */
package org.usfirst.frc.team3316.robot.subsystems;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.chassis.commands.Drive;
import org.usfirst.frc.team3316.robot.config.Config;
import org.usfirst.frc.team3316.robot.config.Config.ConfigException;
import org.usfirst.frc.team3316.robot.logger.DBugLogger;

import com.kauailabs.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Chassis extends Subsystem 
{
	Config config = Robot.config;
	DBugLogger logger = Robot.logger;
	
	private VictorSP left;
	private VictorSP right;
	private VictorSP center;
	
	private IMUAdvanced navx;
	
	private double leftScale, rightScale, centerScale;
	
	Drive defaultDrive;
	
	public Chassis ()
	{
		left = Robot.actuators.chassisLeft;
		right = Robot.actuators.chassisRight;
		center = Robot.actuators.chassisCenter;
		
		navx = Robot.sensors.navx;
		//need to init defaultDrive before setting it as the default drive
		//defaultDrive = new StrafeDrive ();
	}
	
    public void initDefaultCommand() 
    {
       //setDefaultCommand(defaultDrive);
    }
    
    public void configUpdate ()
    {
		//CR: If you want drive to call the "configUpdate" method only
		//    when prompted by the controller, while the rest of the code
		//    calls it automatically - make sure you explain why...
    	try
    	{
    		leftScale = (double)config.get("chassisLeftScale");
    		rightScale = (double)config.get("chassisRightScale");
    		centerScale = (double)config.get("chassisCenterScale");
    	}
    	catch (ConfigException e)
    	{
			//CR: are you passing the message or the throwable?
    		logger.severe(e.getMessage());
    	}
    }
    
    public boolean set (double left, double right, double center)
    {
    	this.left.set(left*leftScale);
    	this.right.set(right*rightScale);
    	this.center.set(center*centerScale);
    	return true;
    }
    //CR: What about accelerometer readings?
    public double getHeading ()
    {
    	return navx.getYaw();
    }
    
    public double getAngularVelocity ()
    {
    	//should return gyro reading
    	return 0;
    }
}

