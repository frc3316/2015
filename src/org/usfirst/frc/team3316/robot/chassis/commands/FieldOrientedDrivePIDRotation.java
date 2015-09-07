package org.usfirst.frc.team3316.robot.chassis.commands;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.config.Config.ConfigException;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldOrientedDrivePIDRotation extends FieldOrientedDrive
{
	private class PIDSourceRotation implements PIDSource
	{
		public double pidGet() 
		{
			return Robot.chassis.getAngularVelocity();
		}
	}
	
	private class PIDOutputRotation implements PIDOutput
	{
		public void pidWrite(double output)
		{
			outputRotation = output;
		}
	}
	
	private PIDController pidControllerRotation;
	
	private double setpointScale;
	private double setpointRotation;
	
	private double outputRotation;

	public FieldOrientedDrivePIDRotation ()
	{
		pidControllerRotation = new PIDController(0, 
												  0, 
												  0, 
												  new PIDSourceRotation(), 
												  new PIDOutputRotation(), 
												  0.05);
		pidControllerRotation.setOutputRange(-1, 1);
	}
	
	protected void initialize ()
	{
		super.initialize();
		pidControllerRotation.enable();
	}
	protected void set ()
	{
		setFieldVector(getRightX(), getRightY(), Robot.chassis.getHeading());
		
		updatePIDVariables();
		setPIDControllerRotationSetpoint();
		SmartDashboard.putNumber("setpointRotation" , setpointRotation);
		SmartDashboard.putNumber("outputRotation" , outputRotation);
		setRotation(outputRotation);
	}
	
	private void updatePIDVariables ()
	{
		try
		{	
			pidControllerRotation.setPID(
					(double) config.get("chassis_RobotOrientedDrivePIDRotation_PIDControllerRotation_KP") / 1000, 
					(double) config.get("chassis_RobotOrientedDrivePIDRotation_PIDControllerRotation_KI") / 1000, 
					(double) config.get("chassis_RobotOrientedDrivePIDRotation_PIDControllerRotation_KD") / 1000);
			
			pidControllerRotation.setAbsoluteTolerance(
					(double) config.get("chassis_RobotOrientedDrivePIDRotation_PIDControllerRotation_AbsoluteTolerance"));
			
			pidControllerRotation.setOutputRange(
					(double) config.get("chassis_RobotOrientedDrivePIDRotation_PIDControllerRotation_MinimumOutput"), 
					(double) config.get("chassis_RobotOrientedDrivePIDRotation_PIDControllerRotation_MaximumOutput"));
		}
		catch (ConfigException e)
		{
			logger.severe(e);
		}
	}
	
	protected void setPIDControllerRotationSetpoint ()
	{
		try
		{
			setpointScale = (double) config.get("chassis_RobotOrientedDrivePIDRotation_SetpointScale");
			setpointRotation = getLeftX() * setpointScale;
			pidControllerRotation.setSetpoint(setpointRotation);
		}
		catch (ConfigException e)
		{
			logger.severe(e);
		}
	}
    
	protected void end() 
    {
    	super.end();
    	pidControllerRotation.disable();
    }
}
