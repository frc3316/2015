package org.usfirst.frc.team3316.robot.chassis.commands;

import java.util.function.DoubleSupplier;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.config.Config;
import org.usfirst.frc.team3316.robot.config.Config.ConfigException;
import org.usfirst.frc.team3316.robot.logger.DBugLogger;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public class RotationPID
{
	protected Config config = Robot.config;
	protected DBugLogger logger = Robot.logger;
	
	private DoubleSupplier setpointSource; 
	
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
	
	public RotationPID ()
	{
		this ( () -> {return TankDrive.getLeftX();} );
	}
	
	public RotationPID (DoubleSupplier setpointSource)
	{
		this.setpointSource = setpointSource;
		pidControllerRotation = new PIDController(0, 
												  0, 
												  0, 
												  new PIDSourceRotation(), 
												  new PIDOutputRotation(), 
												  0.05);
		pidControllerRotation.setOutputRange(-1, 1);
	}
	
	public void setSetpointSource (DoubleSupplier setpointSource)
	{
		this.setpointSource = setpointSource;
	}
	
	public double get ()
	{
		updatePIDVariables();
		
		setPIDControllerRotationSetpoint();
		
		return outputRotation;
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
	
	private void setPIDControllerRotationSetpoint ()
	{
		try
		{
			setpointScale = (double) config.get("chassis_RobotOrientedDrivePIDRotation_SetpointScale");
			setpointRotation = setpointSource.getAsDouble() * setpointScale;
			pidControllerRotation.setSetpoint(setpointRotation);
		}
		catch (ConfigException e)
		{
			logger.severe(e);
		}
	}
	
	public void enable(){
		pidControllerRotation.enable();
	}
	
	public void disable(){
		pidControllerRotation.disable();
	}
}
