package org.usfirst.frc.team3316.robot.chassis.commands;

import java.util.function.DoubleSupplier;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.config.Config.ConfigException;
import org.usfirst.frc.team3316.robot.subsystems.Chassis.NavigationIntegrator;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RobotOrientedNavigation extends FieldOrientedDrive 
{
	/*
	 * PIDSource classes in order to use wpilib's PIDController
	 */
	private class PIDSourceX implements PIDSource
	{
		public double pidGet() 
		{
			logger.finest("" + integrator.getX());
			return integrator.getX();
		}
	}
	private class PIDSourceY implements PIDSource
	{
		public double pidGet() 
		{
			logger.finest("" + integrator.getY());
			return integrator.getY();
		}
	}
	private class PIDSourceHeading implements PIDSource
	{
		public double pidGet() 
		{
			logger.finest("" + integrator.getHeading());
			return integrator.getHeading();
		}
	}
	
	/*
	 * PIDOutput classes in order to use wpilib's PIDController
	 */
	private class PIDOutputX implements PIDOutput
	{
		public void pidWrite (double output) 
		{
			logger.finest("" + output);
			outputX = output;
		}
	}
	private class PIDOutputY implements PIDOutput
	{
		public void pidWrite (double output) 
		{
			logger.finest("" + output);
			outputY = output;
		}
	}
	private class PIDOutputHeading implements PIDOutput
	{
		public void pidWrite (double output) 
		{
			logger.finest("" + output);
			outputHeading = output;
		}
	}
	
	protected NavigationIntegrator integrator;
	
	protected PIDController pidControllerX, pidControllerY, pidControllerHeading;
	
	protected double setpointX, setpointY, setpointHeading;
	
	private double outputX, outputY, outputHeading;
	
	private int set; //the set of pid values to use
	
	private int finishCounter = 0, maxFinishCounter;
	
	public RobotOrientedNavigation (double setpointX , double setpointY, double setpointHeading, int set)
	{
		/*
		 * variable init
		 */
		this.setpointX = setpointX;
		this.setpointY = setpointY;
		this.setpointHeading = setpointHeading;
		this.set = set;
		
		/*
		 * Init of pid controllers
		 */
		// PID variables are updated in updatePIDVariables
		pidControllerX = new PIDController(0, //Kp
										   0, //Ki
										   0, //Kd
										   new PIDSourceX(), //PIDSource 
										   new PIDOutputX(), //PIDOutput
										   0.05); //Update rate in ms
		pidControllerX.setOutputRange(-1, 1);
		
		pidControllerY = new PIDController(0, //Kp
										   0, //Ki
										   0, //Kd
										   new PIDSourceY(), //PIDSource 
										   new PIDOutputY(), //PIDOutput
										   0.05); //Update rate in ms
		pidControllerY.setOutputRange(-1, 1);
		
		pidControllerHeading = new PIDController(0, //Kp
												 0, //Ki
												 0, //Kd
												 new PIDSourceHeading(), //PIDSource 
												 new PIDOutputHeading(), //PIDOutput
												 0.05); //Update rate in ms
		pidControllerHeading.setOutputRange(-1, 1);
		pidControllerHeading.setInputRange(-180, 180);
		pidControllerHeading.setContinuous(true);
		
		pidControllerX.setSetpoint(setpointX);
		pidControllerY.setSetpoint(setpointY);
		pidControllerHeading.setSetpoint(setpointHeading);
		
		rotationPID.setSetpointSource( () -> {return outputHeading;} );
	}
	
	protected void initialize ()
	{
		logger.fine(this.getName() + "initialize");
		
		super.initialize();
		
		integrator = new NavigationIntegrator();
		Robot.chassis.addNavigationIntegrator(integrator);
		
		SmartDashboard.putNumber("Setpoint X", setpointX);
		SmartDashboard.putNumber("Setpoint Y", setpointY);
		SmartDashboard.putNumber("Setpoint Heading", setpointHeading);
		
		finishCounter = 0;
		
		pidControllerX.enable();
		pidControllerY.enable();
		pidControllerHeading.enable();
	}
	
	protected void set ()
	{
		updatePIDVariables();
		logger.finest("output(" + outputX + ", " + outputY + ", " + outputHeading + ")");
		setFieldVector(outputX, outputY, integrator.getHeading());
		setRotation(outputHeading);
	}
	
	protected boolean isFinished ()
	{
		SmartDashboard.putBoolean("PID X on target", pidControllerX.onTarget());
		SmartDashboard.putBoolean("PID Y on target", pidControllerY.onTarget());
		SmartDashboard.putBoolean("PID Heading on target", pidControllerHeading.onTarget());
		
		if (pidControllerX.onTarget() && pidControllerY.onTarget() && pidControllerHeading.onTarget())
		{
			finishCounter++;
		}
		else
		{
			finishCounter = 0;
		}
		
		return finishCounter >= maxFinishCounter;
	}
	
	protected void end ()
	{
		logger.fine(this.getName() + " end");
		super.end();
		
		Robot.chassis.removeNavigationIntegrator(integrator);
		
		pidControllerX.reset();
		pidControllerY.reset();
		pidControllerHeading.reset();
	}
	
	protected void interrupted ()
	{
		logger.fine(this.getName() + " interrupted");
		end();
	}
	
	private void updatePIDVariables ()
	{
		try
		{
			/*
			 * Kp, Ki, Kd values
			 */
			pidControllerX.setPID(
					(double)config.get("chassis_RobotOrientedNavigation_PIDControllerX_KP_" + set) / 1000, 
					(double)config.get("chassis_RobotOrientedNavigation_PIDControllerX_KI_" + set) / 1000, 
					(double)config.get("chassis_RobotOrientedNavigation_PIDControllerX_KD_" + set) / 1000);
			
			pidControllerY.setPID(
					(double)config.get("chassis_RobotOrientedNavigation_PIDControllerY_KP_" + set) / 1000, 
					(double)config.get("chassis_RobotOrientedNavigation_PIDControllerY_KI_" + set) / 1000, 
					(double)config.get("chassis_RobotOrientedNavigation_PIDControllerY_KD_" + set) / 1000);
			
			pidControllerHeading.setPID(
					(double)config.get("chassis_RobotOrientedNavigation_PIDControllerHeading_KP_" + set) / 1000, 
					(double)config.get("chassis_RobotOrientedNavigation_PIDControllerHeading_KI_" + set) / 1000, 
					(double)config.get("chassis_RobotOrientedNavigation_PIDControllerHeading_KD_" + set) / 1000);
			
			/*
			 * Absolute tolerances
			 */
			pidControllerX.setAbsoluteTolerance(
					(double)config.get("chassis_RobotOrientedNavigation_PIDControllerX_AbsoluteTolerance_" + set));
			
			pidControllerY.setAbsoluteTolerance(
					(double)config.get("chassis_RobotOrientedNavigation_PIDControllerY_AbsoluteTolerance_" + set));
			
			pidControllerHeading.setAbsoluteTolerance(
					(double)config.get("chassis_RobotOrientedNavigation_PIDControllerHeading_AbsoluteTolerance_" + set));
			
			/*
			 * Output ranges
			 */
			pidControllerX.setOutputRange(
					(double)config.get("chassis_RobotOrientedNavigation_PIDControllerX_MinimumOutput_" + set), 
					(double)config.get("chassis_RobotOrientedNavigation_PIDControllerX_MaximumOutput_" + set));
			
			pidControllerY.setOutputRange(
					(double)config.get("chassis_RobotOrientedNavigation_PIDControllerY_MinimumOutput_" + set), 
					(double)config.get("chassis_RobotOrientedNavigation_PIDControllerY_MaximumOutput_" + set));
			
			pidControllerHeading.setOutputRange(
					(double)config.get("chassis_RobotOrientedNavigation_PIDControllerHeading_MinimumOutput_" + set), 
					(double)config.get("chassis_RobotOrientedNavigation_PIDControllerHeading_MaximumOutput_" + set));
			
			maxFinishCounter = (int) config.get("chassis_RobotOrientedNavigation_MaxFinishCounter_" + set);
		}
		catch (ConfigException e)
		{
			logger.severe(e);
		}
	}
}
