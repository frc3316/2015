/**
 * Le robot sensors
 */
package org.usfirst.frc.team3316.robot.robotIO;

import com.kauailabs.nav6.frc.IMUAdvanced;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.config.Config;
import org.usfirst.frc.team3316.robot.config.Config.ConfigException;
import org.usfirst.frc.team3316.robot.logger.DBugLogger;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;

public class Sensors 
{
	Config config = Robot.config;
	DBugLogger logger = Robot.logger;
	
	/*
	 * Chassis
	 */
	public Encoder chassisEncoderLeft, chassisEncoderRight, chassisEncoderCenter;
	public IMUAdvanced navx;
	SerialPort serial_port;
	
	/*
	 * Stacker
	 */
	
	public AnalogInput stackerHeightIR;
	public DigitalInput stackerSwitchTote;
	public DigitalInput stackerSwitchContainer;
	
	/*
	 * Anschluss
	 */
	public DigitalInput anschlussDigitalInputClosed;
	public DigitalInput anschlussDigitalInputOpened;
	/*
	 * Roller-Gripper
	 */
	public AnalogInput rollerGripperIRLeft, rollerGripperIRRight;
	
	public Sensors ()
	{
		try 
		{
			/*
			 * Chassis
			 */
			chassisEncoderLeft = new Encoder((int)config.get("chassisEncoderLeftA"), 
											 (int)config.get("chassisEncoderLeftB"), 
											 false, 
											 CounterBase.EncodingType.k4X);
			
			chassisEncoderRight = new Encoder((int)config.get("chassisEncoderRightA"), 
											 (int)config.get("chassisEncoderRightB"), 
											 false, 
											 CounterBase.EncodingType.k4X);
			
			chassisEncoderCenter = new Encoder((int)config.get("chassisEncoderCenterA"), 
					 						  (int)config.get("chassisEncoderCenterB"), 
					 						  false, 
					 						  CounterBase.EncodingType.k4X);

			serial_port = new SerialPort(57600,SerialPort.Port.kMXP);
			navx = new IMUAdvanced(serial_port);
			
			/*
			 * Stacker
			 */
			
			stackerHeightIR = new AnalogInput((int) config.get("STACKER_IR"));
			
			stackerSwitchTote = new DigitalInput((int) config.get("STACKER_SWITCH_TOTE"));
			stackerSwitchContainer = new DigitalInput((int) config.get("STACKER_SWITCH_CONTAINER"));
			
			/*
			 * Anschluss
			 */
			anschlussDigitalInputClosed = new DigitalInput((int) config.get("anschlussDigitalInputClosed")); //check the channel and update it accordingly
			anschlussDigitalInputOpened = new DigitalInput((int) config.get("anschlussDigitalInputOpened")); //check the channel and update it accordingly

			/*
			 * Roller-Gripper
			 */
			rollerGripperIRLeft = new AnalogInput((int) config.get("ROLLER_GRIPPER_IR_LEFT"));
			rollerGripperIRRight = new AnalogInput((int) config.get("ROLLER_GRIPPER_IR_RIGHT"));
		}
		
		catch (ConfigException e)
		{
			logger.severe(e);
		}
	}
}
