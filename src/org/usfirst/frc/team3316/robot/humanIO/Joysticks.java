/**
 * Class for joysticks and joystick buttons
 */
package org.usfirst.frc.team3316.robot.humanIO;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.anschluss.commands.CloseAnschluss;
import org.usfirst.frc.team3316.robot.anschluss.commands.OpenAnschluss;
import org.usfirst.frc.team3316.robot.config.Config;
import org.usfirst.frc.team3316.robot.config.Config.ConfigException;
import org.usfirst.frc.team3316.robot.logger.DBugLogger;
import org.usfirst.frc.team3316.robot.rollerGripper.commands.RollIn;
import org.usfirst.frc.team3316.robot.rollerGripper.commands.RollOut;
import org.usfirst.frc.team3316.robot.rollerGripper.commands.RollTurnClockwise;
import org.usfirst.frc.team3316.robot.rollerGripper.commands.RollTurnCounterClockwise;
import org.usfirst.frc.team3316.robot.stacker.commands.CloseGripper;
import org.usfirst.frc.team3316.robot.stacker.commands.HoldContainer;
import org.usfirst.frc.team3316.robot.stacker.commands.MoveStackerToFloor;
import org.usfirst.frc.team3316.robot.stacker.commands.MoveStackerToStep;
import org.usfirst.frc.team3316.robot.stacker.commands.MoveStackerToTote;
import org.usfirst.frc.team3316.robot.stacker.commands.OpenGripper;
import org.usfirst.frc.team3316.robot.stacker.commands.ReleaseContainer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class Joysticks 
{
	/*
	 * Defines a button in a gamepad POV for an array of angles 
	 */
	private class POVButton extends Button
	{
		Joystick m_joystick;
		int m_deg;
		public POVButton (Joystick joystick, int deg)
		{
			m_joystick = joystick;
			m_deg = deg;
		}
		
		public boolean get()
		{
			if (m_joystick.getPOV() == m_deg)
			{
				return true;
			}
			return false;
		}
	}
	
	
	Config config = Robot.config;
	DBugLogger logger = Robot.logger;
	/*
	 * Joysticks
	 */
	public Joystick joystickLeft;
	public Joystick joystickRight;
	public Joystick joystickOperator;
	
	/*
	 * Joystick Buttons
	 */
	public POVButton buttonRollIn, 
						 buttonRollOut, 
						 buttonRollTurnClockwise, 
						 buttonRollTurnCounterClockwise;
	
	public JoystickButton moveStackerToFloor, 
						  moveStackerToStep, 
						  moveStackerToTote;
	
	public JoystickButton holdContainer,
						  releaseContainer;
	
	public JoystickButton openGripper,
						  closeGripper;
	
	public JoystickButton openAnschluss,
						  closeAnschluss;
	
	public Joysticks ()
	{
		try 
    	{
			/*
			 * Joysticks
			 */
			joystickLeft = new Joystick((int) config.get("JOYSTICK_LEFT"));
			joystickRight = new Joystick((int) config.get("JOYSTICK_RIGHT"));
	    	joystickOperator = new Joystick((int) config.get("JOYSTICK_OPERATOR"));
    	} 
    	catch (ConfigException e) 
    	{
			logger.severe(e);
		}
	}
	
	public void initButtons ()
	{
		/*
    	 * Joystick Buttons
    	 */
		try
		{
	    	/*
	    	 * Roller Gripper
	    	 */
	    	buttonRollIn = new POVButton(joystickOperator, 
	    			(int) config.get("BUTTON_ROLL_IN"));
	    	buttonRollIn.whileHeld(new RollIn());
	    	
	    	buttonRollOut = new POVButton(joystickOperator, 
	    			(int) config.get("BUTTON_ROLL_OUT"));
	    	buttonRollOut.whileHeld(new RollOut());
	    	
	    	buttonRollTurnClockwise = new POVButton(joystickOperator, 
	    			(int) config.get("BUTTON_ROLL_TURN_CLOCKWISE"));
	    	buttonRollTurnClockwise.whileHeld(new RollTurnClockwise());
	    	
	    	buttonRollTurnCounterClockwise = new POVButton(joystickOperator, 
	    			(int) config.get("BUTTON_ROLL_TURN_COUNTER_CLOCKWISE"));
	    	buttonRollTurnCounterClockwise.whileHeld(new RollTurnCounterClockwise());
	    	
	    	/*
	    	 * Stacker
	    	 */
	    	moveStackerToFloor = new JoystickButton(joystickOperator, 
 					(int) config.get("BUTTON_MOVE_STACKER_TO_FLOOR"));
			moveStackerToFloor.whenPressed(new MoveStackerToFloor());
			
			moveStackerToStep = new JoystickButton(joystickOperator, 
					(int) config.get("BUTTON_MOVE_STACKER_TO_STEP"));
			moveStackerToStep.whenPressed(new MoveStackerToStep());
			
			moveStackerToTote = new JoystickButton(joystickOperator, 
					(int) config.get("BUTTON_MOVE_STACKER_TO_TOTE"));
			moveStackerToTote.whenPressed(new MoveStackerToTote());
			
			holdContainer = new JoystickButton(joystickOperator,
					(int) config.get("BUTTON_HOLD_CONTAINER"));
			holdContainer.whenPressed(new HoldContainer());
			
			releaseContainer = new JoystickButton(joystickOperator,
					(int) config.get("BUTTON_RELEASE_CONTAINER"));
			releaseContainer.whenPressed(new ReleaseContainer());
			
			openGripper = new JoystickButton(joystickOperator,
					(int) config.get("BUTTON_OPEN_GRIPPER"));
			openGripper.whenPressed(new OpenGripper());
			
			closeGripper = new JoystickButton(joystickOperator,
					(int) config.get("BUTTON_CLOSE_GRIPPER"));
			closeGripper.whenPressed(new CloseGripper());
			
			/*
			 * Anschluss
			 */
			
			openAnschluss = new JoystickButton(joystickOperator,
					(int) config.get("BUTTON_OPEN_ANSCHLUSS"));
			openAnschluss.whileHeld(new OpenAnschluss());
			
			closeAnschluss = new JoystickButton(joystickOperator,
					(int) config.get("BUTTON_CLOSE_ANSCHLUSS"));
			closeAnschluss.whileHeld(new CloseAnschluss());
			
		}
		catch (ConfigException e)
		{
			logger.severe(e);
		}
	}
}
