package org.usfirst.frc.team2194.robot.commands;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class HookVisionOnOff extends Command {
	boolean myOn;

	public HookVisionOnOff(boolean on) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		myOn = on;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

		Robot.hookVisionTurnedOn = myOn;
		if (Robot.hookVisionTurnedOn) {
			RobotMap.greenLedsOn.set(Value.kOn);
			RobotMap.cameraLEDs.set(Robot.ledPowerHookCam);
			Robot.gearVisionTurnedOn = false;
			Robot.boilerVisionTurnedOn = false;

		} else { 
			RobotMap.greenLedsOn.set(Value.kOff);
			RobotMap.cameraLEDs.set(0);

		}

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
