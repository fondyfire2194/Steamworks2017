package org.usfirst.frc.team2194.robot.commands;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.subsystems.VisionData;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IncrementPxHeightIndex extends Command {

	public IncrementPxHeightIndex() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.visionData.iPx += 1;
		if (Robot.visionData.iPx > VisionData.numberReadings - 1)
			Robot.visionData.iPx = 0;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
