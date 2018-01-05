package org.usfirst.frc.team2194.robot.commands;

import java.io.IOException;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.WriteFile;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SaveCameraData extends Command {

	public SaveCameraData() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		setTimeout(30);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		try {
			Robot.writeFile = new WriteFile("u/vision", Robot.visionData.heightPx, Robot.visionData.targetWidthPixels,
					Robot.visionData.distanceInches);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut();
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
