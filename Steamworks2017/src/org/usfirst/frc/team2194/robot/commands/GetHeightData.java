package org.usfirst.frc.team2194.robot.commands;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.subsystems.VisionData;

import edu.wpi.first.wpilibj.command.Command;

/**
 * used to capture vision target height and ultrasound distance data
 */
public class GetHeightData extends Command {
	private int i;

	public GetHeightData() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		i = 0;
		setTimeout(45);
		for (i = 0; i < +VisionData.numberReadings; i++) {
			Robot.visionData.heightPx[i] = 0;
			Robot.visionData.targetWidthPixels[i] = 0;
			Robot.visionData.distanceInches[i] = 0;
		}
		i = Robot.visionData.getAverageHeight() / 5;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.visionData.getAverageHeight() / 5 >= i && Robot.visionData.heightPx[i] == 0) {
			Robot.visionData.heightPx[i] = Robot.visionData.getAverageHeight();
			Robot.visionData.targetWidthPixels[i] = Robot.visionData.getTargetWidthPixels();
			Robot.visionData.distanceInches[i] = Robot.ultraSound.readAverageUltrasoundInches();
			i++;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return i >= VisionData.numberReadings || isTimedOut();
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
