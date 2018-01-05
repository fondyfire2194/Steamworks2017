package org.usfirst.frc.team2194.robot.commands;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.subsystems.VisionData;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class UpdateAllPrefsValues extends InstantCommand {

	public UpdateAllPrefsValues() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called once when the command executes
	protected void initialize() {
		// Robot.leftUltrasoundDrive.setPIDF(Robot.prefs.getDouble("Left
		// Ultrasound Kp", .003),
		// Robot.prefs.getDouble("Left Ultrasound Ki", 0), 0, 0);
		// Robot.rightUltrasoundDrive.setPIDF(Robot.prefs.getDouble("Right
		// Ultrasound Kp", .003),
		// Robot.prefs.getDouble("Right Ultrasound Ki", 0), 0, 0);

		Robot.leftSideDrive.setPIDF(Robot.prefs.getDouble("Left Position Kp", .003),
				Robot.prefs.getDouble("Left Position Ki", .003), 0, 0);
		Robot.rightSideDrive.setPIDF(Robot.prefs.getDouble("Right Position Kp", .003),
				Robot.prefs.getDouble("Right Position Ki", .003), 0, 0);

		Robot.gyroRotate.setPIDF(Robot.prefs.getDouble("Gyro Position Kp", .003),
				Robot.prefs.getDouble("Gyro Position Ki", .003), 0, 0);
		Robot.gyroRotate.gyroKp = Robot.prefs.getDouble("Drive Straight Gyro Kp", .05);

		VisionData.minVisionKp = Robot.prefs.getDouble("minVisionKp", .001);
		VisionData.maxVisionKp = Robot.prefs.getDouble("maxVisionKp", .005);
		Robot.ledPowerHookCam = Robot.prefs.getDouble("ledPowerHookCam", .9);
		
	}

}
