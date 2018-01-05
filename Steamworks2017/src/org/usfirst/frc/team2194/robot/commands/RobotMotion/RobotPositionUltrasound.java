package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RobotPositionUltrasound extends Command {
	private double mySpeed;
	private double myInPositionBand;
	private boolean myEndItNow;
	private double myTimeout;
	private double rampIncrement;
	private double myEndpoint;
	private boolean useGyroComp;
	private double comp;
	private boolean doneAccelerating;
	public static double currentMaxSpeed;

	public enum CompMode {
		none, gyro
	}

	public enum MoveMode {
		relative, absolute
	}

	// side distances are in inches
	// side speeds are in per unit where .25 = 25%
	// inPositionband is in inches

	public RobotPositionUltrasound(double distance, double speed,
			org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotPosition.MoveMode absolute,
			org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotPosition.CompMode none, boolean endItNow,
			double inPositionBand, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain);
		requires(Robot.leftUltrasoundDrive);
		requires(Robot.rightUltrasoundDrive);

		myEndpoint = distance;
		mySpeed = speed;
		myEndItNow = endItNow;
		myInPositionBand = inPositionBand;
		myTimeout = timeout;
		switch (none) {
		case none:
			useGyroComp = false;
			break;
		case gyro:
			useGyroComp = true;
			break;
		}

		switch (absolute) {
		case relative:
			myEndpoint -= Robot.ultraSound.readLeftUltrasoundInches();
			break;

		case absolute:

			break;
		}
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		// Robot.leftUltrasoundDrive.setPIDF(Robot.prefs.getDouble("Left
		// Ultrasound Kp", .003), 0, 0, 0);
		// Robot.rightUltrasoundDrive.setPIDF(Robot.prefs.getDouble("Right
		// Ultrasound Kp", .003), 0, 0, 0);

		rampIncrement = mySpeed / 50;
		Robot.leftUltrasoundDrive.setSetpoint(myEndpoint);
		Robot.leftUltrasoundDrive.enable();
		Robot.rightUltrasoundDrive.setSetpoint(myEndpoint);
		Robot.rightUltrasoundDrive.enable();

		Robot.isUltrasoundPositioning = true;
		setTimeout(myTimeout);

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (!doneAccelerating) {
			currentMaxSpeed = currentMaxSpeed + rampIncrement;
			if (currentMaxSpeed > mySpeed) {
				currentMaxSpeed = mySpeed;
				doneAccelerating = true;
			}
		}
		if (useGyroComp)
			comp = Robot.gyroRotate.getTargetYawComp();
		else
			comp = 0;

		Robot.leftUltrasoundDrive.getComp(currentMaxSpeed, comp);

		Robot.rightUltrasoundDrive.getComp(currentMaxSpeed, comp);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isTimedOut() || myEndItNow || (Robot.leftUltrasoundDrive.inPosition(myInPositionBand)
				&& Robot.rightUltrasoundDrive.inPosition(myInPositionBand));
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.leftUltrasoundDrive.disable();
		Robot.rightUltrasoundDrive.disable();
		Robot.isUltrasoundPositioning = false;

		doneAccelerating = false;
		currentMaxSpeed = 0;

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
