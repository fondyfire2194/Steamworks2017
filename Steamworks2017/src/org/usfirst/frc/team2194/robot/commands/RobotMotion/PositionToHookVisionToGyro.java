package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Use vision to determin corrrection angle needed and feed it to gyro as robot
 * approaches hook to hang gear. Motion starts out as regular poritioning move
 * then switches over to utrasound positioning to complete
 */
public class PositionToHookVisionToGyro extends Command {
	private double mySpeed;
	private double myEndpoint;
	private boolean myEndItNow;
	private double myTimeout;
	private double myInPositionBand;

	public double myDistance;
	public double startPosition;
	private boolean changedToUltrasoundPositioning;
	private boolean inUltrasoundRange;
	private boolean doneAccelerating;
	public static double currentMaxSpeed;
	private double rampIncrement;
	private double distanceEncoderCounts;

	private double startOfVisionPoint = 50;
	private double endOfVisionPoint = 10;
	private double ultraSoundTargetPoint = 12;
	private boolean inUltraSoundRange;
	private double lookForUlrasound = 15;

	// side distances are in inches
	// side speeds are in per unit where .25 = 25%
	// inPositionband is in inches
	private boolean inVisionRange;

	public PositionToHookVisionToGyro(double distance, double speed, boolean endItNow, double inPositionBand,
			double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain);
		requires(Robot.leftSideDrive);
		requires(Robot.rightSideDrive);

		myEndpoint = distance;
		mySpeed = speed;
		myEndItNow = endItNow;
		myDistance = distance;
		myTimeout = timeout;
		myInPositionBand = inPositionBand;
		distanceEncoderCounts = distance * Robot.driveTrain.encoderCountsPerInch;

	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

		setTimeout(myTimeout);

		rampIncrement = mySpeed / 50;

		Robot.leftSideDrive.setSetpoint(distanceEncoderCounts);
		Robot.rightSideDrive.setSetpoint(distanceEncoderCounts);

		Robot.leftSideDrive.enable();
		Robot.rightSideDrive.enable();
		doneAccelerating = false;
		inVisionRange = false;
		currentMaxSpeed = 0;
		inUltraSoundRange = false;
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

		// if inside vision range
		inVisionRange = (myEndpoint - Robot.driveTrain.getLeftPosition()) < startOfVisionPoint
				&& (Robot.ultraSound.readAverageUltrasoundInches() > endOfVisionPoint);

		// use encoders until in ultrasound range
		Robot.remainingDistanceToHook = myEndpoint - Robot.driveTrain.getLeftPosition();

		if (inVisionRange && Robot.visionData.getConsistentValidImages())
			Robot.gyroRotate.targetAngle = Robot.gyroRotate.getGyroAngle()
					+ Robot.visionData.getTargetCorrectionAngle();

		Robot.leftSideDrive.getComp(currentMaxSpeed, Robot.gyroRotate.getTargetYawComp());
		Robot.rightSideDrive.getComp(currentMaxSpeed, Robot.gyroRotate.getTargetYawComp());

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isTimedOut() || myEndItNow
				|| inUltraSoundRange && ((Robot.ultraSound.readLeftUltrasoundInches() < ultraSoundTargetPoint
						|| (Robot.ultraSound.readLeftUltrasoundInches() < ultraSoundTargetPoint)));
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.leftSideDrive.disable();
		Robot.rightSideDrive.disable();
		currentMaxSpeed = 0;
		inVisionRange = false;
		doneAccelerating = false;
		inUltraSoundRange = false;

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
