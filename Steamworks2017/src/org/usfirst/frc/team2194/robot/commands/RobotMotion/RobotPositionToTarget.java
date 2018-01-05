package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RobotPositionToTarget extends Command {
	private double mySpeed;
	private boolean myEndItNow;
	private double myTimeout;
	private double rampIncrement;
	private double distanceEncoderCounts;
	private boolean useGyroComp;
	private boolean useVisionComp;
	private boolean doneAccelerating;
	public static double currentMaxSpeed;
	private double remainingDistance = 0.0;
	private double startingTargetAngle;
	private boolean motionStarted;
	private boolean finalEndpointSet;

	public enum DistancesMode {
		vision, ultrasound, encoder
	}

	// side distances are in inches
	// side speeds are in per unit where .25 = 25%
	// inPositionband is in inches

	public RobotPositionToTarget(double distance, double speed, boolean endItNow, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain);

		distanceEncoderCounts = distance * Robot.driveTrain.encoderCountsPerInch;
		mySpeed = speed;
		myEndItNow = endItNow;

		myTimeout = timeout;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		rampIncrement = mySpeed / 50;
		setTimeout(myTimeout);
		// Robot.leftSideDrive.setPIDF(Robot.prefs.getDouble("Left Position Kp",
		// .003), 0, 0, 0);
		// Robot.rightSideDrive.setPIDF(Robot.prefs.getDouble("Right Position
		// Kp", .003), 0, 0, 0);
		Robot.leftSideDrive.setSetpoint(distanceEncoderCounts);
		Robot.rightSideDrive.setSetpoint(distanceEncoderCounts);
		Robot.leftSideDrive.enable();
		Robot.rightSideDrive.enable();
		Robot.isPositioning = true;
		startingTargetAngle = Robot.gyroRotate.targetAngle;
		motionStarted = false;
		finalEndpointSet = false;
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

		// wait for good vision results
		// between start vision and latching the angle compensation
		// using vision comp and zero the angle comp
		if (Robot.visionData.isValidNumberImages()) {
			Robot.angleCorrectionPixels = 0;
			useVisionComp = true;
			useGyroComp = false;
		}
		// in vision zone. Angle correction is 0 until robot is locked on target
		// then use gyro angle to get angle compensation so point of gear hook
		// is the target

		if (Robot.visionData.getConsistentValidImages()) {
			Robot.gyroRotate.targetAngle = Robot.gyroRotate.getGyroAngle() + Robot.visionData.getAngleToTargetCenter();
		}
		// vision becomes unavailable close to the target.
		// Constantly update robot target angle to actual angle so when vision
		// goes away
		// robot can use the gyro as guide
		// when vision unavailable switch to gyro
		if (Robot.visionData.getAverageHeight() < Robot.heightPxChangeover) {
			useVisionComp = false;
			useGyroComp = true;
		}

		// release gear an stop motion
		if (useVisionComp)
			Robot.activeMotionComp = Robot.visionData.getVisionComp();
		if (useGyroComp)
			Robot.activeMotionComp = Robot.gyroRotate.getTargetYawComp();

		// need to determine motion direction so max and min outputs of PID loop
		// can be set

		if (Robot.visionData.getAverageHeight() > Robot.heightPxChangeover && !finalEndpointSet
				&& Robot.visionData.isValidNumberImages()) {
			Robot.leftSideDrive
					.setSetpoint(Robot.leftSideDrive.getPosition() + 22 * Robot.driveTrain.encoderCountsPerInch);
			Robot.rightSideDrive
					.setSetpoint(Robot.rightSideDrive.getPosition() + 22 * Robot.driveTrain.encoderCountsPerInch);
			finalEndpointSet = true;
		}

		Robot.leftSideDrive.getComp(currentMaxSpeed, Robot.activeMotionComp);

		Robot.rightSideDrive.getComp(currentMaxSpeed, Robot.activeMotionComp);

		if (Robot.isPositioning)
			motionStarted = true;
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isTimedOut() || myEndItNow;
	}

	@Override
	protected void end() {
		Robot.isPositioning = false;
		Robot.stopMotion = false;
		doneAccelerating = false;
		motionStarted = false;
		finalEndpointSet = false;
		currentMaxSpeed = 0;
		Robot.gyroRotate.targetAngle = startingTargetAngle;

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
