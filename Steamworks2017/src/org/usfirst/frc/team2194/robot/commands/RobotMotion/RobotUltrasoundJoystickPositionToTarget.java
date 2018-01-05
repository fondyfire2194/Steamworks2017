package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RobotUltrasoundJoystickPositionToTarget extends Command {
	private double mySpeed;
	private boolean myEndItNow;
	private double myTimeout;
	private double rampIncrement;
	private boolean useGyroComp;
	private boolean useVisionComp;
	private boolean doneAccelerating;
	public static double currentMaxSpeed;
	private double remainingDistance = 0.0;
	private double startingTargetAngle;
	private boolean motionStarted;
	private double myEndpoint;

	public enum DistancesMode {
		vision, ultrasound, encoder
	}

	// side distances are in inches
	// side speeds are in per unit where .25 = 25%
	// inPositionband is in inches

	public RobotUltrasoundJoystickPositionToTarget(double distance, double speed, boolean endItNow, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain);
		requires(Robot.leftUltrasoundDrive);
		requires(Robot.rightUltrasoundDrive);
		myEndpoint = distance;
		mySpeed = speed;
		myEndItNow = endItNow;

		myTimeout = timeout;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		rampIncrement = mySpeed / 50;
		setTimeout(myTimeout);
		Robot.leftUltrasoundDrive.setPIDF(Robot.prefs.getDouble("Left Ultrasound Kp", .003), 0, 0, 0);
		Robot.rightUltrasoundDrive.setPIDF(Robot.prefs.getDouble("Right Ultrasound Kp", .003), 0, 0, 0);

		rampIncrement = mySpeed / 50;
		Robot.leftUltrasoundDrive.setSetpoint(myEndpoint);
		Robot.leftUltrasoundDrive.enable();
		Robot.rightUltrasoundDrive.setSetpoint(myEndpoint);
		Robot.rightUltrasoundDrive.enable();

		Robot.isUltrasoundPositioning = true;
		setTimeout(myTimeout);
		Robot.anglePixelIncrement = Robot.prefs.getInt("Corr Pixs", 5);
		startingTargetAngle = Robot.gyroRotate.targetAngle;
		motionStarted = false;
		Robot.angleCorrectionPixels = 0;

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		remainingDistance = Robot.ultraSound.readLeftUltrasoundInches();

		currentMaxSpeed = mySpeed * -Robot.oi.joystick1.getY();
		// wait for good vision results
		if ((!Robot.visionData.getConsistentValidImages())) {
			useGyroComp = true;
			useVisionComp = false;
		}
		// between start vision and latching the angle compensation
		// using vision comp and zero the angle comp
		if (remainingDistance < Robot.startVisionDistance && remainingDistance > Robot.latchAngleCompPixelsDistance
				&& Robot.visionData.isValidNumberImages()) {
			Robot.angleCorrectionPixels = 0;
			useVisionComp = true;
			useGyroComp = false;
		}
		// in vision zone. Angle correction is 0 until robot is locked on target
		// then use gyro angle to get angle compensation so point of gear hook
		// is the target
		if (remainingDistance < Robot.latchAngleCompPixelsDistance && Robot.angleCorrectionPixels == 0
				&& Robot.visionData.isValidNumberImages() && Robot.visionData.inRangeTargetCenter()) {
			Robot.angleCorrectionPixels = (int) Robot.gyroRotate.getTargetError() * Robot.anglePixelIncrement;
			// Robot.gyroRotate.targetAngle = Robot.gyroRotate.getGyroAngle();
			if (Robot.angleCorrectionPixels > 150)
				Robot.angleCorrectionPixels = 150;
			if (Robot.angleCorrectionPixels < -150)
				Robot.angleCorrectionPixels = -150;
		}

		if (Robot.visionData.getConsistentValidImages()) {
			Robot.gyroRotate.targetAngle = Robot.gyroRotate.getGyroAngle() + Robot.visionData.getAngleToTargetCenter();
		}
		// vision becomes unavailable close to the target.
		// Constantly update robot target angle to actual angle so when vision
		// goes away
		// robot can use the gyro as guide
		if (remainingDistance < Robot.latchAngleCompPixelsDistance && remainingDistance > Robot.heightPxChangeover
				&& Robot.visionData.isValidNumberImages()) {
			useVisionComp = true;
			useGyroComp = false;
			// Robot.gyroRotate.targetAngle = Robot.gyroRotate.getGyroAngle();
		}
		// when vision unavailable switch to gyro
		if (remainingDistance < Robot.heightPxChangeover) {
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

		Robot.leftUltrasoundDrive.getComp(currentMaxSpeed, Robot.activeMotionComp);

		Robot.rightUltrasoundDrive.getComp(currentMaxSpeed, Robot.activeMotionComp);
		if (Robot.isUltrasoundPositioning)
			motionStarted = true;
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isTimedOut() || myEndItNow || Robot.ultraSound.readLeftUltrasoundInches() < Robot.releaseGearDistance;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.leftUltrasoundDrive.disable();
		Robot.rightUltrasoundDrive.disable();
		Robot.isUltrasoundPositioning = false;
		motionStarted = false;
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
