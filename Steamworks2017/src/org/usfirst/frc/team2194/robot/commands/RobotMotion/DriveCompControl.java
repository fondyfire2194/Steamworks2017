package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.commands.ReleaseGear;
import org.usfirst.frc.team2194.robot.commands.StopMotion;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Run this as a command during vision gear to hook approach It does the
 * following Keeps angle correction pixels at zero when outside 40" from target
 * Latches the gyro angle at that point and fills in the angle compensation
 * pixels value Continuously sets the gyro target angle to the gyro angle, which
 * will be changing as vision comp ajusts the robot to point t the tip of the
 * gear hook This will allow switching to gyro control as the robot gets too
 * close for vision comp to be available because of hook or other interference
 * with targets It starts the gear release command at the set distance an ends
 * any motion that drove the robot to the target
 */
public class DriveCompControl extends Command {
	private boolean myUseVisionDistances;

	public DriveCompControl(boolean useVisionDistances) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		myUseVisionDistances = useVisionDistances;
	}

	private double remainingDistance = 0.0;
	private double startingTargetAngle;
	private double startingAngle;

	private boolean useGyroComp;
	private boolean useVisionComp;
	private double ultrasoundAngle;

	private boolean targetAngleSet = false;

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.anglePixelIncrement = Robot.prefs.getInt("Corr Pixs", 7);
		startingTargetAngle = Robot.gyroRotate.targetAngle;
		startingAngle = Robot.gyroRotate.getGyroAngle();
		Robot.angleCorrectionPixels = 0;

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (myUseVisionDistances) {
			remainingDistance = Robot.visionData.getDistanceFromHeightArray();
		} else {
			remainingDistance = Robot.ultraSound.readLeftUltrasoundInches();
		}

		// outside vision zone - use gyro

		ultrasoundAngle = Math
				.atan((Robot.ultraSound.readLeftUltrasoundInches() - Robot.ultraSound.readLeftUltrasoundInches()) / 21);

		if ((!myUseVisionDistances && remainingDistance > Robot.startVisionDistance)
				|| (myUseVisionDistances && !Robot.visionData.getConsistentValidImages())) {
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
		//
		if (remainingDistance < Robot.latchAngleCompPixelsDistance && Robot.angleCorrectionPixels == 0
				&& Robot.visionData.getConsistentValidImages() && Robot.visionData.inRangeTargetCenter()) {
			Robot.angleCorrectionPixels = (int) (Robot.gyroRotate.getGyroAngle() - startingAngle)
					* Robot.anglePixelIncrement;
			// Robot.angleCorrectionPixels = (int)
			// (Robot.visionData.getAngleToTargetCenter() *(double)
			// Robot.anglePixelIncrement);
			// Robot.angleCorrectionPixels =
			// Robot.visionData.getAnglePixelComp();
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
		// release gear and stop motion
		if (Robot.ultraSound.readLeftUltrasoundInches() < Robot.releaseGearDistance) {
			new ReleaseGear();
			new StopMotion();
			// Robot.visionCompJoystick = false;
		}

		if (Robot.oi.joystick1.getY() > 0) {
			if (targetAngleSet == false) {
				Robot.gyroRotate.targetAngle = Robot.gyroRotate.getGyroAngle();
			}
			targetAngleSet = true;
			useVisionComp = false;
			useGyroComp = true;
		} else {
			targetAngleSet = false;
		}

		if (useVisionComp)
			Robot.activeMotionComp = Robot.visionData.getVisionComp();
		if (useGyroComp)
			Robot.activeMotionComp = Robot.gyroRotate.getTargetYawComp();

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return (!Robot.visionCompJoystick);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		useVisionComp = false;
		useGyroComp = false;
		Robot.gyroRotate.targetAngle = startingTargetAngle;
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
