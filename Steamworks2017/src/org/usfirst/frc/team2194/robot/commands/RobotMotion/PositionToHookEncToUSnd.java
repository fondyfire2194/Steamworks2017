package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Use as a parallel command to contol vision / gyro comp as robot approaches
 * hook to hang gear Motion starts out as regular poritioning move then switches
 * over to utrasound positioning to complete
 */
public class PositionToHookEncToUSnd extends Command {
	private double mySpeed;
	private double myEndpoint;
	private boolean myEndItNow;
	private double myTimeout;
	private double myInPositionBand;
	private boolean useGyroComp;
	private boolean useVisionComp;

	public double myDistance;
	public double startPosition;
	private boolean doneAccelerating;
	public static double currentMaxSpeed;
	private double rampIncrement;
	private double distanceEncoderCounts;

	private double startOfVisionPoint = 60;
	private double endOfVisionPoint = 12;
	private boolean inVisionRange;

	public PositionToHookEncToUSnd(double distance, double speed, boolean endItNow, double inPositionBand,
			double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.leftSideDrive);
		requires(Robot.rightSideDrive);

		myEndpoint = distance;
		mySpeed = speed;
		myEndItNow = endItNow;
		myDistance = distance;
		myTimeout = timeout;
		myInPositionBand = inPositionBand * Robot.driveTrain.encoderCountsPerInch;
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
		currentMaxSpeed = 0;
		inVisionRange = false;
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

		Robot.remainingDistanceToHook = myEndpoint - Robot.driveTrain.getRightPosition();

		inVisionRange = Robot.remainingDistanceToHook < startOfVisionPoint
				&& Robot.remainingDistanceToHook > endOfVisionPoint;

		useVisionComp = inVisionRange && Robot.visionData.isValidNumberImages();
		useGyroComp = !useVisionComp;

		if (useVisionComp) {
			Robot.gyroRotate.targetAngle = Robot.gyroRotate.getGyroAngle();
			Robot.activeMotionComp = Robot.visionData.getVisionComp();
		}
		if (useGyroComp)
			Robot.activeMotionComp = Robot.gyroRotate.getTargetYawComp();

		SmartDashboard.putBoolean("VComp", useVisionComp);
		SmartDashboard.putBoolean("GComp", useGyroComp);
		SmartDashboard.putBoolean("InVRange", inVisionRange);

		SmartDashboard.putNumber("RemDist", Robot.remainingDistanceToHook);

		Robot.leftSideDrive.getComp(currentMaxSpeed, Robot.activeMotionComp);
		Robot.rightSideDrive.getComp(currentMaxSpeed, Robot.activeMotionComp);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isTimedOut() || myEndItNow || Robot.remainingDistanceToHook < 1
				|| Robot.leftSideDrive.inPosition(myInPositionBand)
				|| Robot.rightSideDrive.inPosition(myInPositionBand);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.leftSideDrive.disable();
		Robot.rightSideDrive.disable();
		currentMaxSpeed = 0;
		doneAccelerating = false;
		inVisionRange = false;

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
