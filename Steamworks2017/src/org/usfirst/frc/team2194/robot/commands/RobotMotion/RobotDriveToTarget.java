package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RobotDriveToTarget extends Command {
	private double mySpeed;
	private boolean myEndItNow;
	private double myTimeout;
	private double rampIncrement;
	private boolean useGyroComp;
	private boolean useVisionComp;

	private boolean doneAccelerating;
	public static double currentMaxSpeed;
	private double startingTargetAngle;

	public double myDistance;
	public double slowDownInches = 20;
	public boolean decelerate;
	private double myEndpoint;
	private double startOfVisionPoint = 65;

	private double endOfVisionPoint = 10;
	private boolean inVisionRange;

	// side distances are in inches
	// side speeds are in per unit where .25 = 25%
	// inPositionband is in inches

	public RobotDriveToTarget(double distance, double speed, boolean endItNow, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain);

		myEndpoint = distance;
		mySpeed = speed;
		myEndItNow = endItNow;
		myDistance = distance;
		myTimeout = timeout;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		rampIncrement = mySpeed / 25;
		setTimeout(myTimeout);
		Robot.isPositioning = true;
		startingTargetAngle = Robot.gyroRotate.targetAngle;
		currentMaxSpeed = 0;
		doneAccelerating = false;
		decelerate = false;
		slowDownInches = Robot.prefs.getDouble("HookSldnDist", 20);
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
		Robot.remainingDistanceToHook = myEndpoint - Robot.driveTrain.getLeftPosition();

		if (doneAccelerating && !decelerate && Robot.remainingDistanceToHook < slowDownInches) {
			decelerate = true;
		}
		if (decelerate) {
			currentMaxSpeed = (mySpeed * Robot.remainingDistanceToHook) / slowDownInches;
			if (currentMaxSpeed < .3)
				currentMaxSpeed = .3;
		}

		// in vision zone keep gyro target angle current in case need to switch
		// over to gyro

		inVisionRange = Robot.remainingDistanceToHook < startOfVisionPoint
				&& Robot.remainingDistanceToHook > endOfVisionPoint;

		useVisionComp = inVisionRange && Robot.visionData.isValidNumberImages();
		useGyroComp = !useVisionComp;

		if (useVisionComp) {
			Robot.activeMotionComp = Robot.visionData.getVisionComp();
			Robot.gyroRotate.targetAngle = Robot.gyroRotate.getGyroAngle();
		}
		if (useGyroComp)
			Robot.activeMotionComp = Robot.gyroRotate.getTargetYawComp();

		Robot.driveTrain.driveStraight(currentMaxSpeed, Robot.activeMotionComp);

		// SmartDashboard.putBoolean("VComp", useVisionComp);
		// SmartDashboard.putBoolean("GComp", useGyroComp);
		// SmartDashboard.putBoolean("InVRange", inVisionRange);
		//
		// SmartDashboard.putBoolean("Deceling", decelerate);
		// SmartDashboard.putNumber("RemDist", Robot.remainingDistanceToHook);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isTimedOut() || myEndItNow || Robot.driveTrain.getLeftPosition() >= (myEndpoint);// ||
																								// decelerate
																								// &&
																								// Robot.driveTrain.isStopped();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.driveTrain.driveStraight(0, 0);
		Robot.isPositioning = false;
		Robot.stopMotion = false;
		doneAccelerating = false;
		decelerate = false;
		Robot.gyroRotate.targetAngle = startingTargetAngle;
		currentMaxSpeed = 0;
		inVisionRange = false;

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
