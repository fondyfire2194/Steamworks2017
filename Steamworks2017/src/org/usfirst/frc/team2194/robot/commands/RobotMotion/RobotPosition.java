package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RobotPosition extends Command {
	private double mySpeed;
	private double inPositionBandEncoderCounts;
	private boolean myDisableWhenDone = false;
	private boolean myEndItNow = false;
	private double myTimeout;
	private double rampIncrement;
	private double distanceEncoderCounts;
	private boolean useGyroComp;

	private double comp;
	private boolean doneAccelerating = false;
	public static double currentMaxSpeed;
	private static double myEndpoint;

	public enum CompMode {
		none, gyro
	}

	public enum MoveMode {
		relative, absolute
	}

	// side distances are in inches
	// side speeds are in per unit where .25 = 25%
	// inPositionband is in inches

	public RobotPosition(double distance, double speed, MoveMode type, CompMode compMode, boolean endItNow,
			double inPositionBand, boolean disableWhenDone, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain);
		requires(Robot.leftSideDrive);
		requires(Robot.rightSideDrive);

		myEndpoint = distance;
		mySpeed = speed;
		myEndItNow = endItNow;
		inPositionBandEncoderCounts = inPositionBand * Robot.driveTrain.encoderCountsPerInch;
		myDisableWhenDone = disableWhenDone;
		myTimeout = timeout;
		switch (compMode) {
		case none:
			useGyroComp = false;
			break;
		case gyro:
			useGyroComp = true;
			break;
		}

		switch (type) {
		case relative:
			myEndpoint += (Robot.leftSideDrive.getPosition() + Robot.rightSideDrive.getPosition()) / 2;
			break;

		case absolute:

			break;
		}
		distanceEncoderCounts = myEndpoint * Robot.driveTrain.encoderCountsPerInch;

	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		// Robot.leftSideDrive.setPIDF(Robot.prefs.getDouble("Left Position Kp",
		// .003),
		// Robot.prefs.getDouble("Left Position Ki", .003), 0, 0);
		// Robot.rightSideDrive.setPIDF(Robot.prefs.getDouble("Right Position
		// Kp", .003),
		// Robot.prefs.getDouble("Right Position Ki", .003), 0, 0);
		rampIncrement = mySpeed / 25;
		Robot.leftSideDrive.setSetpoint(distanceEncoderCounts);
		Robot.rightSideDrive.setSetpoint(distanceEncoderCounts);
		Robot.leftSideDrive.enable();
		Robot.rightSideDrive.enable();
		Robot.isPositioning = true;
		setTimeout(myTimeout);
		doneAccelerating = false;
		currentMaxSpeed = 0;
		// Robot.directionIsMinus = false;
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

		// need to determine motion direction so max and min outputs of PID loop
		// can be set

		Robot.leftSideDrive.getComp(currentMaxSpeed, comp);

		Robot.rightSideDrive.getComp(currentMaxSpeed, comp);

		if(doneAccelerating && Robot.driveTrain.isRightStopped()){
			Robot.rightSideDrive.disable();
		}
		
		if(doneAccelerating && Robot.driveTrain.isLeftStopped()){
			Robot.leftSideDrive.disable();
		}
		
		
		
		
//		 SmartDashboard.putNumber("leftspeed", Math.round(mySpeed * 100.) /
//		 100.);
//		 SmartDashboard.putNumber("rightspeed", Math.round(mySpeed * 100.)
//		 / 100.);
//		 SmartDashboard.putNumber("maxspeed", Math.round(currentMaxSpeed *
//		 100.) / 100.);
//		 SmartDashboard.putNumber("myspeed", Math.round(mySpeed * 100.) /
//		 100.);
//		 SmartDashboard.putNumber("comp", Math.round(comp * 100.) / 100.);
//		 SmartDashboard.putNumber("myendpt", Math.round(myEndpoint * 100.) /
//		 100.);
//		 SmartDashboard.putNumber("distec", Math.round(distanceEncoderCounts * 100.) /
//		 100.);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isTimedOut() || myEndItNow || (Robot.leftSideDrive.inPosition(inPositionBandEncoderCounts)
				|| Robot.rightSideDrive.inPosition(inPositionBandEncoderCounts)) || (doneAccelerating && Robot.driveTrain.isStopped());
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		if (myDisableWhenDone) {
			Robot.leftSideDrive.disable();
			Robot.rightSideDrive.disable();
		}
		doneAccelerating = false;
		Robot.isPositioning = false;
		Robot.stopMotion = false;
		currentMaxSpeed = 0;

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
