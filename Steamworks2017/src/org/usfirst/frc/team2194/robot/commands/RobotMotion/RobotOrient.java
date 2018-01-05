/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 * @author John
 */
public class RobotOrient extends Command {
	private double mySpeed;
	private double myAngle;
	private double myTimeout;
	private double myInPositionBand;
	private boolean doneAccelerating;
	public static double currentMaxSpeed;
	private double rampIncrement;

	public enum MoveModeRot {
		relative, absolute
	}

	public RobotOrient(double angle, double speed, MoveModeRot type, double inPositionBand, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.gyroRotate);
		requires(Robot.leftSideDrive);
		requires(Robot.rightSideDrive);
		requires(Robot.driveTrain);

		mySpeed = speed;
		myAngle = angle;
		myTimeout = timeout;
		myInPositionBand = inPositionBand;

		switch (type) {
		case relative:
			myAngle += Robot.gyroRotate.getGyroAngle();
			if (myAngle >= 360)
				myAngle -= 360;
			if (myAngle < 0)
				myAngle += 360;
			break;

		case absolute:

			break;
		}

	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.gyroRotate.setPIDF(Robot.prefs.getDouble("Gyro Position Kp", .003),
				Robot.prefs.getDouble("Gyro Position Ki", .003), 0, 0);
		rampIncrement = mySpeed / 5;
		Robot.gyroRotate.setSetpoint(myAngle);
		Robot.gyroRotate.enablePID();
		Robot.isOrienting = true;
		doneAccelerating = false;
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

		Robot.gyroRotate.setMaxOut(mySpeed);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isTimedOut() || Robot.gyroRotate.inPosition(myInPositionBand);

	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.gyroRotate.disable();
		Robot.isOrienting = false;
		Robot.motionSeen = false;

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
