package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunFromGamepad extends Command {

	public RunFromGamepad() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.leftSideDrive.disablePID();
		Robot.rightSideDrive.disablePID();

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (!Robot.visionCompJoystick && !Robot.gyroCompJoystick) {

			if (Math.abs(Robot.oi.gamepad.getY()) > 0.1 || Math.abs(Robot.oi.gamepad.getX()) > 0.1) {
				Robot.driveTrain.driveRobot(-Robot.oi.gamepad.getY(), Robot.oi.gamepad.getX());
			} else
				Robot.driveTrain.driveRobot(0, 0);
		}
		if (Robot.gyroCompJoystick) {
			if (Math.abs(Robot.oi.gamepad.getY()) > 0.1) {
				Robot.driveTrain.driveStraight(-Robot.oi.gamepad.getY(), Robot.gyroRotate.getTargetYawComp());
			} else {
				Robot.driveTrain.driveStraight(0, 0);
			}
		}
		if (Robot.visionCompJoystick) {
			if (Math.abs(Robot.oi.gamepad.getY()) > 0.1) {
				Robot.driveTrain.driveStraight(-Robot.oi.gamepad.getY(), Robot.activeMotionComp);
			} else {
				Robot.driveTrain.driveStraight(0, 0);
			}
		}

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
