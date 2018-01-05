package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunFromJoystick extends Command {

	private double moveSpeed;

	public RunFromJoystick() {
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
		double deadBand = 0.25;

		if (!Robot.visionCompJoystick && !Robot.gyroCompJoystick) {

			if (Math.abs(Robot.oi.joystick1.getY()) > deadBand || Math.abs(Robot.oi.joystick1.getZ()) > deadBand) {
				Robot.driveTrain.driveRobot(getOutput(deadBand, 1, -Robot.oi.joystick1.getY()),
						getOutput(deadBand, 1, (Robot.oi.joystick1.getZ() + Robot.oi.joystick1.getX())));
			} else
				Robot.driveTrain.driveRobot(0, 0);
		}
		if (Robot.gyroCompJoystick) {
			if (Math.abs(Robot.oi.joystick1.getY()) > deadBand) {
				Robot.driveTrain.driveStraight(getOutput(deadBand, 1, -Robot.oi.joystick1.getY()),
						Robot.gyroRotate.getTargetYawComp());
			} else {
				Robot.driveTrain.driveStraight(0, 0);
			}
		}
		if (Robot.visionCompJoystick) {
			if (Math.abs(Robot.oi.joystick1.getY()) > deadBand
					&& (!Robot.stopMotion || Robot.oi.joystick1.getY() > 0)) {
				// double robotError =
				// Robot.ultraSound.readLeftUltrasoundInches() -
				// Robot.releaseGearDistance;
				// double multiplier =
				// (robotError/(Robot.latchAngleCompPixelsDistance -
				// Robot.releaseGearDistance));
				//
				// if(Robot.ultraSound.readLeftUltrasoundInches()>Robot.latchAngleCompPixelsDistance
				// || Robot.oi.joystick1.getY()>0)
				// {
				// moveSpeed = Robot.oi.joystick1.getY();
				// }
				// else
				// {
				// if(multiplier < 0.25)
				// {
				// multiplier = 0.25;
				// }
				// if(Robot.ultraSound.readLeftUltrasoundInches()<Robot.releaseGearDistance)
				// {
				// multiplier = 0;
				// }
				// moveSpeed = Robot.oi.joystick1.getY() * multiplier;
				// }
				//
				moveSpeed = Robot.oi.joystick1.getY();
				Robot.driveTrain.driveStraight(getOutput(deadBand, 1, -moveSpeed), Robot.activeMotionComp);
			} else {
				Robot.driveTrain.driveStraight(0, 0);
			}
		}

	}

	public double getOutput(double deadband, double maxOutput, double axis) {
		double output;
		if (Math.abs(axis) < deadband) {
			output = 0;
		} else {
			double motorOutput = (((Math.abs(axis) - deadband) / (1 - deadband)) * (axis / Math.abs(axis)));
			output = motorOutput * maxOutput;
		}
		return output;
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
