package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class JoystickGearPickupComp extends Command {

	public JoystickGearPickupComp() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.visionCompJoystick = false;
		Robot.hookVisionTurnedOn = false;
		Robot.gearPickupJoystick = true;
		Robot.gearVisionTurnedOn = true;
		RobotMap.greenLedsOn.set(Value.kOff);
		Robot.gyroCompJoystick = false;
		Robot.gyroRotate.targetAngle = Robot.gyroRotate.getGyroAngle();
		Robot.gearPickupJoystick = true;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (Robot.oi.joystick1.getY() < -.1) {
			Robot.driveTrain.driveStraight(-Robot.oi.joystick1.getY(), -Robot.gearVision.getXError() * 0.001);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return (Robot.gearVision.isGearInCenter() && Robot.gearVision.isGearInThresh()) || !Robot.gearPickupJoystick;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.gearPickupJoystick = false;
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
