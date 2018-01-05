package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.commands.UpdateAllPrefsValues;

import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class JoystickGyroComp extends Command {

	public JoystickGyroComp() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		(new UpdateAllPrefsValues()).start();
		Robot.visionCompJoystick = false;
		Robot.gyroCompJoystick = true;
		Robot.gearPickupJoystick = false;
		Robot.hookVisionTurnedOn = false;
		RobotMap.greenLedsOn.set(Value.kOff);
		RobotMap.cameraLEDs.set(0);
		Robot.gyroRotate.targetAngle = Robot.gyroRotate.getGyroAngle();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return true;
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
