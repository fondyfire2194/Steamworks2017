package org.usfirst.frc.team2194.robot.commands.Climber;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class StartAgitator extends Command {
	private double mySpeed;

	public StartAgitator(double speed) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		mySpeed = speed;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		RobotMap.agitator.enable();
		Robot.agitator.setPctVbus(mySpeed);
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
