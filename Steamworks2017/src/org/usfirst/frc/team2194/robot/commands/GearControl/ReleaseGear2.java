package org.usfirst.frc.team2194.robot.commands.GearControl;

import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ReleaseGear2 extends Command {

	public ReleaseGear2() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		setTimeout(.5);
		RobotMap.gearGrip.set(DoubleSolenoid.Value.kForward);

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		RobotMap.gearGrip.set(DoubleSolenoid.Value.kForward);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isTimedOut();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		RobotMap.gearGrip.set(DoubleSolenoid.Value.kOff);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
