package org.usfirst.frc.team2194.robot.commands.Feeder;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class StartSecondFeeder extends Command {

	public StartSecondFeeder() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);

	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.feeder.enableFeederControl();
		Robot.feeder.setSecondFeederPctVbus(0.5);
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
	    	end();
	    }
}