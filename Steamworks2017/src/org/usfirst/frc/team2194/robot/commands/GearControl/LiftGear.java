package org.usfirst.frc.team2194.robot.commands.GearControl;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LiftGear extends Command {

    public LiftGear() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    @Override
	protected void initialize() {
    	setTimeout(0.5);
    	RobotMap.gearLift.set(DoubleSolenoid.Value.kReverse);
    	
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    @Override
	protected void end() {
    	RobotMap.gearLift.set(DoubleSolenoid.Value.kOff);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
	protected void interrupted() {
    }
}
