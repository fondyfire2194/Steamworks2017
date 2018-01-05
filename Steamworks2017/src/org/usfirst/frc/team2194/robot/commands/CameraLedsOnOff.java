package org.usfirst.frc.team2194.robot.commands;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CameraLedsOnOff extends Command {
private boolean myState;
    public CameraLedsOnOff(boolean state) {
		myState = state;

        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		if (myState) {
			RobotMap.greenLedsOn.set(Value.kOn);
			RobotMap.cameraLEDs.set(Robot.ledPowerHookCam);
		} else { 
			RobotMap.greenLedsOn.set(Value.kOff);
			RobotMap.cameraLEDs.set(0);

		}

	}



    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
