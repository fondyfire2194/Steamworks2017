package org.usfirst.frc.team2194.robot.commands.Feeder;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class ReverseSecondFeeder extends InstantCommand {

    public ReverseSecondFeeder() {
        super();
        
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called once when the command executes
    @Override
	protected void initialize() {
    	Robot.feeder.enableFeederControl();
        Robot.feeder.setSecondFeederPctVbus(-.5);
    }

}
