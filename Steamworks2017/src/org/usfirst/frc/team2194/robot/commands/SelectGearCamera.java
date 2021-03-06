package org.usfirst.frc.team2194.robot.commands;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class SelectGearCamera extends InstantCommand {

	public SelectGearCamera() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called once when the command executes
	protected void initialize() {
		if (!Robot.hookVisionTurnedOn) {
			Robot.viewHookCamera = false;
			Robot.viewGearCamera = true;
		}
	}

}
