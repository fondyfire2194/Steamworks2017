package org.usfirst.frc.team2194.robot.commands.Shooter;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class DecTopShooterRPM extends InstantCommand {

	public DecTopShooterRPM() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called once when the command executes
	@Override
	protected void initialize() {
		Robot.shooter.topShooterRPM *= .95;
		if (Robot.shooter.topShooterRPM < -2000)
			Robot.shooter.topShooterRPM = -2000;
		Robot.shooter.setTopShooterRPM(Robot.shooter.topShooterRPM);

	}

}
