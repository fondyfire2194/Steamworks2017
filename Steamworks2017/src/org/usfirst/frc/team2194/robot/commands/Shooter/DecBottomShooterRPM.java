package org.usfirst.frc.team2194.robot.commands.Shooter;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class DecBottomShooterRPM extends InstantCommand {

	public DecBottomShooterRPM() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called once when the command executes
	@Override
	protected void initialize() {
		Robot.shooter.bottomShooterRPM *= .95;
		if (Robot.shooter.bottomShooterRPM < 2000)
			Robot.shooter.bottomShooterRPM = 2000;
		Robot.shooter.setBottomShooterRPM(Robot.shooter.bottomShooterRPM);
	}

}
