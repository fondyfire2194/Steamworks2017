package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.commands.StartCompressor;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class AirCompressor extends Subsystem {
	public void start() {
		RobotMap.compressor.start();
	}

	public void stop() {
		RobotMap.compressor.stop();
	}
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	@Override
	public void initDefaultCommand() {

		setDefaultCommand(new StartCompressor());
	}

	public boolean isRunning() {
		return (!RobotMap.compressor.getPressureSwitchValue());
	}

	public void updateStatus() {
		SmartDashboard.putBoolean("Compressor Running", isRunning());
	}
}
