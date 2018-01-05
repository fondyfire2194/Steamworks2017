package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.RobotMap;

import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Agitator extends Subsystem {

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public void setPctVbus(double PCT) {
		RobotMap.agitator.changeControlMode(TalonControlMode.PercentVbus);
		RobotMap.agitator.set(PCT);
	}

	public void disable() {
		RobotMap.agitator.changeControlMode(TalonControlMode.Disabled);
	}

	public boolean isSwitchPressed() {
		return RobotMap.climberTrigger.get();
	}

	public void updateStatus() {
		// SmartDashboard.putBoolean("Lifter Trigger", isSwitchPressed());
	}

}
