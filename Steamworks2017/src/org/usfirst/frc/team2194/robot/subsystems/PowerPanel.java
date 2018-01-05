package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.commands.ControlPower;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PowerPanel extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new ControlPower());
	}

	public double getTotalCurrent() {
		return RobotMap.pdp.getTotalCurrent();
	}

	public double getTotalEnergy() {
		return RobotMap.pdp.getTotalEnergy();
	}

	public double getTotalPower() {
		return RobotMap.pdp.getTotalPower();
	}

	public double getVoltage() {
		return RobotMap.pdp.getVoltage();
	}

	public double getChannelCurrent(int channel) {
		return RobotMap.pdp.getCurrent(channel);
	}

	public void resetTotalEnergy() {
		RobotMap.pdp.resetTotalEnergy();
	}

	public void updateStatus() {
//		SmartDashboard.putNumber("Energy Used", getTotalEnergy());
//		SmartDashboard.putNumber("Amps Total", getTotalCurrent());
		SmartDashboard.putNumber("Channel 0", getChannelCurrent(0));
		SmartDashboard.putNumber("Channel 1", getChannelCurrent(1));
		SmartDashboard.putNumber("Channel 2", getChannelCurrent(2));
		SmartDashboard.putNumber("Channel 3", getChannelCurrent(3));
	}
}
