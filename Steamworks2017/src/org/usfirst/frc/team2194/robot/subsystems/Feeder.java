package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Feeder extends Subsystem {

	public void setFirstFeederPctVbus(double PCT) {
		RobotMap.firstFeeder.changeControlMode(TalonControlMode.PercentVbus);
		RobotMap.firstFeeder.set(PCT);
	}

	public void setSecondFeederPctVbus(double PCT) {
		RobotMap.secondFeeder.changeControlMode(TalonControlMode.PercentVbus);
		RobotMap.secondFeeder.set(PCT);
	}

	public void disableFirstFeeder() {
		RobotMap.firstFeeder.changeControlMode(TalonControlMode.Disabled);
	}

	public void disableSecondFeeder() {
		RobotMap.secondFeeder.changeControlMode(TalonControlMode.Disabled);
	}
	
	public double getFirstFeederCurrent(){
		return RobotMap.firstFeeder.getOutputCurrent();
	}
	
	public double getSecondFeederCurrent(){
		return RobotMap.secondFeeder.getOutputCurrent();
	}

	public boolean isStuck() {
		double currentPeak = 50;
		if (currentPeak <= RobotMap.secondFeeder.getOutputCurrent()
				|| currentPeak <= RobotMap.firstFeeder.getOutputCurrent()) {
			return true;
		} else {
			return false;
		}
	}

	public void updateStatus() {
//		SmartDashboard.putNumber("First Feeder Current", getFirstFeederCurrent());
//		SmartDashboard.putNumber("Second Feeder Current", getSecondFeederCurrent());
	}

	public void enableFeederControl() {
		//RobotMap.secondFeeder.enableControl();
		//RobotMap.firstFeeder.enableControl();
		RobotMap.firstFeeder.enable();
		RobotMap.secondFeeder.enable();
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
