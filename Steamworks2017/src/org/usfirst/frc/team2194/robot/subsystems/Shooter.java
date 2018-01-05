package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.RobotMap;

import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Shooter extends Subsystem {

	public double topShooterRPM;
	public double bottomShooterRPM;

	public void setTopShooterRPM(double speed) {
		RobotMap.topShooter.changeControlMode(TalonControlMode.Speed);
		RobotMap.topShooter.set(speed);
		topShooterRPM = speed;
	}

	public void setBottomShooterRPM(double speed) {
		RobotMap.bottomShooter.changeControlMode(TalonControlMode.Speed);
		RobotMap.bottomShooter.set(speed);
		bottomShooterRPM = speed;
	}

	public void setBottomShooterVoltage(double voltage) {
		RobotMap.bottomShooter.changeControlMode(TalonControlMode.PercentVbus);
		RobotMap.bottomShooter.set(voltage);

	}

	public void setTopShooterVoltage(double voltage) {
		RobotMap.topShooter.changeControlMode(TalonControlMode.PercentVbus);
		RobotMap.topShooter.set(voltage);

	}

	public void setBottomShooterPID(double p, double i, double d, double f, int profile) {
		RobotMap.bottomShooter.setPID(p, i, d, f, 0, f, profile);
	}

	public void setTopShooterPID(double p, double i, double d, double f, int profile) {
		RobotMap.topShooter.setPID(p, i, d, f, 0, f, profile);
	}

	public void disableTopShooter() {
		RobotMap.topShooter.changeControlMode(TalonControlMode.Disabled);
	}

	public void disableBottomShooter() {
		RobotMap.bottomShooter.changeControlMode(TalonControlMode.Disabled);
	}

	public boolean isStuck() {
		double currentPeak = 50;
		if (currentPeak <= RobotMap.topShooter.getOutputCurrent()
				|| currentPeak <= RobotMap.bottomShooter.getOutputCurrent())
			return true;
		else
			return false;

	}

	public void setBottomShooterPctVBUS(double set) {
		RobotMap.bottomShooter.changeControlMode(TalonControlMode.PercentVbus);
		RobotMap.bottomShooter.set(set);
	}

	public void setTopShooterPctVBUS(double set) {
		RobotMap.topShooter.changeControlMode(TalonControlMode.PercentVbus);
		RobotMap.topShooter.set(set);
	}

	public void setTopShooterRampRate(double rampRate) {
		RobotMap.topShooter.setCloseLoopRampRate(rampRate);
	}

	public void setBottomShooterRampRate(double rampRate) {
		RobotMap.bottomShooter.setCloseLoopRampRate(rampRate);
	}

	public void setShooterVoltage() {
		RobotMap.bottomShooter.configNominalOutputVoltage(+0f, -0f);
		RobotMap.bottomShooter.configPeakOutputVoltage(+12f, -12f);
		RobotMap.topShooter.configNominalOutputVoltage(+0f, -0f);
		RobotMap.topShooter.configPeakOutputVoltage(+12f, -12f);
	}

	public void setFeederVoltage() {
		RobotMap.secondFeeder.configNominalOutputVoltage(+0f, -0f);
		RobotMap.secondFeeder.configPeakOutputVoltage(+12f, -12f);
		RobotMap.firstFeeder.configNominalOutputVoltage(+0f, -0f);
		RobotMap.secondFeeder.configPeakOutputVoltage(+12f, -12f);
	}

	public void setACLE(int set) {
		RobotMap.bottomShooter.setAllowableClosedLoopErr(set);
		RobotMap.topShooter.setAllowableClosedLoopErr(set);
	}

	public void enableControl() {
		RobotMap.bottomShooter.enableControl();
		RobotMap.topShooter.enableControl();
	}

	// public void enableFeederControl() {
	// RobotMap.secondFeeder.enableControl();
	// RobotMap.firstFeeder.enableControl();
	// }

	public void enable() {
		RobotMap.bottomShooter.enable();
		RobotMap.topShooter.enable();
	}

	public void setSpeedLimit(double set) {
		RobotMap.bottomShooter.configMaxOutputVoltage(set);
		RobotMap.topShooter.configMaxOutputVoltage(set);
	}

	public void setShooterProfile(int profile) {
		RobotMap.bottomShooter.setProfile(profile);
		RobotMap.topShooter.setProfile(profile);
	}

	public void setupShooterMotors() {
		RobotMap.bottomShooter.setEncPosition(0);
		RobotMap.topShooter.setEncPosition(0);
		setACLE(0);
		setShooterProfile(0);
	}

	public double getBottomShooterCurrent() {
		return RobotMap.bottomShooter.getOutputCurrent();
	}

	public double getTopShooterCurrent() {
		return RobotMap.topShooter.getOutputCurrent();
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public void updateStatus() {
//	SmartDashboard.putNumber("Top Shooter Current", getTopShooterCurrent());
//		SmartDashboard.putNumber("Bottom Shooter Current", getBottomShooterCurrent());
		// SmartDashboard.putBoolean("Is Jammed?", isStuck());
//		SmartDashboard.putNumber("B Shooter Speed", Math.round(RobotMap.bottomShooter.getSpeed() * 100.) / 100.);
//		SmartDashboard.putNumber("T Shooter Speed", Math.round(RobotMap.topShooter.getSpeed() * 100.) / 100.);

		// SmartDashboard.putNumber("Top Shooter Vbus",
		// RobotMap.topShooter.getBusVoltage());
		// SmartDashboard.putNumber("Bottom Shooter Vbus",
		// RobotMap.bottomShooter.getBusVoltage());
		// SmartDashboard.putNumber("Bottom Shooter Error",
		// RobotMap.bottomShooter.getError());
		// SmartDashboard.putNumber("Top Shooter Error",
		// RobotMap.topShooter.getError());
		// SmartDashboard.putNumber("Bottom Shooter Amps",
		// Math.round(RobotMap.bottomShooter.getOutputCurrent() * 100.) / 100.);
		// SmartDashboard.putNumber("Top Shooter Amps",
		// Math.round(RobotMap.topShooter.getOutputCurrent() * 100.) / 100.);

	}
}
