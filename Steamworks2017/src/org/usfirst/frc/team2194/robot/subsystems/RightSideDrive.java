/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 * @author John
 */
public class RightSideDrive extends PIDSubsystem {

	private static final double Kp = .005;
	private static final double Ki = 0.0;
	private static final double Kd = 0.0;

	// Initialize your subsystem here
	public RightSideDrive() {
		super("RightPositionLinear", Kp, Ki, Kd);
		getPIDController().disable();
		getPIDController().setPercentTolerance(5);

		// Use these to get going:
		// setSetpoint() - Sets where the PID controller should move the system
		// to
		// enable() - Enables the PID controller.
	}

	public void setPIDF(double Kp, double Ki, double Kd, double Kf) {
		getPIDController().setPID(Kp, Ki, Kd, Kf);
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	@Override
	protected double returnPIDInput() {
		return RobotMap.rightEncoder.pidGet();
		// Return your input value for the PID loop
		// e.g. a sensor, like a potentiometer:
		// yourPot.getAverageVoltage() / kYourMaxVoltage;
	}

	@Override
	protected void usePIDOutput(double output) {
		RobotMap.driveRightMotor1.set(output);
		// Use output to drive your system, like a motor
		// e.g. yourMotor.set(output);
	}

	public void enablePID() {
		getPIDController().enable();

	}

	public void disablePID() {
		getPIDController().disable();
	}

	@Override
	public void setSetpoint(double setpoint) {
		getPIDController().setSetpoint(setpoint);
	}

	public double getSetpoint() {
		return getPIDController().getSetpoint();
	}

	public void getComp(double speed, double comp) {
		if (Math.abs(comp) > speed)
			comp = 0;

		if (getError() >= 0)
			getPIDController().setOutputRange(0, (speed + comp));
		else
			getPIDController().setOutputRange((-speed + comp), 0);
	}

	public void setMaxOutputRange(double min, double max) {
		getPIDController().setOutputRange(min, max);
	}

	public boolean inPosition(double inPositionBand) {
		return (Math.abs(getPIDController().getSetpoint() - RobotMap.rightEncoder.getDistance()) < inPositionBand);
	}

	public double getError() {
		return getPIDController().getError();
	}

	public boolean isEnabled() {
		return getPIDController().isEnabled();
	}

	public void updateStatus() {
		// SmartDashboard.putNumber("Right Error", getError());
		// SmartDashboard.putNumber("Right Setpoint",
		// getPIDController().getSetpoint());
		// SmartDashboard.putBoolean("RightDriveEnabled",
		// getPIDController().isEnabled());
	}
}
