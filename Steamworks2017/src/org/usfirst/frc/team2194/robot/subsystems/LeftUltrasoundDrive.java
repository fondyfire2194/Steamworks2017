/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 * @author John
 */
public class LeftUltrasoundDrive extends PIDSubsystem {

	private static final double Kp = .02;
	private static final double Ki = 0.0;
	private static final double Kd = 0.0;

	public boolean gyroState = true;
	public double targetAngle = 0;
	double lastTime = 0;

	// Initialize your subsystem here
	public LeftUltrasoundDrive() {
		super("UltrasoundPosition", Kp, Ki, Kd);
		// getPIDController().setContinuous();
		// getPIDController().setInputRange(0, 360);
		getPIDController().disable();
		getPIDController().setPercentTolerance(1);

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
		return Robot.ultraSound.readLeftUltrasoundInches();
		// Return your input value for the PID loop
		// e.g. a sensor, like a potentiometer:
		// yourPot.getAverageVoltage() / kYourMaxVoltage;
	}

	@Override
	protected void usePIDOutput(double output) {
		// Ultrasound counts down to target so invert direction to compensate
		RobotMap.driveLeftMotor1.set(-output);
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
	// the ultrasound feedback count decreases in the positive travel direction
	// of the robot
	// this is the opposite of a typical encoder base axis where a + move
	// increases the count +
	// so a 2 inch to a 10 inch position move gives a positive closed loop error
	// but means a negative robot direction
	// This is compensated by negating the PID output to the drive axis
	// So we need to reverse the error sign also

	public double getError() {
		return -getPIDController().getError();// negate for same reason as
												// output is negated
	}

	public void getComp(double speed, double comp) {
		if (Math.abs(comp) > speed)
			comp = 0;
		if (getError() >= 0)// error alraedy negated so use same as encoder axis
			getPIDController().setOutputRange(-(speed - comp), (speed - comp));
		else
			getPIDController().setOutputRange((-speed - comp), 0);
	}

	public void setMaxOutputRange(double min, double max) {
		getPIDController().setOutputRange(min, max);
	}

	public boolean inPosition(double inPosition) {
		// return getPIDController().onTarget();
		return (Math.abs(getPIDController().getSetpoint() - Robot.ultraSound.readLeftUltrasoundInches()) < inPosition);
	}

	public boolean isEnabled() {
		return getPIDController().isEnabled();
	}

	public boolean isMoving() {
		return Robot.imu.isMoving();
	}

	public void updateStatus() {

	}
}
