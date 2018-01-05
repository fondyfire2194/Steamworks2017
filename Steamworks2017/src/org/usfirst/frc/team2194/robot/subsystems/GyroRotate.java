/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author John
 */
public class GyroRotate extends PIDSubsystem {

	private static final double Kp = .01;
	private static final double Ki = 0.0;
	private static final double Kd = 0.0;

	public boolean gyroState = true;

	public double targetAngle;
	private double targetError;
	public double gyroKp = .05;
	public double gyroAngle;

	// Initialize your subsystem here
	public GyroRotate() {
		super("GyroRotate", Kp, Ki, Kd);
		getPIDController().setContinuous();
		getPIDController().setInputRange(0, 360);
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
		return Robot.imu.pidGet();
		// Return your input value for the PID loop
		// e.g. a sensor, like a potentiometer:
		// yourPot.getAverageVoltage() / kYourMaxVoltage;
	}

	@Override
	protected void usePIDOutput(double output) {
		RobotMap.driveLeftMotor1.set(output);
		RobotMap.driveRightMotor1.set(-output);
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

	public void setMaxOut(double speed) {
		getPIDController().setOutputRange(-speed, speed);
	}

	public double getError() {
		return getPIDController().getError();
	}

	public void resetGyro() {
		Robot.imu.reset();
	}

	public double getGyroAngle() {
		return Robot.imu.getAngle();
	}

	public float getGyroYaw() {
		return Robot.imu.getYaw();
	}

	public double getGyroYawComp() {
		return (getGyroYaw() * gyroKp);
	}

	public boolean inPosition(double inPositionBand) {
		// return getPIDController().onTarget();
		double gyroAngle = 0;
		if (getGyroAngle() > 0) {
			gyroAngle = getGyroAngle();

		} else {
			gyroAngle = 360 + getGyroAngle();
		}
		return (Math.abs(getPIDController().getSetpoint() - gyroAngle) < inPositionBand);
	}

	public boolean isEnabled() {
		return getPIDController().isEnabled();
	}

	public double getCompassHeading() {
		return Robot.imu.getCompassHeading();
	}

	public boolean isMoving() {
		return Robot.imu.isMoving();
	}

	public boolean isRotating() {
		return Robot.imu.isRotating();
	}

	public void setTargetAngle(double targetAngle) {
		this.targetAngle = targetAngle;
	}

	public double getTargetError() {
		targetError = getGyroAngle() - targetAngle;
		if (Math.abs(targetError) < 180)
			return targetError;
		else if (targetError >= 180)
			return 360 - targetError;
		else
			return 360 + targetError;
	}

	public double getTargetYawComp() {
		if ((Robot.gyroRotate.getTargetError() * gyroKp) >= .1) {
			return .1;
		} else if (Robot.gyroRotate.getTargetError() <= -.1) {
			return -.1;
		} else {
			return getTargetError() * gyroKp;
		}
	}

	public void updateStatus() {

		// SmartDashboard.putNumber("IMU_Yaw", Math.round(Robot.imu.getYaw() *
		// 100.) / 100.);
		SmartDashboard.putNumber("Target Error", Math.round(getTargetError() * 100.) / 100.);
		// SmartDashboard.putNumber("Yaw Comp", Math.round(getGyroYawComp() *
		// 100.) / 100.);
		// SmartDashboard.putNumber("Gyro Comp Kp", Math.round(gyroKp * 100.) /
		// 100.);

		SmartDashboard.putNumber("Gyro Angle", Math.round(getGyroAngle() * 100.) / 100.);
		// SmartDashboard.putNumber("Target Gyro Angle", Math.round(targetAngle
		// * 100.) / 100.);
		// SmartDashboard.putNumber("TargetYaw Comp",
		// Math.round(getTargetYawComp() * 100.) / 100.);

		// SmartDashboard.putBoolean("Is Calibrating",
		// Robot.imu.isCalibrating());
	}
}
