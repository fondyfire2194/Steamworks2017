/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team2194.robot.subsystems;

/**
 *
 * @author John
 */
/*
public class LeftLidarDrive extends PIDSubsystem {

	private static final double Kp = .02;
	private static final double Ki = 0.0;
	private static final double Kd = 0.0;

	public boolean gyroState = true;
	public double targetAngle = 0;
	double lastTime = 0;

	// Initialize your subsystem here
	public LeftLidarDrive() {
		super("LidarPosition", Kp, Ki, Kd);
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

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	protected double returnPIDInput() {
		return Robot.lidarSensor.readLidarSensorInch();
		// Return your input value for the PID loop
		// e.g. a sensor, like a potentiometer:
		// yourPot.getAverageVoltage() / kYourMaxVoltage;
	}

	protected void usePIDOutput(double output) {
		// Lidar counts down to target so invert direction to compensate
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

	public void setSetpoint(double setpoint) {
		getPIDController().setSetpoint(setpoint);
	}

	public void setMaxOut(double speed) {
		getPIDController().setOutputRange(-speed, speed);
	}

	public boolean inPosition() {
		// return getPIDController().onTarget();
		return (Math.abs(getPIDController().getSetpoint() - Robot.lidarSensor.readLidarSensorInch()) < 1);
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
*/
