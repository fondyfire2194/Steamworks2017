package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RunFromJoystick;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain extends Subsystem {
	RobotDrive drive = RobotMap.drive;
	Encoder leftEncoder = RobotMap.leftEncoder;
	Encoder rightEncoder = RobotMap.rightEncoder;
	// Encoder calibration notes
	// There are 100 encoder counts per rev of the drive wheel
	// A 3" diameter wheel will have an encoderCountsPerInch of 100/(3*Pi)
	// A 4" diameter wheel will have an encoderCountsPerInch of 100/(4*Pi)
	// The larger the wheel, the smaller the encoderCountsPerInch
	// As a wheel wears, it will go less far so its value must go up
	// Inches are calclated by dividing encoder counts by encoderCountsPerInch
	// Example
	// A 3" wheel running with 4" encoderCountsPerInch will move 3*Pi inches in
	// one rev but have an encoder reading of 4*Pi Inches
	// So the tape measure reading is less than the encoder reading and the
	// encoderCountsPerInch are too low.
	// Conversely a 4" wheel running with 3" EncoderCountsPerInch will give
	// encoder readings less than the tape measure and the value should go down

	// Reset encoders and move robot a minimum of 100" using tape measure
	// Compare encoder readings and tape measure reading
	// If necessary, calculate encoderCountsPerInch using following formula

	// New Value = (OldValue * encoder reading)/tape measure distance

	public double encoderCountsPerInch = 8.189; //8.049;//7.8417;//COMPETITION
//	public double encoderCountsPerInch = 8.53;// PRACTISE

	private double comp;
	double rightMultiplier;
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new RunFromJoystick());
		// setDefaultCommand(new RunFromGamepad());
	}

	public void driveRobot(double leftSpeed, double rightSpeed) {
		double maxMultiplier = .5;
		double minMultiplier = .35;
		
		if(Robot.isInHighGear)
		{
			maxMultiplier = .35;
			minMultiplier = .15;
		}
		rightMultiplier = minMultiplier + ((1 - Math.abs(leftSpeed)) * (maxMultiplier - minMultiplier));

		RobotMap.driveLeftMotor1.set(leftSpeed + (rightSpeed * rightMultiplier));
		RobotMap.driveRightMotor1.set(leftSpeed - (rightSpeed * rightMultiplier));
	}

	public void driveStraight(double speed, double comp) {
		if (speed != 0) {
			RobotMap.driveLeftMotor1.set(speed - comp);
			RobotMap.driveRightMotor1.set(speed + comp);
		} else {
			RobotMap.driveLeftMotor1.set(0);
			RobotMap.driveRightMotor1.set(0);
		}

	}

	public double getLeftPosition() {
		return Math.round(leftEncoder.getDistance() * 10. / encoderCountsPerInch) / 10.;
	}

	public double getRightPosition() {
		return Math.round(rightEncoder.getDistance() * 10. / encoderCountsPerInch) / 10.;
	}

	public double getRobotPosition() {
		return (getLeftPosition() + getRightPosition()) / 2;
	}

	public double getLeftInchesPerSecond() {
		return RobotMap.leftEncoder.getRate() / encoderCountsPerInch;
	}

	public double getLeftInchesPerMinute() {
		return getLeftInchesPerSecond() * 60;
	}

	public double getRightInchesPerSecond() {
		return RobotMap.rightEncoder.getRate() / encoderCountsPerInch;
	}

	public double getRightInchesPerMinute() {
		return getRightInchesPerSecond() * 60;
	}

	public double getInchesPerSecond() {
		return (getRightInchesPerSecond() + getLeftInchesPerSecond()) / 2;
	}

	public double getInchesPerMinute() {
		return getInchesPerSecond() * 60;
	}

	public boolean isStopped() {
		return RobotMap.rightEncoder.getStopped() && RobotMap.leftEncoder.getStopped();
	}

	public boolean isLeftStopped() {
		return RobotMap.leftEncoder.getStopped();
	}

	public boolean isRightStopped() {
		return RobotMap.rightEncoder.getStopped();
	}

	public void updateStatus() {
		SmartDashboard.putNumber("Robot LPosition", Math.round(getLeftPosition() * 100.) / 100.);
		SmartDashboard.putNumber("Robot RPosition", Math.round(getRightPosition() * 100.) / 100.);
		SmartDashboard.putNumber("Robot LPower", Math.round(RobotMap.driveLeftMotor1.get() * 100.) / 100.);
		SmartDashboard.putNumber("Robot RPower", -Math.round(RobotMap.driveRightMotor1.get() * 100.) / 100.);
		SmartDashboard.putNumber("Robot AVG Position", (getLeftPosition() + getRightPosition())/2);
//		SmartDashboard.putNumber("Robot IPM", getInchesPerMinute());


	}
}
