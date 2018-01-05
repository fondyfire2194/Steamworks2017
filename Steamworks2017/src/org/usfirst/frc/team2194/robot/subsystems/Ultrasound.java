package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Ultrasound extends Subsystem {
	// private double ultrasoundVoltsPerCm = .0049;
	// private double leftUltrasoundVoltsPerInch = .0098;// 0.0161798;// .0098;
	// private double rightUltrasoundVoltsPerInch = .0098 * 2; // 0.0161790;//
//	 .0098;
	 double leftMinVolts = 1.00;
	 double leftMaxVolts = 4.78;
	 double leftMinDistance = 6;
	 double leftMaxDistance = 37;
	
	 double rightMinVolts = 1.00;
	 double rightMaxVolts = 4.78;
	 double rightMinDistance = 6;
	 double rightMaxDistance = 37;
	// 12 volt operation
//	double leftMinVolts = 1.00;
//	double leftMaxVolts = 3.55;
//	double leftMinDistance = 6;
//	double leftMaxDistance = 28;
//
//	double rightMinVolts = 1.00;
//	double rightMaxVolts = 3.55;
//	double rightMinDistance = 6;
//	double rightMaxDistance = 28;

	double ultrasoundSpacing = 20.5;

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public double readLeftUltrasoundVolts() {
		return RobotMap.leftUltrasoundSensor.getAverageVoltage();
	}

	public double readLeftUltrasoundInches() {
		return leftMinDistance + (readLeftUltrasoundVolts() - leftMinVolts)
				* ((leftMaxDistance - leftMinDistance) / (leftMaxVolts - leftMinVolts));
	}

	public double readRightUltrasoundVolts() {
		return RobotMap.rightUltrasoundSensor.getAverageVoltage();
	}

	public double readRightUltrasoundInches() {
		return rightMinDistance + (readRightUltrasoundVolts() - rightMinVolts)
				* ((rightMaxDistance - rightMinDistance) / (rightMaxVolts - rightMinVolts));
	}

	public double readAverageUltrasoundInches() {
		return (readRightUltrasoundInches() + readLeftUltrasoundInches()) / 2;
	}

	public double readUltrasoundAngle() {
		return Math
				.toDegrees(Math.atan((readRightUltrasoundInches() - readLeftUltrasoundInches()) / ultrasoundSpacing));
	}

	// the else code resets the sensor in event of a zero reading
	// lock up
	public void updateStatus() {
		SmartDashboard.putNumber("Left Ultrasound Volts", Math.round(readLeftUltrasoundVolts() * 100.) / 100.);
		SmartDashboard.putNumber("Right Ultrasound Volts", Math.round(readRightUltrasoundVolts() * 100.) / 100.);
		SmartDashboard.putNumber("Left Ultrasound Inches", Math.round(readLeftUltrasoundInches() * 10.) / 10.);
		SmartDashboard.putNumber("Right Ultrasound Inches", Math.round(readRightUltrasoundInches() * 10.) / 10.);
		SmartDashboard.putNumber("Ultrasound Angle", Math.round(readUltrasoundAngle() * 10.) / 10.);

	}

}
