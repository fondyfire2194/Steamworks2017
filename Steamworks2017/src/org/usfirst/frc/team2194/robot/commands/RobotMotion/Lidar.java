package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Lidar extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public double readLidarSensor() {
		Robot.lidarPulseWidth = RobotMap.pwmWidth.getPeriod();// get pwm high
																// time
		if (Robot.lidarPulseWidth != 0) {
			Robot.lidarPulseWidth = Robot.lidarPulseWidth * 10000;
			return Robot.lidarPulseWidth * 25.4;
		}
		// 10 microseconds is 1 cm. Value returned is in seconds so this is mm
		else {
			RobotMap.lidarPowerEnable.set(false);
			Timer.delay(.005);
			RobotMap.lidarPowerEnable.set(true);
			return 0;
		}
	}

	// the else code resets the sensor in event of a zero reading
	// lock up
	public void updateStatus() {
		SmartDashboard.putNumber("Lidar Reading", readLidarSensor());
		return;
	}
}
