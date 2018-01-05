package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class GearVision extends Subsystem {
	public int gearInDistance = 180; // !!!!!!!!!!!!!!!! CHANGE !!!!!!!!!!!!!

	public boolean getVisionOn() { // + comp + right side
		return Robot.gearVisionTurnedOn;
	}

	public int getX0() {
		return Robot.gearX0;
	}

	public int getY0() {
		return Robot.gearY0;
	}

	public int getHeight0() {
		return Robot.gearHeight0;
	}

	public int getWidth0() {
		return Robot.gearWidth0;
	}

	public double getCenterX0() {
		return Robot.gearX0 + (Robot.gearWidth0 / 2);
	}

	public int getCenterY0() {
		return Robot.gearY0 + (Robot.gearHeight0 / 2);
	}

	public int Y0() {
		return Robot.gearY0 + (Robot.gearHeight0 / 2);
	}

	public double getXError() {
		return getCenterX0() - (Robot.IMG_WIDTH / 2);
	}

	public int getYError() {
		return Y0() - gearInDistance;
	}

	public boolean isGearInThresh() {
		return (getCenterY0() >= gearInDistance);

	}

	public boolean isGearInCenter() {
		return (-10 < getXError() && getXError() < 10);
	}

	public double getHoriz() {
		return (Robot.IMG_WIDTH / 2) - getCenterX0();
	}

	public double getVert() {
		return (Robot.IMG_HEIGHT - getCenterY0());
	}

	public double getGearTargetAngle() {
		return -(Math.atan2(getHoriz(), getVert()) * (180 / Math.PI));
	}

	public boolean getGearVisionOn() {
		return Robot.gearVisionTurnedOn;
	}

	public void updateStatus() {
//		SmartDashboard.putBoolean("Gear Vision On", getGearVisionOn());
		SmartDashboard.putNumber("Gear Thread", Robot.gearThreadCounter);

		// SmartDashboard.putBoolean("Is Gear In?", isGearInThresh());
		// SmartDashboard.putBoolean("Is Gear Center?", isGearInCenter());
		// SmartDashboard.putNumber("X Error", getXError());
		// SmartDashboard.putNumber("Y Error", getYError());
		// SmartDashboard.putNumber("X0", getX0());
		// SmartDashboard.putNumber("Y0", getY0());
		// SmartDashboard.putNumber("Height0", getHeight0());
		// SmartDashboard.putNumber("Width0", getWidth0());
		// SmartDashboard.putNumber("Gear Image", Robot.gearNumberImages);
		// SmartDashboard.putNumber("Gear Center Y", getCenterY0());
		// SmartDashboard.putNumber("Gear Center X", getCenterX0());
		// SmartDashboard.putNumber("Gear Target Angle", getGearTargetAngle());
		// SmartDashboard.putNumber("Vert Comp", getVert());
		// SmartDashboard.putNumber("Horiz Comp", getHoriz());

	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
