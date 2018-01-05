package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.commands.PistonControlDefault;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class PistonControl extends Subsystem {
	public boolean gearRetracted;

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new PistonControlDefault());
	}

	public void gearPush() {
		RobotMap.gearPush1.set(DoubleSolenoid.Value.kForward);
		gearRetracted = false;
	}

	public void retractGear() {
		RobotMap.gearPush1.set(DoubleSolenoid.Value.kReverse);
		gearRetracted = true;
	}

	public void gripGear() {
		RobotMap.gearGrip.set(DoubleSolenoid.Value.kReverse);

	}

	public void releaseGear() {
		RobotMap.gearGrip.set(DoubleSolenoid.Value.kForward);
	}

	public void liftGear() {
		RobotMap.gearLift.set(DoubleSolenoid.Value.kReverse);

	}

	public void lowerGear() {
		RobotMap.gearLift.set(DoubleSolenoid.Value.kForward);
	}
	
	public void liftCamera() {
		RobotMap.tiltCamera.set(DoubleSolenoid.Value.kReverse);

	}

	public void lowerCamera() {
		RobotMap.tiltCamera.set(DoubleSolenoid.Value.kForward);
	}
	
	public void tiltCameraOff(){
		RobotMap.tiltCamera.set(DoubleSolenoid.Value.kOff);
	}

	public boolean valueToBool() {
		if (RobotMap.driveShifter.get() == DoubleSolenoid.Value.kForward) {
			return true;
		} else
			return false;
	}

	public void setupPistons() {
		// liftGear();
		// retractGear();
		// gripGear();
	}

	public void updateStatus() {
		// SmartDashboard.putBoolean("Super Shifter Value", valueToBool() );
	}

}
