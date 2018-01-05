package org.usfirst.frc.team2194.robot.commands.Autonomous;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.commands.BoilerVisionOnOff;
import org.usfirst.frc.team2194.robot.commands.HookVisionOnOff;
import org.usfirst.frc.team2194.robot.commands.LiftCamera;
import org.usfirst.frc.team2194.robot.commands.LowerCamera;
import org.usfirst.frc.team2194.robot.commands.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.StartShootersAndFeeders;
import org.usfirst.frc.team2194.robot.commands.TimeDelay;
import org.usfirst.frc.team2194.robot.commands.UpdateAllPrefsValues;
import org.usfirst.frc.team2194.robot.commands.GearControl.OuttakeGear;
import org.usfirst.frc.team2194.robot.commands.Gyro.ResetGyro;
import org.usfirst.frc.team2194.robot.commands.Gyro.SetTargetAngle;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotDriveToTarget;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotOrient;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotOrient.MoveModeRot;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotPosition;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotPosition.CompMode;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.RobotPosition.MoveMode;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Red_BoilerSideGearShoot extends CommandGroup {
	// gear values
	double driveSpeed;
	double driveSpeedOrient;
	double dist2Turn = 67.2;// 68.5;
	double targetPoint = 64.6;// 58.7;
	double turnAngle = 300;
	// shoot values
	double reverseDistance = 21;
	double shootTurnAngle = 143.1;//144.8;
	double shootForwardPoint = 34.3;//34.2;

	public Red_BoilerSideGearShoot() {

		requires(Robot.driveTrain);
		requires(Robot.leftSideDrive);
		requires(Robot.rightSideDrive);
		driveSpeed = Robot.prefs.getDouble("DriveSpeed", .9);
		driveSpeedOrient = Robot.prefs.getDouble("DriveSpeedOrient", .55);

		addSequential(new UpdateAllPrefsValues());

		addSequential(new ResetGyro());
		addParallel(new LowerCamera());

		addSequential(new ResetEncoders());

		addSequential(new SetTargetAngle(0));

		addSequential(new RobotPosition(dist2Turn, driveSpeed, MoveMode.absolute, CompMode.gyro, false, 2, true, 5));
		addSequential(new TimeDelay(.1));

		addSequential(new RobotOrient(turnAngle, driveSpeedOrient, MoveModeRot.absolute, 2, 1.25));
		addSequential(new HookVisionOnOff(true));
		addSequential(new SetTargetAngle(turnAngle));
		addSequential(new ResetEncoders());

		addSequential(new RobotDriveToTarget(targetPoint, driveSpeed, false, 6));
		addSequential(new HookVisionOnOff(false));

		addSequential(new OuttakeGear());
//		addSequential(new ResetEncoders());
//		addSequential(new TimeDelay(.1));
		// **********************************************************************************************************8
		addSequential(new RobotPosition(targetPoint-reverseDistance, driveSpeed, MoveMode.absolute, CompMode.gyro, false, 2, true, 3));
		addSequential(new TimeDelay(.1));

		addParallel(new LiftCamera());
		addSequential(new RobotOrient(shootTurnAngle, driveSpeedOrient, MoveModeRot.absolute, 2, 2));
		addSequential(new BoilerVisionOnOff(true));

		addSequential(new SetTargetAngle(shootTurnAngle));
		addSequential(new ResetEncoders());
		addSequential(new TimeDelay(.1));

		addParallel(new StartShootersAndFeeders(Robot.prefs.getDouble("ShootRPM", 2900)));
		addSequential(new RobotDriveToTarget(shootForwardPoint, driveSpeed, false, 4));
		addParallel(new LowerCamera());
		addSequential(new BoilerVisionOnOff(false));

	}
}
