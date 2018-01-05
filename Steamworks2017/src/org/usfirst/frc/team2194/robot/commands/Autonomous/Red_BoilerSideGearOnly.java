package org.usfirst.frc.team2194.robot.commands.Autonomous;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.commands.HookVisionOnOff;
import org.usfirst.frc.team2194.robot.commands.ResetEncoders;
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
public class Red_BoilerSideGearOnly extends CommandGroup {

	double driveSpeed;
	double driveSpeedOrient;
	public Red_BoilerSideGearOnly() {
		// Add Commands here:
		// e.g. addSequential(new Command1());
		// addSequential(new Command2());
		// these will run in order.

		// To run multiple commands at the same time,
		// use addParallel()
		// e.g. addParallel(new Command1());
		// addSequential(new Command2());
		// Command1 and Command2 will run in parallel.

		// A command group will require all of the subsystems that each member
		// would require.
		// e.g. if Command1 requires chassis, and Command2 requires arm,
		// a CommandGroup containing them would require both the chassis and the
		// arm.
		double dist2Turn = 67.2;
		double targetPoint = 64.6;
		double turnAngle = 300;

		requires(Robot.driveTrain);
		requires(Robot.leftSideDrive);
		requires(Robot.rightSideDrive);
		driveSpeed = Robot.prefs.getDouble("DriveSpeed", .9);
		driveSpeedOrient = Robot.prefs.getDouble("DriveSpeedOrient", .55);

		addSequential(new UpdateAllPrefsValues());

		addSequential(new ResetGyro());

		addSequential(new ResetEncoders());

		addSequential(new SetTargetAngle(0));

		addSequential(new RobotPosition(dist2Turn, driveSpeed, MoveMode.absolute, CompMode.gyro, false, 1, true, 5));

		addSequential(new RobotOrient(turnAngle, driveSpeedOrient, MoveModeRot.absolute, 2, 1.25));
		addSequential(new ResetEncoders());
		addSequential(new HookVisionOnOff(true));
		addSequential(new SetTargetAngle(turnAngle));

		addSequential(new RobotDriveToTarget(targetPoint, driveSpeed, false, 6));
		addSequential(new HookVisionOnOff(false));

		addSequential(new OuttakeGear());
//		addSequential(new TimeDelay(1));
		addSequential(new RobotPosition(targetPoint-35, driveSpeed, MoveMode.absolute, CompMode.gyro, false, .75, true, 6));
		
		addSequential(new RobotOrient(0, driveSpeedOrient, MoveModeRot.absolute, 2, 1.25));
		addSequential(new SetTargetAngle(0));
		addParallel(new ResetEncoders());
		addSequential(new RobotPosition(260, driveSpeed, MoveMode.absolute, CompMode.gyro, false, .75, true, 6));

	}
}
