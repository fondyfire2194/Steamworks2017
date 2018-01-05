package org.usfirst.frc.team2194.robot.commands.GearControl;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.commands.HasGear;
import org.usfirst.frc.team2194.robot.commands.SelectGearCamera;
import org.usfirst.frc.team2194.robot.commands.TimeDelay;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class PrepIntake extends CommandGroup {

	public PrepIntake() {
		// Add Comma
		// e.g. addSequential(new Command1());
		// addSequential(new Command2());
		// these will run in order.
		requires(Robot.pistonControl);
//		addParallel(new PushGear());
		// addSequential(new TimeDelay(.15));
		addParallel(new LowerGear());
		addSequential(new TimeDelay(.1));
		addParallel(new ReleaseGear2());
		addParallel(new SelectGearCamera());
		addSequential(new RetractGear());
		addParallel(new HasGear(false));
//		Robot.hasGear = false;

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
	}
}
