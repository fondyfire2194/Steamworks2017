package org.usfirst.frc.team2194.robot.commands.GearControl;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.commands.HasGear;
import org.usfirst.frc.team2194.robot.commands.SelectGearCamera;
import org.usfirst.frc.team2194.robot.commands.TimeDelay;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.JoystickNoComp;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class IntakeGear extends CommandGroup {

	public IntakeGear() {
		// Add Commands here:
		// e.g. addSequential(new Command1());
		// addSequential(new Command2());
		// these will run in order.
		requires(Robot.pistonControl);

		addParallel(new GripGear());
		addParallel(new PushGear());
		// addSequential(new TimeDelay(.05));
		addParallel(new LiftGear());
		addSequential(new TimeDelay(.1));
		addParallel(new RetractGear());
		addParallel(new JoystickNoComp());
		addParallel(new SelectGearCamera());
		addParallel(new HasGear(true));
//		Robot.hasGear = true;

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
