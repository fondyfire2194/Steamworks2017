package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.commands.HookVisionOnOff;
import org.usfirst.frc.team2194.robot.commands.UpdateAllPrefsValues;
import org.usfirst.frc.team2194.robot.commands.Gyro.SetCurrentAngleAsTarget;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class JoystickToTarget extends CommandGroup {

	public JoystickToTarget() {
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
		addSequential(new UpdateAllPrefsValues());
		addSequential(new JoystickVisionComp());
		new SetCurrentAngleAsTarget();
		addSequential(new DriveCompControl(true));
		addSequential(new HookVisionOnOff(false));

	}

}
