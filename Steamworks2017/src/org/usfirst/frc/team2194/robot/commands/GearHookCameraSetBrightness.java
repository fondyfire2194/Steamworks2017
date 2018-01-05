package org.usfirst.frc.team2194.robot.commands;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class GearHookCameraSetBrightness extends InstantCommand {
private int mySetting;
    public GearHookCameraSetBrightness(int setting) {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        mySetting = setting;
    }

    // Called once when the command executes
    protected void initialize() {
		Robot.gearHookCamera.setBrightness(mySetting);

    }

}
