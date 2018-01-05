package org.usfirst.frc.team2194.robot;

import org.usfirst.frc.team2194.robot.commands.HighGear;
import org.usfirst.frc.team2194.robot.commands.HookBoilerCameraShowOnOff;
import org.usfirst.frc.team2194.robot.commands.LiftCameraTurnOnLeds;
import org.usfirst.frc.team2194.robot.commands.LowGear;
import org.usfirst.frc.team2194.robot.commands.LowerCameraTurnOffLeds;
import org.usfirst.frc.team2194.robot.commands.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.StartShootersAndFeeders;
import org.usfirst.frc.team2194.robot.commands.ToggleBoilerVision;
import org.usfirst.frc.team2194.robot.commands.ToggleGearVision;
import org.usfirst.frc.team2194.robot.commands.ToggleHookVision;
import org.usfirst.frc.team2194.robot.commands.ToggleTargeting;
import org.usfirst.frc.team2194.robot.commands.Climber.StartAgitator;
import org.usfirst.frc.team2194.robot.commands.Climber.StopAgitator;
import org.usfirst.frc.team2194.robot.commands.Feeder.ReverseFeeders;
import org.usfirst.frc.team2194.robot.commands.Feeder.StartFirstFeeder;
import org.usfirst.frc.team2194.robot.commands.Feeder.StartSecondFeeder;
import org.usfirst.frc.team2194.robot.commands.Feeder.StopFeeders;
import org.usfirst.frc.team2194.robot.commands.GearControl.GripGear;
import org.usfirst.frc.team2194.robot.commands.GearControl.IntakeGear;
import org.usfirst.frc.team2194.robot.commands.GearControl.LiftGear;
import org.usfirst.frc.team2194.robot.commands.GearControl.LowerGear;
import org.usfirst.frc.team2194.robot.commands.GearControl.OuttakeGear;
import org.usfirst.frc.team2194.robot.commands.GearControl.PrepIntake;
import org.usfirst.frc.team2194.robot.commands.GearControl.PushGear;
import org.usfirst.frc.team2194.robot.commands.GearControl.ReleaseGear2;
import org.usfirst.frc.team2194.robot.commands.GearControl.ReleaseGearSpecial;
import org.usfirst.frc.team2194.robot.commands.GearControl.RetractGear;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.JoystickGearPickupComp;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.JoystickNoComp;
import org.usfirst.frc.team2194.robot.commands.RobotMotion.JoystickToTarget;
import org.usfirst.frc.team2194.robot.commands.Shooter.DecBottomShooterRPM;
import org.usfirst.frc.team2194.robot.commands.Shooter.DecTopShooterRPM;
import org.usfirst.frc.team2194.robot.commands.Shooter.IncBottomShooterRPM;
import org.usfirst.frc.team2194.robot.commands.Shooter.IncTopShooterRPM;
import org.usfirst.frc.team2194.robot.commands.Shooter.StartBottomShooterVbus;
import org.usfirst.frc.team2194.robot.commands.Shooter.StartTopShooterRPM;
import org.usfirst.frc.team2194.robot.commands.Shooter.StopShooters;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.InternalButton;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());

	private boolean useSecondGamepad = true;

	public Joystick joystick1;
	public Joystick gamepad;
	public Joystick gamepad2;
	// **********************************************************
	// SmartDashboard

	public InternalButton toggleGearVisionSD;
	public InternalButton toggleHookVisionSD;
	public InternalButton toggleBoilerVisionSD;
	public InternalButton toggleTargetingSD;
	
	public InternalButton resetEncodersSD;
	public InternalButton readDataNext;
	public InternalButton readDataLast;
	public InternalButton captureData;
	public InternalButton writeFileData;

	// **********************************************************
	// Main Driver Joystick //
	// **********************************************************
	// Co-driver Gamepad

	public JoystickButton hiGear;
	public JoystickButton noCompJoystick;
	public JoystickButton visionCompJoystick;
	public JoystickButton grabGear;
	public JoystickButton gearPickupCompJoystick;
	public JoystickButton resetGyro;

	public static JoystickButton resetEncoders;

	public static JoystickButton agitate;
	public static JoystickButton reverseAgitator;

	public static JoystickButton startTopShooterBaseRPM;
	public static JoystickButton startBottomShooterBaseRPM;
	public static JoystickButton startShootersAndFeeders;
	public static JoystickButton startFeeders;

	public static JoystickButton incTopShooterSpeed;
	public static JoystickButton decTopShooterSpeed;
	public static JoystickButton stopShooters;

	public static JoystickButton prepIntake;
	public static JoystickButton intakeGear;
	public static JoystickButton outtakeGear;

	public static JoystickButton startFirstFeeder;
	public static JoystickButton startSecondFeeder;
	public static JoystickButton stopFeeders;
	public static JoystickButton reverseFeeders;

	// **********************************************************
	// Setup Gamepad 2

	public static JoystickButton gripGear;
	public static JoystickButton liftGear;
	public static JoystickButton lowerGear;
	public static JoystickButton pushGear;
	public static JoystickButton releaseGear;
	public static JoystickButton retractGear;

	public static JoystickButton startTopShooterVbus;
	public static JoystickButton startBottomShooterVbus;
	public static JoystickButton stopShooters2;
	public static JoystickButton raiseCamera;

	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	public OI()

	{
		// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

		// ************************************************************
		// SmartDashboard Buttons
		toggleGearVisionSD = new InternalButton();
		toggleHookVisionSD = new InternalButton();
		toggleBoilerVisionSD = new InternalButton();
		resetEncodersSD = new InternalButton();
		toggleTargetingSD = new InternalButton();
		
		readDataNext = new InternalButton();
		readDataLast = new InternalButton();
		captureData = new InternalButton();
		writeFileData = new InternalButton();

		SmartDashboard.putData("Toggle Gear Vision", toggleGearVisionSD);
		SmartDashboard.putData("Toggle Hook Vision", toggleHookVisionSD);
		SmartDashboard.putData("Toggle Boiler Vision", toggleBoilerVisionSD);
		SmartDashboard.putData("Toggle Targeting", toggleTargetingSD);
		
		SmartDashboard.putData("Reset Encoders", resetEncodersSD);

		toggleHookVisionSD.whenPressed(new ToggleHookVision());
		toggleGearVisionSD.whenPressed(new ToggleGearVision());
		toggleBoilerVisionSD.whenPressed(new ToggleBoilerVision());
		resetEncodersSD.whenPressed(new ResetEncoders());
		toggleTargetingSD.whenPressed(new ToggleTargeting());

		// ******************************************************************
		// Joystick 1 Main Driver
		// ******************************************

		joystick1 = new Joystick(1);
		gamepad = new Joystick(0);
		// *************************************************************

		hiGear = new JoystickButton(joystick1, 1);
		hiGear.whileHeld(new HighGear());
		hiGear.whenReleased(new LowGear());

		noCompJoystick = new JoystickButton(joystick1, 4);
		noCompJoystick.whenPressed(new JoystickNoComp());
		
		visionCompJoystick = new JoystickButton(joystick1, 8);
		visionCompJoystick.whenPressed(new JoystickToTarget());
		visionCompJoystick.whenReleased(new JoystickNoComp());

		grabGear = new JoystickButton(joystick1, 6);
		grabGear.whenPressed(new GripGear());
		grabGear.whenReleased(new ReleaseGearSpecial());

		gearPickupCompJoystick = new JoystickButton(joystick1, 7);
		gearPickupCompJoystick.whenPressed(new JoystickGearPickupComp());
		gearPickupCompJoystick.whenReleased(new JoystickNoComp());
		
		prepIntake = new JoystickButton(joystick1, 11);
		prepIntake.whenPressed(new PrepIntake());

		intakeGear = new JoystickButton(joystick1, 3);
		intakeGear.whenPressed(new IntakeGear());

		outtakeGear = new JoystickButton(joystick1, 2);
		outtakeGear.whenPressed(new OuttakeGear());


		// *************************************************************

		// Co driver gamepad

		startFeeders = new JoystickButton(gamepad, 1);
		startFeeders.whenPressed(new StartFirstFeeder());
		startFeeders.whenPressed(new StartSecondFeeder());
		// startFeeders.whenPressed(new StartAgitator(1));
		
		decTopShooterSpeed = new JoystickButton(gamepad, 2);
		decTopShooterSpeed.whenPressed(new DecTopShooterRPM());
		decTopShooterSpeed.whenPressed(new DecBottomShooterRPM());

		stopFeeders = new JoystickButton(gamepad, 3);
		stopFeeders.whenPressed(new StopFeeders());
		stopFeeders.whenPressed(new StopAgitator());

		incTopShooterSpeed = new JoystickButton(gamepad, 4);
		incTopShooterSpeed.whenPressed(new IncTopShooterRPM());
		incTopShooterSpeed.whenPressed(new IncBottomShooterRPM());
		
		startShootersAndFeeders = new JoystickButton(gamepad, 6);
		startShootersAndFeeders.whenPressed(new StartShootersAndFeeders(Robot.prefs.getDouble("ShootRPM", 3000)));

		raiseCamera = new JoystickButton(gamepad, 7);

		raiseCamera.whenPressed(new LiftCameraTurnOnLeds());
		raiseCamera.whenPressed(new HookBoilerCameraShowOnOff(true));
				
		raiseCamera.whenReleased(new LowerCameraTurnOffLeds());
		raiseCamera.whenReleased(new HookBoilerCameraShowOnOff(false));

		stopShooters = new JoystickButton(gamepad, 8);
		stopShooters.whenPressed(new StopFeeders());
		stopShooters.whenPressed(new StopShooters());
		stopShooters.whenPressed(new StopAgitator());

		reverseFeeders = new JoystickButton(gamepad, 10);
		reverseFeeders.whileHeld(new ReverseFeeders());
		reverseFeeders.whileHeld(new StartAgitator(-1));
		reverseFeeders.whenReleased(new StopFeeders());
		reverseFeeders.whenReleased(new StopAgitator());
		
		
		// *************************************************************
		// *************************************************************

		// ******************************************
		// Test and Setup Only Gamepad
		if (useSecondGamepad) {

			gamepad2 = new Joystick(2);

			 startTopShooterVbus = new JoystickButton(gamepad2, 1);
			 startTopShooterVbus.whenPressed(new StartTopShooterRPM(3000));
			
			 startBottomShooterVbus = new JoystickButton(gamepad2, 2);
			 startBottomShooterVbus.whenPressed(new StartBottomShooterVbus());
			 startBottomShooterVbus.whenPressed(new StartAgitator(1));


			gripGear = new JoystickButton(gamepad2, 3);
			gripGear.whenPressed(new GripGear());

			releaseGear = new JoystickButton(gamepad2, 4);
			releaseGear.whenPressed(new ReleaseGear2());

			liftGear = new JoystickButton(gamepad2, 5);
			liftGear.whenPressed(new LiftGear());

			lowerGear = new JoystickButton(gamepad2, 6);
			lowerGear.whenPressed(new LowerGear());

			pushGear = new JoystickButton(gamepad2, 7);
			pushGear.whenPressed(new PushGear());

			retractGear = new JoystickButton(gamepad2, 8);
			retractGear.whenPressed(new RetractGear());

			stopShooters2 = new JoystickButton(gamepad2, 10);
			stopShooters2.whenPressed(new StopShooters());

		}

	}

	public Joystick getJoystick1() {
		return joystick1;
	}

	public Joystick getgamepad2() {
		return gamepad2;
	}

	public Joystick getgamepad() {
		return gamepad;
	}

}
