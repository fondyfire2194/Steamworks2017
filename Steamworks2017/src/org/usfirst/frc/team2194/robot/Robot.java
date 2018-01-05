
package org.usfirst.frc.team2194.robot;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team2194.robot.commands.HookVisionOnOff;
import org.usfirst.frc.team2194.robot.commands.LowerCamera;
import org.usfirst.frc.team2194.robot.commands.SelectGearCamera;
import org.usfirst.frc.team2194.robot.commands.UpdateAllPrefsValues;
import org.usfirst.frc.team2194.robot.commands.Autonomous.BlueCenterGearOnly;
import org.usfirst.frc.team2194.robot.commands.Autonomous.BlueCenterGearShoot;
import org.usfirst.frc.team2194.robot.commands.Autonomous.Blue_BoilerSideGearOnly;
import org.usfirst.frc.team2194.robot.commands.Autonomous.Blue_BoilerSideGearShoot;
import org.usfirst.frc.team2194.robot.commands.Autonomous.Blue_LoadSideGearOnly;
import org.usfirst.frc.team2194.robot.commands.Autonomous.CrossLine;
import org.usfirst.frc.team2194.robot.commands.Autonomous.DoNothing;
import org.usfirst.frc.team2194.robot.commands.Autonomous.RedCenterGearShoot;
import org.usfirst.frc.team2194.robot.commands.Autonomous.Red_BoilerSideGearOnly;
import org.usfirst.frc.team2194.robot.commands.Autonomous.Red_BoilerSideGearShoot;
import org.usfirst.frc.team2194.robot.commands.Autonomous.Red_CenterGearOnly;
import org.usfirst.frc.team2194.robot.commands.Autonomous.Red_LoadSideGearOnly;
import org.usfirst.frc.team2194.robot.commands.Climber.StopAgitator;
import org.usfirst.frc.team2194.robot.commands.Feeder.StopFeeders;
import org.usfirst.frc.team2194.robot.commands.GearControl.GripGear;
import org.usfirst.frc.team2194.robot.commands.GearControl.LiftGear;
import org.usfirst.frc.team2194.robot.commands.GearControl.RetractGear;
import org.usfirst.frc.team2194.robot.commands.Shooter.StopShooters;
import org.usfirst.frc.team2194.robot.subsystems.Agitator;
import org.usfirst.frc.team2194.robot.subsystems.AirCompressor;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrain;
import org.usfirst.frc.team2194.robot.subsystems.Feeder;
import org.usfirst.frc.team2194.robot.subsystems.GearVision;
import org.usfirst.frc.team2194.robot.subsystems.GyroRotate;
import org.usfirst.frc.team2194.robot.subsystems.LeftSideDrive;
import org.usfirst.frc.team2194.robot.subsystems.LeftUltrasoundDrive;
import org.usfirst.frc.team2194.robot.subsystems.Lidar;
import org.usfirst.frc.team2194.robot.subsystems.PistonControl;
import org.usfirst.frc.team2194.robot.subsystems.PowerPanel;
import org.usfirst.frc.team2194.robot.subsystems.RightSideDrive;
import org.usfirst.frc.team2194.robot.subsystems.RightUltrasoundDrive;
import org.usfirst.frc.team2194.robot.subsystems.Shooter;
import org.usfirst.frc.team2194.robot.subsystems.Ultrasound;
import org.usfirst.frc.team2194.robot.subsystems.VisionData;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static WriteFile writeFile;
	public static GyroRotate gyroRotate;
	public static OI oi;
	public static HookPipeline hookPipeline;
	public static BoilerPipeline boilerPipeline;
	public static GearPipeline gearPipeline;
	public static MovingAverage leftUSAvge;
	public static MovingAverage rightUSAvge;

	public static VisionData visionData;
	public static UsbCamera gearHookCamera;
	UsbCamera gearPickupCamera;
	public static AirCompressor airCompressor;
	public static PistonControl pistonControl;
	public static Shooter shooter;
	public static Feeder feeder;
	public static Agitator agitator;
	public static GearVision gearVision;

	public static boolean isInHighGear;

	public static final int IMG_WIDTH = 320;
	public static final int IMG_HEIGHT = 240;

	private VisionThread hookVisionThread;
	private VisionThread boilerVisionThread;
	private Thread visionThreadAddTargeting;
	private VisionThread gearVisionThread;

	public static PowerPanel powerPanel;

	public static AHRS imu;

	public static Preferences prefs;

	public static double startVisionDistance = 110;
	public static double latchAngleCompPixelsDistance = 25;
	public static int heightPxChangeover = 48;
	public static double releaseGearDistance = 17;
	public static int correctionPixelsInLimit = 5;
	public static int anglePixelIncrement = 5;
	public static double remainingDistanceToHook;

	public static double leftUltrasoundAverage;
	public static double rightUltrasoundAverage;

	Rect r0;

	public static double ledPowerHookCam = .9;

	private static int hookImagesAllowed = 6;
	Rect[] r;
	public static int[] x;
	public static int[] y;
	public static int[] height;
	public static int[] width;
	public static int[] xv;
	public static int[] yv;
	public static int[] heightv;
	public static int[] widthv;

	public static int[] ar;
	public static int minHookAr = 10;// hook vision strips aspect ratio = 200/5
	public static int maxHookAr = 200;
	public static int minBoilerWidth = 10;//
	public static int maxBoilerWidth = 200;//
	//

	public static int validar;
	public static int x0;
	public static int y0;
	public static int gVHeight0;
	public static int width0;
	public static int x1;
	public static int y1;
	public static int height1;
	public static int width1;

	public static int hookNumberImages;// amt of images KW
	public static double fullTargetAreaWidth = 10.5;// inches
	public static double visionTargetsWidth = 2.0;// inches
	public static double distance;

	private final Object imgLock = new Object();
	private final Object gearImgLock = new Object();

	// 4:3 Aspect Ratio
	// DFOV 60 HFOV 53.4 VFOV 31.6
	// DFOV 70 HFOV 62.8 VFOV 37.9
	// Interpolate for other values

	// Microsoft HD5000 4:3 aspect ratio
	// private final double dFOV = 66 * Math.PI / 180;
	// public static double hFOV = 55 * Math.PI / 180;
	// private final double vFOV = 43 * Math.PI / 180;

	// Logitech C250 16:9 aspect ratio
	private final double dFOV = 60 * Math.PI / 180;
	public static double hFOV = 43 * Math.PI / 180;
	private final double vFOV = 38.2 * Math.PI / 180;

	private final double targetWidth = 10.5;// inches

	public static double maxEncoderCountsPerSecond;
	public static LeftSideDrive leftSideDrive;
	public static RightSideDrive rightSideDrive;
	public static LeftUltrasoundDrive leftUltrasoundDrive;
	public static RightUltrasoundDrive rightUltrasoundDrive;

	public static Lidar lidarSensor;
	public static double lidarPulseWidth;

	public static DriveTrain driveTrain;

	public static Ultrasound ultraSound;

	public static boolean isOrienting;
	public static boolean isPositioning;

	public static double remainingMoveDistanceInches;

	public static boolean motionSeen;

	public static double positionEndpoint20 = 20;
	public static double positionEndpoint100 = 100;

	public static double positionRate35 = .35;
	public static double positionRate50 = .5;

	public static double angle = 45;
	public static double turnRate = .5;
	public static boolean noCompJoystick;
	public static boolean visionCompJoystick;
	public static boolean gyroCompJoystick;
	public static boolean gearPickupJoystick;

	public static boolean hookVisionTurnedOn;
	public static int hookThreadCounter;
	private static int noImages;
	public static int angleCorrectionPixels;
	public static int degreePixels;
	public static boolean isUltrasoundPositioning;
	public static double activeMotionComp;
	public static boolean useVisionDistances;
	public static boolean stopMotion;

	public static Scalar yellow = new Scalar(60, 100, 100, 0);
	public static Scalar red = new Scalar(0, 0, 255, 0);
	public static Scalar green = new Scalar(0, 255, 0, 0);
	public static Scalar blue = new Scalar(255, 0, 0, 0);
	public int boxAddition = 4;

	Command autonomousCommand;

	SendableChooser<Command> autoChooser;
	private Thread gearThreadAddTargeting;
	private boolean runSmartDashboard;
	private double smartDashboardTimer;
	public static int boilerThreadCounter;
	public static boolean boilerVisionTurnedOn;
	public static boolean hookVisionTargetingOn = false;
	public static boolean gearVisionTargetingOn = false;
	public static boolean hookVisionStreamOn = false;
	public static boolean hookBoilerCameraShow;
	public static boolean showTargeting;

	public static double testAvge;

	public static boolean viewGearCamera;
	public static boolean viewHookCamera = true;

	public static int gearThreadCounter;
	public static boolean gearVisionTurnedOn;
	public static int gearNumberImages;
	public static int gearHeight0;
	public static int gearWidth0;
	public static int gearX0;
	public static int gearY0;

	public static boolean gearCompRanOnce = false;

	public static boolean hasGear;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */

	@Override
	public void robotInit() {
		RobotMap.init();
		pistonControl = new PistonControl();// leave these here must not be
											// after oi or code will crash
		powerPanel = new PowerPanel();
		airCompressor = new AirCompressor();

		prefs = Preferences.getInstance();
		oi = new OI();

		// prefs.putDouble("Top Shooter Vbus", 1);
		// prefs.putDouble("Bottom Shooter Vbus", 1);

		// Robot.prefs.putDouble("Left Position Kp", .003);
		// Robot.prefs.putDouble("Left Position Ki", 0);
		// Robot.prefs.putDouble("Right Position Kp", .003);
		// Robot.prefs.putDouble("Right Position Ki", 0);
		// Robot.prefs.putDouble("Left Ultrasound Kp", .003);
		// Robot.prefs.putDouble("Left Ultrasound Ki", 0);
		// Robot.prefs.putDouble("Right Ultrasound Kp", .003);
		// Robot.prefs.putDouble("Right Ultrasound Ki", 0);
		// Robot.prefs.putDouble("Drive Straight Gyro Kp", .05);
		// Robot.prefs.putDouble("Gyro Position Kp", .01);
		// Robot.prefs.putDouble("Gyro Position Ki", 0);
		// Robot.prefs.putDouble("Vision Kp", .002);
		// Robot.prefs.putDouble("DriveSpeed", .8);
		// Robot.prefs.putDouble("DriveSpeedOrient", .55);
		// Robot.prefs.putDouble("HookSldnDist", 20);

		hookPipeline = new HookPipeline();
		boilerPipeline = new BoilerPipeline();
		gearPipeline = new GearPipeline();

		leftSideDrive = new LeftSideDrive();
		rightSideDrive = new RightSideDrive();
		gyroRotate = new GyroRotate();
		ultraSound = new Ultrasound();
		leftUltrasoundDrive = new LeftUltrasoundDrive();// ultrasound used
		rightUltrasoundDrive = new RightUltrasoundDrive();// ultrasound used
		driveTrain = new DriveTrain();
		visionData = new VisionData();
		visionData.init();// set vision capture values
		// lidarSensor = new Lidar();
		shooter = new Shooter();
		feeder = new Feeder();
		agitator = new Agitator();
		gearVision = new GearVision();

		leftUSAvge = new MovingAverage(3);
		rightUSAvge = new MovingAverage(3);

		(new UpdateAllPrefsValues()).start();

		Scheduler.getInstance().run();

		autoChooser = new SendableChooser<Command>();

		SmartDashboard.putData("Chooser", autoChooser);

		autoChooser.addDefault("BLUE Center Gear Only", new BlueCenterGearOnly());// iz
		autoChooser.addObject("BLUE Center Gear Shoot", new BlueCenterGearShoot());// iz
		autoChooser.addObject("BLUE Boiler Side Gear Only", new Blue_BoilerSideGearOnly());
		autoChooser.addObject("BLUE Boiler Side Gear And Shoot", new Blue_BoilerSideGearShoot());
		autoChooser.addObject("BLUE Load Side Gear Only", new Blue_LoadSideGearOnly());

		autoChooser.addObject("RED Center Gear Only", new Red_CenterGearOnly());// iz
		autoChooser.addObject("RED Center Gear Shoot", new RedCenterGearShoot());// iz
		autoChooser.addObject("RED Boiler Side Gear Only", new Red_BoilerSideGearOnly());
		autoChooser.addObject("RED Boiler Side Gear Shoot", new Red_BoilerSideGearShoot());
		autoChooser.addObject("RED Load Side Gear Only", new Red_LoadSideGearOnly());

		autoChooser.addObject("Do Nothing", new DoNothing());
		autoChooser.addObject("Cross Line", new CrossLine());

		SmartDashboard.putData(Scheduler.getInstance());

		Robot.shooter.setupShooterMotors();

		// *******************************************************************************************

		try {
			imu = new AHRS(SPI.Port.kMXP);
			// imu = new AHRS(SerialPort.Port.kUSB1);
			imu.setPIDSourceType(PIDSourceType.kDisplacement);

		} catch (Exception ex) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}
		if (imu != null) {
			LiveWindow.addSensor("IMU", "Gyro", imu);
		}
		Timer.delay(3);

		imu.reset();
		// ************************************************************************************

		r = new Rect[hookImagesAllowed - 1];
		x = new int[hookImagesAllowed - 1];
		y = new int[hookImagesAllowed - 1];
		height = new int[5];
		width = new int[hookImagesAllowed - 1];
		xv = new int[hookImagesAllowed - 1];
		yv = new int[hookImagesAllowed - 1];
		heightv = new int[hookImagesAllowed - 1];
		widthv = new int[hookImagesAllowed - 1];

		ar = new int[hookImagesAllowed - 1];

		if (hookVisionStreamOn)
			gearHookCamera = CameraServer.getInstance().startAutomaticCapture("Gear Hook Cam", 0);
		else
			gearHookCamera = new UsbCamera("Gear Hook Cam", 0);

		gearHookCamera.setResolution(IMG_WIDTH, IMG_HEIGHT);
		gearHookCamera.setBrightness(8);
		gearHookCamera.setExposureManual(1);
		gearHookCamera.setWhiteBalanceManual(3000);

		hookVisionThread = new VisionThread(gearHookCamera, new HookPipeline(), hookPipeline -> {
			// while (!Thread.interrupted()) {
			hookNumberImages = hookPipeline.filterContoursOutput().size();
			hookThreadCounter += 1;

			while (!hookVisionTurnedOn && !Thread.currentThread().isInterrupted())
				try {
					Thread.sleep(10000);
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}

			if (hookVisionTurnedOn) {
				hookNumberImages = hookPipeline.filterContoursOutput().size();
				validar = 0;

				if (hookNumberImages > 0 && hookNumberImages <= (hookImagesAllowed - 1)) {
					for (int i = 0; i < hookNumberImages; i++) {
						r[i] = Imgproc.boundingRect(hookPipeline.filterContoursOutput().get(i));

						synchronized (imgLock) {
							height[i] = r[i].height;
							width[i] = r[i].width;
							x[i] = r[i].x;
							y[i] = r[i].y;
							ar[i] = width[i] * 100 / height[i];
							if (ar[i] >= minHookAr && ar[i] <= maxHookAr) {
								xv[validar] = x[i];
								yv[validar] = y[i];
								heightv[validar] = height[i];
								widthv[validar] = width[i];
								validar += 1;
							}
						}

						noImages = 0;
						x0 = xv[0];
						y0 = yv[0];
						gVHeight0 = heightv[0];
						width0 = widthv[0];
						x1 = xv[1];
						y1 = yv[1];
						height1 = heightv[1];
						width1 = widthv[1];
					}

				}

				if (noImages > 2) {
					validar = 0;
					hookNumberImages = 0;
					x0 = 0;
					y0 = 0;
					gVHeight0 = 0;
					width0 = 0;
					x1 = 0;
					y1 = 0;
					height1 = 0;
					width1 = 0;
				}
			}

		}
		// }
		);
		hookVisionThread.setDaemon(true);
		hookVisionThread.start();
		// the boiler vision thread uses common variable with the hook vision
		boilerVisionThread = new VisionThread(gearHookCamera, new BoilerPipeline(), boilerPipeline -> {
			// while (!Thread.interrupted()) {
			hookNumberImages = boilerPipeline.filterContoursOutput().size();
			boilerThreadCounter += 1;

			while (!boilerVisionTurnedOn && !Thread.currentThread().isInterrupted())
				try {
					Thread.sleep(10000);
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}

			if (boilerVisionTurnedOn) {
				hookNumberImages = boilerPipeline.filterContoursOutput().size();
				validar = 0;

				if (hookNumberImages > 0 && hookNumberImages <= (hookImagesAllowed - 1)) {
					for (int i = 0; i < hookNumberImages; i++) {
						r[i] = Imgproc.boundingRect(boilerPipeline.filterContoursOutput().get(i));

						synchronized (imgLock) {
							height[i] = r[i].height;
							width[i] = r[i].width;
							x[i] = r[i].x;
							y[i] = r[i].y;
							// tape strips are different heights so 2 acceptable
							// aspect ratios
							// if ((width[i] >= minBoilerWidth && ar[i] <=
							// maxBoilerWidth)) {
							if ((width[i] >= minBoilerWidth && width[i] <= maxBoilerWidth)) {
								xv[validar] = x[i];
								yv[validar] = y[i];
								heightv[validar] = height[i];
								widthv[validar] = width[i];
								validar += 1;
							}
						}

						noImages = 0;
						x0 = xv[0];
						y0 = yv[0];
						gVHeight0 = heightv[0];
						width0 = widthv[0];
						x1 = xv[1];
						y1 = yv[1];
						height1 = heightv[1];
						width1 = widthv[1];
					}

				}

				if (noImages > 2) {
					validar = 0;
					hookNumberImages = 0;
					x0 = 0;
					y0 = 0;
					gVHeight0 = 0;
					width0 = 0;
					x1 = 0;
					y1 = 0;
					height1 = 0;
					width1 = 0;
				}
			}

		}
		// }
		);
		boilerVisionThread.setDaemon(true);
		boilerVisionThread.start();

		// data handling from the vision pipeline is done in the VisionData
		// subsystem
		// results can be obtained through methods such as getVisionComp()

		visionThreadAddTargeting = new Thread(() -> {
			// while (!hookVisionTurnedOn &&
			// !Thread.currentThread().isInterrupted())
			// try {
			// Thread.sleep(10000);
			// } catch (Exception e) {
			// // TODO Auto-generated catch block
			// e.printStackTrace();
			// }

			// // Get the UsbCamera from CameraServer
			// // Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo(gearHookCamera);
			// // Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("HookTarget", IMG_WIDTH, IMG_HEIGHT);
			// // Mats are very memory expensive. Lets reuse this Mat.

			Mat mat = new Mat();

			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted() && hookVisionTargetingOn) {
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat. If there is an error notify the output.
				if (cvSink.grabFrame(mat) == 0) {
					// Send the output the error.
					outputStream.notifyError(cvSink.getError());
					// skip the rest of the current iteration
					continue;
				}
				// Draw on the image

				// image vertical center line
				Imgproc.line(mat, new Point(IMG_WIDTH / 2, 0), new Point(IMG_WIDTH / 2, IMG_HEIGHT - 1), red, 2);
				int x1Val = 0;
				int y1Val = 0;
				int x2Val = 0;
				int y2Val = 0;

				if (hookVisionTurnedOn) {

					// target box 1

					x1Val = Range.ensure(Robot.visionData.getLeftX(), 0, IMG_WIDTH - 1);
					y1Val = Range.ensure(Robot.visionData.getLeftY(), 0, IMG_HEIGHT - 1);
					x2Val = Range.ensure(Robot.visionData.getLeftX() + Robot.visionData.getLeftWidth(), x1Val + 1,
							IMG_WIDTH - 1);
					y2Val = Range.ensure(Robot.visionData.getLeftY() + Robot.visionData.getLeftHeight(), y1Val + 1,
							IMG_HEIGHT - 1);

					Imgproc.rectangle(mat, new Point(x1Val, y1Val), new Point(x2Val, y2Val), red, 2);
					// // target box 2
					x1Val = Range.ensure(Robot.visionData.getRightX(), 0, IMG_WIDTH - 1);
					y1Val = Range.ensure(Robot.visionData.getRightY(), 0, IMG_HEIGHT - 1);
					x2Val = Range.ensure(Robot.visionData.getRightX() + Robot.visionData.getRightWidth(), x1Val + 1,
							IMG_WIDTH - 1);
					y2Val = Range.ensure(Robot.visionData.getRightY() + Robot.visionData.getRightHeight(), y1Val + 1,
							IMG_HEIGHT - 1);
					//
					Imgproc.rectangle(mat, new Point(x1Val, y1Val), new Point(x2Val, y2Val), red, 2);
					//
					// // center of targets
					x1Val = Range.ensure(IMG_WIDTH / 2 - Robot.visionData.getHookTargetCorrectionPixels(), 0,
							IMG_WIDTH - 1);
					x2Val = Range.ensure(IMG_WIDTH / 2 - Robot.visionData.getHookTargetCorrectionPixels(), 0,
							IMG_WIDTH - 1);

					Imgproc.line(mat, new Point(x1Val, 0), new Point(x2Val, IMG_HEIGHT - 1), green, 2);
					// target center

				}
				if (boilerVisionTurnedOn) {

					x1Val = Range.ensure(Robot.visionData.getX0(), 0, IMG_WIDTH - 1);
					y1Val = Range.ensure(Robot.visionData.getY0(), 0, IMG_HEIGHT - 1);
					x2Val = Range.ensure(Robot.visionData.getX0() + Robot.visionData.getWidth0(), x1Val + 1,
							IMG_WIDTH - 1);
					y2Val = Range.ensure(Robot.visionData.getY0() + Robot.visionData.getHeight0(), y1Val + 1,
							IMG_HEIGHT - 1);

					Imgproc.rectangle(mat, new Point(x1Val, y1Val), new Point(x2Val, y2Val), red, 2);
					// // target box 2

					x1Val = Range.ensure(Robot.visionData.getX1(), 0, IMG_WIDTH - 1);
					y1Val = Range.ensure(Robot.visionData.getY1(), 0, IMG_HEIGHT - 1);
					// if(y1Val <= y2Val)y1Val = y2Val +1;//make sure they don't
					// overlap

					x2Val = Range.ensure(Robot.visionData.getX1() + Robot.visionData.getWidth1(), x1Val + 1,
							IMG_WIDTH - 1);
					y2Val = Range.ensure(Robot.visionData.getY1() + Robot.visionData.getHeight1(), y1Val + 1,
							IMG_HEIGHT - 1);

					Imgproc.rectangle(mat, new Point(x1Val, y1Val), new Point(x2Val, y2Val), red, 2);

					Imgproc.line(mat,
							new Point(Robot.visionData.getAverageRightEdge()
									- (Robot.visionData.getBoilerTargetCorrectionPixels()), 0),
							new Point(Robot.visionData.getAverageRightEdge()
									- (Robot.visionData.getBoilerTargetCorrectionPixels()), IMG_HEIGHT),
							green, 2);
					Imgproc.line(mat, new Point(Robot.visionData.getAverageMidX(), 0),
							new Point(Robot.visionData.getAverageMidX(), IMG_HEIGHT), blue, 2);
				}
				// Give the output stream a new image to display
				outputStream.putFrame(mat);
			}
		});
		if (hookVisionTargetingOn) {
			visionThreadAddTargeting.setDaemon(true);
			visionThreadAddTargeting.start();
		}
		gearPickupCamera = CameraServer.getInstance().startAutomaticCapture("Gear Pick Up Cam", 1);
		gearPickupCamera.setResolution(240, 135);
		gearPickupCamera.setFPS(15);
		gearPickupCamera.setExposureManual(1);

		// gearPickupCamera.setExposureHoldCurrent();

		gearVisionThread = new VisionThread(gearPickupCamera, new GearPipeline(), gearPipeline -> {
			// while (!Thread.interrupted()) {
			gearNumberImages = gearPipeline.convexHullsOutput().size();
			gearThreadCounter += 1;

			while (!gearVisionTurnedOn && !Thread.currentThread().isInterrupted())
				try {
					Thread.sleep(10000);
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}

			if (gearVisionTurnedOn == true) {
				gearNumberImages = gearPipeline.convexHullsOutput().size();
			}
			if (gearNumberImages > 0) {
				r0 = Imgproc.boundingRect(gearPipeline.convexHullsOutput().get(0));

				synchronized (gearImgLock) {
					gearHeight0 = r0.height;
					gearWidth0 = r0.width;
					gearX0 = r0.x;
					gearY0 = r0.y;
				}
			}
			// }
		});
		gearVisionThread.setDaemon(true);
		gearVisionThread.start();

		gearThreadAddTargeting = new Thread(() ->

		{
			// while (!hookVisionTurnedOn &&
			// !Thread.currentThread().isInterrupted())
			// try {
			// Thread.sleep(10000);
			// } catch (Exception e) {
			// // TODO Auto-generated catch block
			// e.printStackTrace();
			// }

			// Get the UsbCamera from CameraServer
			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink1 = CameraServer.getInstance().getVideo(gearPickupCamera);
			// // Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("GearTarget", 480, 270);
			// // Mats are very memory expensive. Lets reuse this Mat.

			Mat mat1 = new Mat();

			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted() && gearVisionTargetingOn) {
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat. If there is an error notify the output.
				if (cvSink1.grabFrame(mat1) == 0) {
					// Send the output the error.
					outputStream.notifyError(cvSink1.getError());
					// skip the rest of the current iteration
					continue;
				}
				Imgproc.rectangle(mat1, new Point(Robot.gearVision.getX0(), Robot.gearVision.Y0()),
						new Point(Robot.gearVision.getX0() + Robot.gearVision.getWidth0(),
								Robot.gearVision.Y0() + Robot.gearVision.getHeight0()),
						red, 2);
				Imgproc.line(mat1, new Point(0, 270 / 2), new Point(480, 270 / 2), red);
				Imgproc.line(mat1, new Point(IMG_WIDTH / 2, 0), new Point(IMG_WIDTH / 2, IMG_HEIGHT), red);
				Imgproc.line(mat1, new Point(IMG_WIDTH / 2, IMG_HEIGHT),
						new Point(Robot.gearVision.getCenterX0(), Robot.gearVision.getCenterY0()), red, 6);
				Imgproc.line(mat1, new Point(IMG_WIDTH / 2, IMG_HEIGHT),
						new Point(Robot.gearVision.getCenterX0(), IMG_HEIGHT), red, 6);
				Imgproc.line(mat1, new Point(Robot.gearVision.getCenterX0(), IMG_HEIGHT),
						new Point(Robot.gearVision.getCenterX0(), Robot.gearVision.getCenterY0()), red, 6);
				// Give the output stream a new image to display
				outputStream.putFrame(mat1);
			}
		});
		if (gearVisionTargetingOn) {
			gearThreadAddTargeting.setDaemon(true);
			gearThreadAddTargeting.start();
		}

	}

	// *******************************************************************************************
	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		updateStatus();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		updateStatus();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by ading additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = autoChooser.getSelected();
		autonomousCommand.start();
		new GripGear().start();
		new RetractGear().start();
		new LiftGear().start();
		gearHookCamera.setBrightness(8);
		new LowerCamera();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();

	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();

		updateStatus();
	}

	@Override
	public void teleopInit() {
		(new HookVisionOnOff(false)).start();
		(new StopShooters()).start();
		(new StopFeeders()).start();
		(new StopAgitator()).start();
		(new RetractGear()).start();
		(new GripGear()).start();
		(new SelectGearCamera()).start();
		new LowerCamera();

		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();

		// ********************************************************************************************

		//
		updateStatus();

	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}

	public void updateStatus() {
		// *********************************************************************************************
		if (hookVisionTurnedOn && hookVisionThread.getState() == Thread.State.TIMED_WAITING) {
			hookVisionThread.interrupt();
		}
		if (boilerVisionTurnedOn && boilerVisionThread.getState() == Thread.State.TIMED_WAITING) {
			boilerVisionThread.interrupt();
		}

		if (hookVisionTargetingOn && (hookVisionTurnedOn || boilerVisionTurnedOn)
				&& visionThreadAddTargeting.getState() == Thread.State.TIMED_WAITING) {
			visionThreadAddTargeting.interrupt();
		}
		if (gearVisionTurnedOn && gearVisionThread.getState() == Thread.State.TIMED_WAITING) {
			gearVisionThread.interrupt();
		}
		if (gearVisionTargetingOn && gearVisionTurnedOn
				&& gearThreadAddTargeting.getState() == Thread.State.TIMED_WAITING) {
			gearThreadAddTargeting.interrupt();
		}
		if (!RobotMap.topShooter.isControlEnabled() && !Robot.airCompressor.isRunning()) {
			Robot.airCompressor.start();
		}

		if (viewHookCamera && !showTargeting && hookVisionStreamOn && hookVisionTargetingOn)
			NetworkTable.getTable("").putString("CameraSelection", gearHookCamera.getName());

		if (viewHookCamera && showTargeting && hookVisionStreamOn && hookVisionTargetingOn)
			NetworkTable.getTable("").putString("CameraSelection", "HookTarget");

		if (viewGearCamera)
			NetworkTable.getTable("").putString("CameraSelection", gearPickupCamera.getName());

		if (hookVisionStreamOn && (hookVisionTurnedOn || boilerVisionTurnedOn || hookBoilerCameraShow)) {
			viewHookCamera = true;
			viewGearCamera = false;
		} else {
			viewHookCamera = false;
			viewGearCamera = true;

		}
		if (runSmartDashboard) {
			SmartDashboard.putBoolean("JVComp", visionCompJoystick);
			SmartDashboard.putBoolean("JGComp", gyroCompJoystick);
			SmartDashboard.putBoolean("JGPComp", gearPickupJoystick);
			SmartDashboard.putString("HookCam", gearHookCamera.getDescription());
			SmartDashboard.putString("GearPickupCam", gearPickupCamera.getDescription());
			SmartDashboard.putNumber("Hook Thread Counter", hookThreadCounter);
			SmartDashboard.putNumber("Boiler Thread Counter", boilerThreadCounter);
			// SmartDashboard.putNumber("Gear Thread Counter",
			// gearThreadCounter);
			// SmartDashboard.putNumber("HookCam Bright",
			// gearHookCamera.getBrightness());

			// ****************************************************************************************

			//
			Robot.visionData.updateStatus();
			// Robot.leftUltrasoundDrive.updateStatus();
			// Robot.rightUltrasoundDrive.updateStatus();
			Robot.gyroRotate.updateStatus();
			Robot.leftSideDrive.updateStatus();
			Robot.rightSideDrive.updateStatus();
			Robot.driveTrain.updateStatus();
			// Robot.ultraSound.updateStatus();
			Robot.pistonControl.updateStatus();
			Robot.feeder.updateStatus();
			Robot.shooter.updateStatus();
			Robot.agitator.updateStatus();
			Robot.gearVision.updateStatus();
			Robot.powerPanel.updateStatus();

			SmartDashboard.putNumber("Timer", Timer.getMatchTime());
			// Robot.lidarSensor.updateStatus();
			// SmartDashboard.putBoolean("Is in High Gear?", isInHighGear);
			SmartDashboard.putBoolean("Has Gear?", hasGear);
			smartDashboardTimer = Timer.getFPGATimestamp();
			runSmartDashboard = false;
		} else {
			runSmartDashboard = Timer.getFPGATimestamp() - smartDashboardTimer > .5;
		}
	}
}
