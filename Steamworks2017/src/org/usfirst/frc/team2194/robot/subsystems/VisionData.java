package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class VisionData extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public int testAvge;

	private int validImagesStartCount;
	private int lookUpIndex;
	private int segmentPixelRange;
	public static int numberReadings = 28;

	public int[] heightPx;
	public double[] distanceInches;;
	public int[] targetWidthPixels;
	public double[] HFOV = new double[numberReadings];;
	public int iPx;
	private double distanceRange;
	private int pixelsIntoRange;
	private int threadCounter;

	private int widthRange;

	public static double minVisionKp;//
	public static double maxVisionKp;
	public static double minVisionKpInches = 15;
	public static double maxVisionKpInches = 60;
	private static double visionKpPerInch;
	private static double visionKp;

	public static double widthBetweenTargets = 8.5;
	public static double cameraToRobotFrontDistance = 8;
	public static double gearHookLength = 10.5;
	public static int boilerStripWidth = 15;// inches

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		// setDefaultCommand(new RunFromGamepad());
	}

	public void init() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		heightPx = new int[] { 0, 0, 0, 0, 0, 23, 25, 30, 35, 40, 45, 50, 55, 60, 65, 71, 77, 80, 86, 90, 97, 0, 0, 0,
				0, 0, 0, 0 };
		targetWidthPixels = new int[] { 0, 0, 0, 0, 0, 45, 48, 60, 71, 82, 93, 104, 114, 127, 139, 151, 166, 171, 187,
				199, 212, 0, 0, 0, 0, 0, 0, 0 };
		distanceInches = new double[] { 0, 0, 0, 0, 0, 89.5, 82.75, 64, 51, 43.6, 35.7, 30.9, 27.4, 24.2, 21.1, 20,
				15.7, 14, 13.3, 12.7, 9.8, 0, 0, 0, 0, 0, 0, 0 };

		for (int i = 0; i < VisionData.numberReadings; i++) {
			Robot.visionData.HFOV[i] = Math.atan(Math.toDegrees((Robot.fullTargetAreaWidth * Robot.IMG_WIDTH)
					/ (2 * Robot.visionData.distanceInches[i] * Robot.visionData.targetWidthPixels[i])));
		}
		minVisionKp = Robot.prefs.getDouble("minVisionKp", .001);
		maxVisionKp = Robot.prefs.getDouble("maxVisionKp", .005);

		visionKpPerInch = (maxVisionKp - minVisionKp) / (maxVisionKpInches - minVisionKpInches);
	}

	public boolean getHookVisionOn() {
		return Robot.hookVisionTurnedOn;
	}

	public boolean getBoilerVisionOn() {
		return Robot.boilerVisionTurnedOn;
	}

	public int getNumberOfImages() {
		return Robot.hookNumberImages;
	}

	public int getNumberOfValidImages() {
		return Robot.validar;
	}

	public boolean isValidNumberImages() {
		return getNumberOfValidImages() == 2;
	}

	// look for minimum 5 good vision captures in a row
	public boolean getConsistentValidImages() {
		if (Robot.hookVisionTurnedOn)
			threadCounter = Robot.hookThreadCounter;
		else
			threadCounter = Robot.boilerThreadCounter;
		if (!isValidNumberImages())
			validImagesStartCount = -6;
		if (isValidNumberImages() && validImagesStartCount == -6)
			validImagesStartCount = threadCounter;
		return (threadCounter > validImagesStartCount + 3) && validImagesStartCount != -6;

	}
	// *************************************************************************

	public int getX0() {
		return Robot.x0;
	}

	public int getY0() {
		return Robot.y0;
	}

	public int getHeight0() {
		return Robot.gVHeight0;
	}

	public int getWidth0() {
		return Robot.width0;
	}
	public int getAspectRatio0(){
		if (getHeight0()!=0)
		return (100*getWidth0())/getHeight0();
		else return 0;
	}
	public int getPerimeter0(){
		return 2* (getWidth0()+ getHeight0());
	}


	public int getMidXTop() {
		return getX0() + getWidth0() / 2;
	}

	public int getMidYTop() {
		return getY0() + getHeight0() / 2;
	}

	public int getX1() {
		return Robot.x1;
	}

	public int getY1() {
		return Robot.y1;
	}

	public int getHeight1() {
		return Robot.height1;
	}

	public int getWidth1() {
		return Robot.width1;
	}
	public int getAspectRatio1(){
		if(getHeight1()!=0)
		return (100*getWidth1())/getHeight1();
		else return 0;
	}
	
	public int getPerimeter1(){
		return 2* (getWidth1()+ getHeight1());
	}


	public int getMidXLower() {
		return getX1() + getWidth1() / 2;
	}

	public int getMidYLower() {
		return getY1() + getHeight1() / 2;
	}

	public int getAverageMidX() {
		return (getMidXTop() + getMidXLower()) / 2;
	}

	public int getAverageMidY() {
		return (getMidYTop() + getMidYLower()) / 2;
	}

	public int getTopRightEdge() {
		return (getX0() + getWidth0());
	}

	public int getBottomRightEdge() {
		return (getX1() + getWidth1());
	}

	public int getAverageRightEdge() {
		return (getTopRightEdge() + getBottomRightEdge()) / 2;
	}

	public int getBoilerWidthPxPerInch() {
		return 100*(getWidth0() + getWidth1()) / 2 / boilerStripWidth;
	}

	// ************************************************************
	// **************************************************************
	public int getLeftX() {
		if (Robot.x0 < Robot.x1)
			return Robot.x0;
		else
			return Robot.x1;
	}

	public int getLeftWidth() {
		if (getX0() <= getX1())
			return getWidth0();
		else
			return getWidth1();
	}

	public int getLeftY() {
		if (Robot.x0 < Robot.x1)
			return Robot.y0;
		else
			return Robot.y1;
	}

	public int getLeftHeight() {
		if (getX0() <= getX1())
			return getHeight0();
		else
			return getHeight1();
	}

	public int getLeftXCenter() {
		return getLeftX() + (getLeftWidth() / 2);

	}

	public int getLeftYCenter() {
		return getLeftY() + (getLeftHeight() / 2);
	}

	public int getLeftAspectRatio() {
		return getLeftWidth() * 100 / (getLeftHeight() + 1);

	}

	// ************************************************
	public int getRightX() {
		if (Robot.x0 < Robot.x1)
			return Robot.x1;
		else
			return Robot.x0;
	}

	public int getRightY() {
		if (Robot.x0 < Robot.x1)
			return Robot.y1;
		else
			return Robot.y0;
	}

	public int getRightWidth() {
		if (getX0() <= getX1())
			return getWidth1();
		else
			return getWidth0();
	}

	public int getRightHeight() {
		if (getX0() <= getX1())
			return getHeight1();
		else
			return getHeight0();
	}

	public int getRightAspectRatio() {
		return getRightWidth() * 100 / (getRightHeight() + 1);

	}

	public int getRightYCenter() {
		return getRightY() + (getRightHeight() / 2);
	}

	public int getAverageYCenter() {
		return (getRightYCenter() + getLeftYCenter()) / 2;
	}

	public int getRightXCenter() {
		return getRightX() + (getRightWidth() / 2);
	}

	// ************************************************

	public double getWidthDifference() {
		return getLeftWidth() - getRightWidth();
	}

	public int getAverageWidth() {
		return (getLeftWidth() + getRightWidth()) / 2;
	}

	public int getTargetsCenter() {
		return (getLeftXCenter() + getRightXCenter()) / 2;
	}

	// ************************************************************************************
	// a negative number means move robot left

	public int getHookTargetCorrectionPixels() {
		return (Robot.IMG_WIDTH / 2) - (getTargetsCenter());//
	}

	// camera is offset from shooter by approx 7.5 inches
	public int getBoilerTargetCorrectionPixels() {
		return (Robot.IMG_WIDTH / 2) - (getAverageRightEdge());
	}

	// calculated during approach to target to compensate for approaching at an
	// angle instead of straight on. Determined by gyro error from known target
	// angle Approaching from left means push hook point target to right
	// gyro error is positive if it is turned right from zero
	// so it must be subtracted from target value to get hook point value

	public int getAngleCorrectionPixels() {
		return Robot.angleCorrectionPixels;// set in motion to target routine(s)
	}

	public int getHookPointCorrectionPixels() {
		return getHookTargetCorrectionPixels() - getAngleCorrectionPixels();
	}
	// public int getBoilerCorrectionPixels(){
	// return 99;
	// }

	public double getVisionComp() {
		if (Robot.hookVisionTurnedOn)
			return getHookPointCorrectionPixels() * getVisionKpAtDistance();
		else
			return getBoilerTargetCorrectionPixels() * getVisionKpAtDistance() * .75;
	}

	// ************************************************************************************

	public double getPixelsPerInchFromTargetWidth() {
		return getTargetWidthPixels() / Robot.fullTargetAreaWidth;
	}

	public double getTargetCorrectionInches() {
		return getHookTargetCorrectionPixels() / getPixelsPerInchFromTargetWidth();
	}

	public double getTargetCorrectionAngle() {
		return Math.toDegrees(Math.atan(getTargetCorrectionInches() / Robot.remainingDistanceToHook));
	}

	public double getVisionKpAtDistance() {
		// minVisionKp = Robot.prefs.getDouble("minVisionKp", .001);
		// maxVisionKp = Robot.prefs.getDouble("maxVisionKp", .005);
		// prorate vision kp based on distance from target
		// the further away, the lower the kp should be since pixels
		// have a smaller inch value farther away than they do close uo
		minVisionKp = Robot.prefs.getDouble("minVisionKp", .001);
		maxVisionKp = Robot.prefs.getDouble("maxVisionKp", .005);

		visionKpPerInch = (maxVisionKp - minVisionKp) / (maxVisionKpInches - minVisionKpInches);
		if (Robot.visionCompJoystick)
			visionKp = maxVisionKp
					- ((Robot.visionData.getDistanceFromHeightArray() - minVisionKpInches) * visionKpPerInch);
		else
			visionKp = maxVisionKp - ((Robot.remainingDistanceToHook - minVisionKpInches) * visionKpPerInch);

		if (visionKp < minVisionKp) {
			visionKp = minVisionKp;
		}
		if (visionKp > maxVisionKp) {
			visionKp = maxVisionKp;
		}
		return visionKp;
	}

	// ***************************************************************************

	public double getHorizontalFOV() {

		return 0;
	}

	public double getAngleToTargetCenter() {
		double angleToGoal = 0;
		if (isValidNumberImages()) {
			double distanceFromCenterPixels = getTargetsCenter() - (Robot.IMG_WIDTH / 2);
			double distanceFromCenterInch = distanceFromCenterPixels * getPixelsPerInchFromTargetWidth();
			angleToGoal = Math.toDegrees(Math.atan(distanceFromCenterInch / getDistanceFromHeightArray()));
		}
		return angleToGoal;
	}

	// get angle pixel cmp from ultrasound angle
	public int getAnglePixelComp() {
		return (int) (gearHookLength / Math.tan(Robot.ultraSound.readUltrasoundAngle() * Math.PI / 180));
	}

	public boolean inRangeTargetCenter() {
		return Math.abs(Robot.visionData.getHookTargetCorrectionPixels()) < Robot.correctionPixelsInLimit;
	}

	// ****************************************************

	public int getAverageHeight() {
		return (getLeftHeight() + getRightHeight()) / 2;
	}

	public int getTargetWidthPixels() {
		return (getRightX() + getRightWidth()) - getLeftX();
	}

	public double getDistanceFromHeightArray() {
		if (getAverageHeight() > 0 && getAverageHeight() < 130) {
			lookUpIndex = getAverageHeight() / 5;
			segmentPixelRange = (heightPx[lookUpIndex + 1] - heightPx[lookUpIndex]);
			if (segmentPixelRange != 0) {
				pixelsIntoRange = (getAverageHeight() - lookUpIndex * 5);
				distanceRange = (distanceInches[lookUpIndex] - distanceInches[lookUpIndex + 1]);
				return (distanceInches[lookUpIndex]
						- ((distanceRange * (double) pixelsIntoRange) / (double) segmentPixelRange));
			} else
				return 0;
		} else
			return 0;
	}

	public int getTargetWidthFromHeightArray() {
		if (getAverageHeight() > 0 && getAverageHeight() < 130) {
			lookUpIndex = getAverageHeight() / 5;
			segmentPixelRange = (heightPx[lookUpIndex + 1] - heightPx[lookUpIndex]);
			if (segmentPixelRange != 0) {
				pixelsIntoRange = (getAverageHeight() - lookUpIndex * 5);
				widthRange = (targetWidthPixels[lookUpIndex + 1] - targetWidthPixels[lookUpIndex]);
				return (targetWidthPixels[lookUpIndex] + ((widthRange * pixelsIntoRange) / segmentPixelRange));
			} else
				return 0;
		} else
			return 0;
	}

	public void updateStatus() {
//		SmartDashboard.putBoolean("Hook Vision On", getHookVisionOn());
//		SmartDashboard.putBoolean("Boiler Vision On", getBoilerVisionOn());
		SmartDashboard.putNumber("Number Hook Images", getNumberOfImages());
//		SmartDashboard.putBoolean("Valid Images", isValidNumberImages());
		SmartDashboard.putBoolean("Valid Consistent Images", getConsistentValidImages());
		SmartDashboard.putNumber("Valid AR Images", getNumberOfValidImages());
//		SmartDashboard.putNumber("Boiler Correction Pixels", getBoilerTargetCorrectionPixels());
//		SmartDashboard.putNumber("Boiler WidthPX", this.getAverageWidth());
		// **********************************************************************************
		// **********************************************************************************
//		SmartDashboard.putNumber("Rect 0 Height", getHeight0());
//		SmartDashboard.putNumber("Rect 0 Width", getWidth0());
//		SmartDashboard.putNumber("Rect 0 AR", getAspectRatio0());
//		SmartDashboard.putNumber("Rect 0 Perimeter", getPerimeter0());
//
//		SmartDashboard.putNumber("Rect 1 Height", getHeight1());
//		SmartDashboard.putNumber("Rect 1 Width", getWidth1());
//		SmartDashboard.putNumber("Rect 1 AR", getAspectRatio1());
//		SmartDashboard.putNumber("Rect 1 Perimeter", getPerimeter1());

		// SmartDashboard.putNumber("Angle To Goal",
		// Math.round(getAngleToTargetCenter() * 100.) / 100.);
		// ****************************************************************************************
		// SmartDashboard.putNumber("TargetWidth", getTargetWidthPixels());
		// SmartDashboard.putNumber("Average Height", getAverageHeight());
		// SmartDashboard.putNumber("PxHeight", heightPx[iPx]);
		// SmartDashboard.putNumber("PxDistance", distanceInches[iPx]);
		// SmartDashboard.putNumber("PxIndex", iPx);
		// SmartDashboard.putNumber("PxWidth", targetWidthPixels[iPx]);
		//
		// SmartDashboard.putNumber("ArrayDistance",
		// getDistanceFromHeightArray());
		// SmartDashboard.putNumber("ArrayTargetWidth",
		// getTargetWidthFromHeightArray());

		// ********************************************************************************************
		// SmartDashboard.putNumber("Vision Kp Distance",
		// getVisionKpAtDistance());
		// SmartDashboard.putNumber("Vision Kp per Inch", visionKpPerInch *
		// 100);
		// *************************************************************************************************
		// SmartDashboard.putNumber("Vision Comp", Math.round(getVisionComp() *
		// 100.) / 100.);
		// **********************************************************************
	}

}
