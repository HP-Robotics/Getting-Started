package org.usfirst.frc.team2823.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	static final double R = 2.5; // inches
	static final double LEVEL_HEIGHT = 15; // inches
	static final double ENCODER_RESOLUTION = 2048; // counts per revolution
	static final double GEAR_RATIO = (15.0 / 22.0); // gearbox is
													// 1:50,
													// chain is
													// 22:15
	static final int MAXIMUM_LEVEL = 78; // inches
	static final int MINIMUM_LEVEL = 0; // inches
	static final double RAW_ELEVATOR_STEP = 0.3; // victor speed
	static final double MAX_DRIVE_SPEED = 0.42; // talon motor input
	static final double DRIVE_R = 3;
	static final double DRIVE_RATIO = 1;

	static enum ShimmyMode {
		DISABLED, FINISHED, LEFT, RIGHT
	}

	// RobotDrive myRobot;
	Joystick stick;
	int autoLoopCounter;
	Talon talon1;
	Talon talon2;
	Talon talon3;
	Talon talon4;
	VictorSP victor;
	Encoder elevatorEncoder;
	Encoder rightEncoder;
	Encoder leftEncoder;
	DigitalInput switchTop;
	DigitalInput switchBottom;
	AnalogInput infraredSensorLeft; // used for testing the IR sensor
	AnalogInput infraredSensorRight;
	AnalogInput gyroInput;

	Gyro myGyro;
	PIDController elevatorControl;
	PIDController slowElevatorControl;

	PIDController turningControl;
	PIDController leftDrivingControl;
	PIDController rightDrivingControl;

	PIDController leftIRControl;
	PIDController rightIRControl;

	Command autoCommand;
	SendableChooser autoChooser;

	double motorScale = 0.97;
	double currentAngle = 0;
	double gyroResetAngle = -1;
	double globalAngleDesired = -1;

	final static double[] voltagesLeft = new double[] { 2.0, 1.79, 1.64, 1.38,
			1.24, 1.13, 1.03, 0.93, 0.88, 0.82, 0.78, 0.73, 0.67, 0.61, 0.58,
			0.56, 0.54, 0.52, 0.50, 0.49, 0.48 };
	final static double[] distancesLeft = new double[] { -0.5, 0, 1, 2, 3, 4,
			5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 20, 24, 28, 32, 36 };

	final static double[] voltagesRight = new double[] { 2.0, 1.80, 1.50, 1.37,
			1.22, 1.11, 1.02, 0.90, 0.89, 0.83, 0.78, 0.73, 0.69, 0.63, 0.60,
			0.56, 0.53, 0.460, 0.39, 0.37, 0.34 };
	final static double[] distancesRight = new double[] { -0.5, 0, 1, 2, 3, 4,
			5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 20, 24, 28, 32, 36 };

	final static double[] levels = { 0, 8.864, 26.597, 44.477, 54.507 };
	final static String[] levelNames = { "!B", "!T1", "!T2", "!T3", "!TP" };
	double myDriveDistance = 144.0;// 0.6;

	boolean BAMUpPressed = false;
	boolean BAMDownPressed = false;
	boolean switchBottomPressed = false;
	boolean elevatorResetPressed = false;
	boolean StraightMode = false;
	boolean messagePrinted = false;
	boolean autoChosen = false;
	ShimmyMode shimmy = ShimmyMode.DISABLED;

	int shimmyCount = 0;
	Timer shimmyTime = new Timer();

	boolean rightStarted = false;
	boolean leftStarted = false;
	double elevatorOffset = levels[4];
	int elevatorIndex = 4;

	Timer EVTimer = new Timer(); // EV = Elevator Velocity
	double timerLast;
	double elevatorVelocity = 0;
	double encoderLast = 0;

	double shimmyTimeout = 0.05;
	int shimmyMaxCount = 5;
	double shimmyPower = 1.0;
	int shimmyElevatorStart = 3;
	
	double rightWheelPosition = 0;
	double leftWheelPosition = 0;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		stick = new Joystick(0);
		talon1 = new Talon(1);
		talon2 = new Talon(2);
		talon3 = new Talon(3);
		talon4 = new Talon(4);
		victor = new VictorSP(0);
		elevatorEncoder = new Encoder(4, 5, true, EncodingType.k4X);
		rightEncoder = new Encoder(0, 1, true, EncodingType.k4X);
		leftEncoder = new Encoder(2, 3, true, EncodingType.k4X);
		infraredSensorRight = new AnalogInput(1);
		infraredSensorLeft = new AnalogInput(2);
		gyroInput = new AnalogInput(0);
		myGyro = new Gyro(gyroInput);
		switchTop = new DigitalInput(6);
		switchBottom = new DigitalInput(7);
		elevatorControl = new PIDController(0.4, 0.0, 0.0, new InchesEncoder(
				elevatorEncoder), new SwitchOverride(new PIDOutputClamp(victor,
				1.0)));
		slowElevatorControl = new PIDController(0.4, 0.0, 0.0,
				new InchesEncoder(elevatorEncoder), new SwitchOverride(
						new PIDOutputClamp(victor, 0.7)));
		// 0.4 clamp value may be better.
		turningControl = new PIDController(4.0, 0.01 / 1000, 14.0, myGyro,
				new PIDOutputClamp(new GyroPIDOutput(), 0.6 * 100));
		turningControl.setPercentTolerance(2);
		turningControl.setOutputRange(-100, 100);
		leftDrivingControl = new PIDController(0.06, 0.01 / 1000, 0.30,
				new DriveInchesEncoder(leftEncoder), new PIDOutputClamp(
						new RightDrivePIDOutput(), 0.5));
		rightDrivingControl = new PIDController(0.06, 0.0, 0.30,
				new DriveInchesEncoder(rightEncoder), new PIDOutputClamp(
						new LeftDrivePIDOutput(), 0.5));

		leftDrivingControl.setPercentTolerance(10); // too high probably
		rightDrivingControl.setPercentTolerance(10);

		leftIRControl = new PIDController(0.125, 0.001, 0.400, new IRPIDSource(
				infraredSensorLeft, distancesLeft, voltagesLeft),
				new PIDOutputClamp(new IRPIDOutputLeft(), 0.4));
		rightIRControl = new PIDController(0.125, 0.001, 0.400,
				new IRPIDSource(infraredSensorRight, distancesRight,
						voltagesRight), new PIDOutputClamp(
						new IRPIDOutputRight(), 0.4));

		SmartDashboard.putNumber("P", 4.0);
		SmartDashboard.putNumber("I", 0.01);
		SmartDashboard.putNumber("D", 8.00);
		SmartDashboard.putNumber("Auto Distance", myDriveDistance);
		SmartDashboard.putNumber("Shimmy Max Count", 5);
		SmartDashboard.putNumber("Shimmy Power", 1.0);
		SmartDashboard.putNumber("Shimmy Timeout", 0.05);
		SmartDashboard.putNumber("Shimmy Elevator Start", 3);
		SmartDashboard.putNumber("Elevator Velocity", elevatorVelocity);
		SmartDashboard.putNumber("Saved Wheel Position - Right", rightWheelPosition);
		SmartDashboard.putNumber("Saved Wheel Position - Left", leftWheelPosition);
		autoChooser = new SendableChooser();
		autoChooser.addDefault("Default: 2 Totes", new DefaultAuto(this));
		autoChooser.addObject("Alternate: Move 72 inches", new AlternateAuto(
				this));
		autoChooser.addObject("Empty: Do Nothing", new EmptyAuto());
		SmartDashboard.putData("Autonomous Mode", autoChooser);

		// LiveWindow.addActuator("Talons", "Talon1", talon1);
		// LiveWindow.addActuator("Talons", "Talon2", talon2);
		// LiveWindow.addActuator("Talons", "Talon3", talon3);
		// LiveWindow.addActuator("Talons", "Talon4", talon4);
		LiveWindow.addSensor("Drive Sensors", "Right Encoder", rightEncoder);
		LiveWindow.addSensor("Drive Sensors", "Left Encoder", leftEncoder);
		LiveWindow.addSensor("Drive Sensors", "Gyro", myGyro);
		LiveWindow.addSensor("Infrared", "IR Right", infraredSensorRight);
		LiveWindow.addSensor("Infrared", "IR Left", infraredSensorLeft);
		LiveWindow.addActuator("Elevator", "Victor", victor);
		LiveWindow.addSensor("Elevator", "Encoder", elevatorEncoder);
		LiveWindow.addActuator("PID", "Elevator Controller", elevatorControl);
		LiveWindow.addActuator("PID", "Drive Controller", turningControl);
		disableAllPIDControllers();
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {
		disableAllPIDControllers();
		autoLoopCounter = 0;
		myGyro.reset();
		myDriveDistance = SmartDashboard.getNumber("Auto Distance");
		((AutoMode) autoChooser.getSelected()).autoInit();
		LEDSignboard
				.sendTextMessage("Initializing global domination routine...");
		updateShimmyConstants();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		((AutoMode) autoChooser.getSelected()).autoPeriodic();
	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	public void teleopInit() {
		EVTimer.reset();
		EVTimer.start();
		disableAllPIDControllers();
		myGyro.reset();
		LiveWindow.setEnabled(false);
		// LEDSignboard.sendFile("HPPretty_legit.ppm");
		if (elevatorIndex >= 0) {
			SmartDashboard.putString("Elevator Level",
					levelNames[elevatorIndex]);

		} else {
			SmartDashboard.putString("Elevator Level", "!??");
		}

	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		// turningControl.setPID(SmartDashboard.getNumber("P"),
		// SmartDashboard.getNumber("I")/1000, SmartDashboard.getNumber("D"));
		// rightIRControl.setPID(SmartDashboard.getNumber("P"),
		// SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"));
		// leftDrivingControl.setPID(SmartDashboard.getNumber("P"),
		// SmartDashboard.getNumber("I")/1000, SmartDashboard.getNumber("D"));
		// rightDrivingControl.setPID(SmartDashboard.getNumber("P"),
		// SmartDashboard.getNumber("I")/1000, SmartDashboard.getNumber("D"));

		if (EVTimer.get() - timerLast != 0) {
			elevatorVelocity = (elevatorEncoder.get() - encoderLast)
					/ (EVTimer.get() - timerLast);
			encoderLast = elevatorEncoder.get();
			timerLast = EVTimer.get();
		}
		
		updateShimmyConstants();

		double axis1 = stick.getRawAxis(1);
		double axis3 = stick.getRawAxis(3);

		if (!switchBottom.get() && switchBottomPressed) {
			elevatorEncoder.reset();
			elevatorOffset = 0;
		}
		switchBottomPressed = switchBottom.get();

		if (stick.getButtonCount() > 0) {

			if (stick.getRawButton(1)) {
				if (!leftDrivingControl.isEnable()) {
					leftEncoder.reset();
					rightEncoder.reset();
					leftDrivingControl.setSetpoint(-81);
					leftDrivingControl.enable();
					rightDrivingControl.setSetpoint(81);
					rightDrivingControl.enable();
				}
			} else if (leftDrivingControl.isEnable()) {
				leftDrivingControl.disable();
				rightDrivingControl.disable();

			}
			if (stick.getRawButton(6)) {
				if (!elevatorControl.isEnable()) {
					elevatorControl.enable();
				}
				if (!BAMUpPressed) {
					elevatorUp();
				}
				BAMUpPressed = true;
			} else {
				BAMUpPressed = false;
			}

			if (stick.getRawButton(8)) {
				if (!elevatorControl.isEnable()) {
					elevatorControl.enable();
				}
				if (!BAMDownPressed) {
					elevatorDown();
				}
				BAMDownPressed = true;
			} else {
				BAMDownPressed = false;
			}
			double slowLift = 2.0;
			if (stick.getRawButton(2)) {
				slowLift = 1.0;
			} else {
				slowLift = 2.0;
			}
			// ***** RAW ELEVATOR CONTROL *****
			if (stick.getRawButton(5)) {
				elevatorIndex = -1;
				LEDSignboard.rawCommand("files up");
				if (elevatorControl.isEnable()) {
					elevatorControl.disable();
				}

				if (!switchTop.get()) {
					victor.set(RAW_ELEVATOR_STEP * slowLift);
				} else {
					victor.set(0);
				}
			} else if (stick.getRawButton(7)) {
				elevatorIndex = -1;
				LEDSignboard.rawCommand("files down");
				if (elevatorControl.isEnable()) {
					elevatorControl.disable();

				}
				if (!switchBottom.get()) {
					victor.set(-RAW_ELEVATOR_STEP * slowLift);

				} else {
					victor.set(0);
				}
			} else if (!elevatorControl.isEnable()) {
				victor.set(0);
				LEDSignboard.rawCommand("clear");
			}

			if (stick.getRawButton(10)) {
				elevatorIndex = -1;
				if (!elevatorControl.isEnable()) {
					elevatorControl.enable();
				}
				elevatorControl.setSetpoint(encoderToInches(elevatorEncoder
						.get()));
			}

			if (stick.getPOV() == 180) {
				if (shimmy != ShimmyMode.FINISHED) {
					if (shimmy == ShimmyMode.DISABLED) {
						shimmyInit();
					}

					doShimmy();

					if (shimmy == ShimmyMode.LEFT)
						axis1 = -1 * shimmyPower;
					if (shimmy == ShimmyMode.RIGHT)
						axis3 = -1 * shimmyPower;
					System.out.println(shimmy);
				}

			} else {
				shimmy = ShimmyMode.DISABLED;
			}

			if (stick.getPOV() == 0) {
				leftIRControl.setSetpoint(0.5);
				rightIRControl.setSetpoint(0.5);

				if (!leftIRControl.isEnable())
					LEDSignboard.sendTextMessage("Yarrr!!!");

				leftIRControl.enable();
				rightIRControl.enable();
			} else if (leftIRControl.isEnable()) {
				leftIRControl.disable();
				rightIRControl.disable();
			}

			if (stick.getPOV() == 90) {
				if (!rightStarted) {
					myGyro.reset();
					rightStarted = true;
					turningControl.setSetpoint(90);
					turningControl.enable();
				}
			} else if (rightStarted) {
				turningControl.disable();
				rightStarted = false;
			}

			if (stick.getPOV() == 270) {
				if (!leftStarted) {
					myGyro.reset();
					leftStarted = true;
					turningControl.setSetpoint(-90);
					turningControl.enable();
				}

			} else if (leftStarted) {
				turningControl.disable();
				leftStarted = false;
			}

			if (!leftIRControl.isEnable() && !turningControl.isEnable()
					&& !leftDrivingControl.isEnable()) {
				driveRobot(axis1 * MAX_DRIVE_SPEED, axis3 * MAX_DRIVE_SPEED);
			}
		}

		sendUsefulValues();
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testInit() {
		rightDrivingControl.disable();
		leftDrivingControl.disable();
		rightIRControl.disable();
		leftIRControl.disable();
		myGyro.reset();
		leftEncoder.reset();
		rightEncoder.reset();
		turningControl.disable();
		LiveWindow.setEnabled(false);
	}

	public void testPeriodic() {
		double leftaxis = stick.getRawAxis(1);
		// leftaxis = Math.min(Math.abs(leftaxis), 0.25) *
		// Math.signum(leftaxis);
		double rightaxis = stick.getRawAxis(3);
		// rightaxis = Math.min(Math.abs(rightaxis), 0.25) *
		// Math.signum(rightaxis);

		/*
		 * // **** ML DISABLE **** if (stick.getPOV() != -1) { double target =
		 * 90 - stick.getPOV() + Math.floor(myGyro.getAngle() / 360) * 360;
		 * turningControl.setSetpoint(target);
		 * SmartDashboard.putNumber("Target", target); if
		 * (!turningControl.isEnable()) { turningControl.enable(); }
		 * 
		 * } else { if (turningControl.onTarget()) { turningControl.disable(); }
		 * } // **** ML DISABLE ****
		 */

		if (stick.getRawButton(6)) {
			victor.set(0.5);
		} else if (stick.getRawButton(8)) {
			victor.set(-0.5);
		} else {
			victor.set(0);
		}

		/*
		 * if (stick.getRawButton(5)) { if (!drivingControl.isEnable()) {
		 * rightEncoder.reset(); myGyro.reset();
		 * drivingControl.setSetpoint(-90);
		 * drivingControl.setPID(SmartDashboard.getNumber("P"),
		 * SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"));
		 * drivingControl.enable(); } //
		 * drivingControl.setSetpoint(drivingControl.getSetpoint() + 256);
		 * 
		 * } else if (stick.getRawButton(7)) { drivingControl.disable(); }
		 */

		/*
		 * // ****JPW disable **** if (stick.getRawButton(7)) { driveRobot(-0.3,
		 * -0.3, -1); } else { // **** JPW disable ****
		 */
		if (!leftDrivingControl.isEnable()) {
			talon1.set(rightaxis);
			talon2.set(rightaxis);
			talon3.set(-leftaxis);
			talon4.set(-leftaxis);

		}

	}

	public void elevatorDown() {
		if (elevatorIndex < 0) {
			elevatorIndex = Arrays.binarySearch(levels,
					encoderToInches(elevatorEncoder.get()));
			if (elevatorIndex < 0) { // Elevator index has changed by now. This
										// is not a duplicate.
				elevatorIndex = -1 - elevatorIndex;
			}
		}
		if (elevatorIndex >= 1) {
			elevatorIndex--;
		}
		elevatorControl.setSetpoint(levels[elevatorIndex]);
		LEDSignboard.sendTextMessage(levelNames[elevatorIndex]);
		SmartDashboard.putString("Elevator Level", levelNames[elevatorIndex]);

	}

	public void elevatorUp() {
		if (elevatorIndex < 0) {
			elevatorIndex = Arrays.binarySearch(levels,
					encoderToInches(elevatorEncoder.get()));
			if (elevatorIndex < 0) {
				elevatorIndex = -2 - elevatorIndex;
			}
		}
		if (elevatorIndex < levels.length - 1) {
			elevatorIndex++;
		}
		elevatorControl.setSetpoint(levels[elevatorIndex]);
		LEDSignboard.sendTextMessage(levelNames[elevatorIndex]);
		SmartDashboard.putString("Elevator Level", levelNames[elevatorIndex]);
	}

	public void driveRobot(double left, double right) {
		talon1.set(right);
		talon2.set(right);
		// Values are multiplied by -1 to ensure that the motors on the right
		// spin opposite the motors on the left.
		talon3.set(-left);
		talon4.set(-left);

	}

	public double leftIRToDistance() {
		return voltageToDistance(infraredSensorLeft.getAverageVoltage(),
				voltagesLeft, distancesLeft);
	}

	public double rightIRToDistance() {
		return voltageToDistance(infraredSensorRight.getAverageVoltage(),
				voltagesRight, distancesRight);
	}

	public double voltageToDistance(double v, double[] voltages,
			double[] distances) {
		int i = 0;
		while (i < voltages.length && v < voltages[i]) {
			i++;
		}

		if (i == 0) {
			return -1;
		}
		if (i >= voltages.length) {
			return 40;
		}

		double d2 = distances[i];
		double d1 = distances[i - 1];
		double v2 = voltages[i];
		double v1 = voltages[i - 1];
		return (d2 - d1) * (v - v1) / (v2 - v1) + d1;
	}

	public double inchesToEncoder(double i) {
		return (i + elevatorOffset) * ENCODER_RESOLUTION
				/ (2 * Math.PI * R * GEAR_RATIO);
	}

	public double encoderToInches(double e) {
		return (e * 2 * Math.PI * R * GEAR_RATIO / (ENCODER_RESOLUTION))
				+ elevatorOffset;
	}

	public double driveInchesToEncoder(double i) {
		return i * ENCODER_RESOLUTION / (2 * Math.PI * DRIVE_R * DRIVE_RATIO);
	}

	public double driveEncoderToInches(double e) {
		return e * 2 * Math.PI * DRIVE_R * DRIVE_RATIO / (ENCODER_RESOLUTION);
	}

	public void shimmyInit() {
		LEDSignboard.sendTextMessage("Shimmy me timbers!!!!!!");
		shimmy = ShimmyMode.LEFT;
		shimmyCount = 0;
		shimmyTime.start();
	}

	public ShimmyMode doShimmy() {
		if (shimmyTime.hasPeriodPassed(shimmyTimeout)) {
			shimmyCount += 1;
			if (shimmyCount > shimmyMaxCount)
				shimmy = ShimmyMode.FINISHED;
			else {
				if (shimmyCount == shimmyElevatorStart) {
					elevatorUp();
					elevatorControl.enable();
				}
				if (shimmy == ShimmyMode.LEFT)
					shimmy = ShimmyMode.RIGHT;
				else if (shimmy == ShimmyMode.RIGHT)
					shimmy = ShimmyMode.LEFT;
			}
		}

		return shimmy;
	}

	public void disableAllPIDControllers() {
		rightDrivingControl.disable();
		leftDrivingControl.disable();
		turningControl.disable();
		elevatorControl.disable();
		slowElevatorControl.disable();
		rightIRControl.disable();
		leftIRControl.disable();
	}

	public void updateShimmyConstants() {
		shimmyMaxCount = (int) SmartDashboard.getNumber("Shimmy Max Count", 8);
		shimmyPower = SmartDashboard.getNumber("Shimmy Power", 1.0);
		shimmyTimeout = SmartDashboard.getNumber("Shimmy Timeout", 0.08);
		shimmyElevatorStart = (int) SmartDashboard.getNumber(
				"Shimmy Elevator Start", 3);
	}

	public void saveWheelPositions() {
		rightWheelPosition = rightEncoder.get();
		leftWheelPosition = leftEncoder.get();
	}

	public void returnToWheelPositions() {
		rightDrivingControl.setSetpoint(rightWheelPosition);
		leftDrivingControl.setSetpoint(leftWheelPosition);
	}

	public void sendUsefulValues() {
		// SmartDashboard.putNumber("Talon1 Motors", talon1.get());
		// SmartDashboard.putNumber("Talon3 Motors", talon3.get());
		SmartDashboard.putNumber("Gyro Value", myGyro.getAngle());
		SmartDashboard.putNumber("Elevator Encoder Value",
				(double) elevatorEncoder.get());
		SmartDashboard.putNumber("Elevator Inches",
				(double) encoderToInches(elevatorEncoder.get()));
		// SmartDashboard.putNumber("ELEVATOR SETPOINT",
		// elevatorControl.getSetpoint());
		// SmartDashboard.putNumber("ELEVATOR OUTPUT", elevatorControl.get());
		SmartDashboard.putNumber(
				"Left Infrared Distance",
				voltageToDistance(infraredSensorLeft.getVoltage(),
						voltagesLeft, distancesLeft));
		SmartDashboard.putNumber(
				"Right Infrared Distance",
				voltageToDistance(infraredSensorRight.getVoltage(),
						voltagesRight, distancesRight));
		SmartDashboard.putNumber("Left IR V",
				infraredSensorLeft.getAverageVoltage());
		SmartDashboard.putNumber("Right IR V",
				infraredSensorRight.getAverageVoltage());

		SmartDashboard.putNumber("Right Encoder Value", rightEncoder.get());
		SmartDashboard.putNumber("Right Encoder Inches",
				driveEncoderToInches(rightEncoder.get()));
		SmartDashboard.putNumber("Left Encoder Value", leftEncoder.get());
		SmartDashboard.putNumber("Left Encoder Inches",
				driveEncoderToInches(leftEncoder.get()));
		SmartDashboard.putBoolean("Top switch", switchTop.get());
		SmartDashboard.putBoolean("Bottom switch", switchBottom.get());
	}

	public class GyroPIDOutput implements PIDOutput {

		@Override
		public void pidWrite(double output) {
			SmartDashboard.putNumber("Gyro PIDOutput", output);
			output /= 100;
			talon1.set(output);
			talon2.set(output);
			talon3.set(output);
			talon4.set(output);
		}
	}

	public class LeftDrivePIDOutput implements PIDOutput {

		@Override
		public void pidWrite(double output) {
			SmartDashboard.putNumber("Left Drive PIDOutput", output);
			talon1.set(-output);
			talon2.set(-output);
		}

	}

	public class RightDrivePIDOutput implements PIDOutput {

		@Override
		public void pidWrite(double output) {
			SmartDashboard.putNumber("Right Drive PIDOutput", output);
			talon3.set(-output);
			talon4.set(-output);
		}

	}

	public class PIDOutputClamp implements PIDOutput {
		PIDOutput clampedOutput;
		double clampValue;

		public PIDOutputClamp(PIDOutput clampedOutput, double clampValue) {
			this.clampedOutput = clampedOutput;
			this.clampValue = clampValue;

		}

		public void pidWrite(double output) {
			if (Math.abs(output) > clampValue) {
				output = Math.signum(output) * clampValue;
			}
			this.clampedOutput.pidWrite(output);
		}
	}

	public class SwitchOverride implements PIDOutput {
		PIDOutput safeOutput;

		public SwitchOverride(PIDOutput safeOutput) {
			this.safeOutput = safeOutput;
		}

		@Override
		public void pidWrite(double output) {
			if (output > 0) { // +
				if (!switchTop.get()) { // not pressed
					safeOutput.pidWrite(output);
				} else {
					safeOutput.pidWrite(0);
				}
			} else {// -
				if (!switchBottom.get()) {// notpressed
					safeOutput.pidWrite(output);
				} else {
					safeOutput.pidWrite(0);
				}
			}
		}
	}

	public class IRPIDOutputLeft implements PIDOutput {
		@Override
		public void pidWrite(double output) {
			SmartDashboard.putNumber("Left IR out", output);

			talon3.set(-output);
			talon4.set(-output);

		}
	}

	public class IRPIDOutputRight implements PIDOutput {
		@Override
		public void pidWrite(double output) {
			SmartDashboard.putNumber("Right IR out", output);
			talon1.set(output);
			talon2.set(output);

		}
	}

	public class InchesEncoder implements PIDSource {
		PIDSource mySource;

		public InchesEncoder(PIDSource mySource) {
			this.mySource = mySource;
		}

		@Override
		public double pidGet() {
			return encoderToInches(mySource.pidGet());
		}
	}

	public class DriveInchesEncoder implements PIDSource {
		PIDSource mySource;

		public DriveInchesEncoder(PIDSource mySource) {
			this.mySource = mySource;
		}

		@Override
		public double pidGet() {
			return driveEncoderToInches(mySource.pidGet());
		}

	}

	public class IRPIDSource implements PIDSource {
		PIDSource mySource;
		double[] distances;
		double[] voltages;

		public IRPIDSource(PIDSource mySource, double[] distances,
				double[] voltages) {

			this.distances = distances;
			this.voltages = voltages;
			this.mySource = mySource;

		}

		public double pidGet() {
			return voltageToDistance(mySource.pidGet(), voltages, distances);
		}
	}
}
