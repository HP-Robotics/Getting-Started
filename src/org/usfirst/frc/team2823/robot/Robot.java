package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Talon;
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
	static final double R = 2; // inches
	static final double LEVEL_HEIGHT = 14; // inches
	static final double ENCODER_RESOLUTION = 2048; // counts per revolution
	static final double GEAR_RATIO = (1.0 / 50.0) * (15.0 / 22.0); // gearbox is
																	// 1:50,
																	// chain is
																	// 22:15
	static final int MAXIMUM_LEVEL = 78; // inches
	static final int MINIMUM_LEVEL = 0; // inches
	static final double RAW_ELEVATOR_STEP = 0.05; // inches per update
	static final double TURN_SPEED = 0.20; // talon motor input
	static final double DELTA_UP = 0.01; // PID output speed increase slow thing
	static final double DELTA_DOWN = -0.01; // other thing
	static final double MAX_DRIVE_SPEED = 0.6; // talon motor input
	static final double MANUAL_INCREASE = 0.5; // amount (in inches) added to
												// setpoint of elevator for
												// every
												// iteration button is pressed
												// for manual elevator drive

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
	PIDController turningControl;
	PIDController drivingControl;

	// TODO add PID Controllers for special approach tote mode.

	Command autoCommand;
	SendableChooser autoChooser;

	double motorScale = 0.97;
	double currentAngle = 0;
	double gyroResetAngle = -1;
	double globalAngleDesired = -1;

	final static double[] voltagesLeft = new double[] { 2.36, 2.18, 2.02, 1.87,
			1.76, 1.66, 1.57, 1.49, 1.42, 1.36, 1.30, 1.24, 1.20, 1.15, 1.11,
			1.07, 1.03, 0.99, 0.96, 0.93, 0.91, 0.89, 0.86, 0.83, 0.82, 0.81,
			0.79, 0.77, 0.76, 0.75, 0.73 };
	final static double[] distancesLeft = new double[] { 10, 11, 12, 13, 14,
			15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
			32, 33, 34, 35, 36, 37, 38, 39, 40 };

	final static double[] voltagesRight = new double[] {};
	final static double[] distancesRight = new double[] { 0 };

	boolean BAMUpPressed = false;
	boolean BAMDownPressed = false;
	boolean elevatorResetPressed = false;
	boolean StraightMode = false;
	boolean messagePrinted = false;
	boolean autoChosen = false;

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
		elevatorControl = new PIDController(0.1, 0.0, 0.0, elevatorEncoder,
				new SwitchOverride(new PIDOutputClamp(victor, 0.5)));
		turningControl = new PIDController(1.0 / 180.0, 0.0, 0.0, myGyro,
				new PIDOutputClamp(new GyroPIDOutput(), 0.25));
		turningControl.setPercentTolerance(2);
		drivingControl = new PIDController(0.1, 0.0, 0.0, myGyro,
				new PIDOutputClamp(new EncoderPIDOutput(), 50.0));
		drivingControl.setAbsoluteTolerance(100);
		drivingControl.setOutputRange(-10000, 10000);

		SmartDashboard.putNumber("P", 0.1);
		SmartDashboard.putNumber("I", 0.0);
		SmartDashboard.putNumber("D", 0.0);
		autoChooser = new SendableChooser();
		autoChooser.addDefault("Default", new DefaultAuto(this));
		autoChooser.addObject("Alternate", new AlternateAuto(this));
		SmartDashboard.putData("Auto Mode", autoChooser);

		LiveWindow.addActuator("Talons", "Talon1", talon1);
		LiveWindow.addActuator("Talons", "Talon2", talon2);
		LiveWindow.addActuator("Talons", "Talon3", talon3);
		LiveWindow.addActuator("Talons", "Talon4", talon4);
		LiveWindow.addSensor("Drive Sensors", "Right Encoder", rightEncoder);
		LiveWindow.addSensor("Drive Sensors", "Left Encoder", leftEncoder);
		LiveWindow.addSensor("Drive Sensors", "Gyro", myGyro);
		LiveWindow.addSensor("Infrared", "IR Right", infraredSensorRight);
		LiveWindow.addSensor("Infrared", "IR Left", infraredSensorLeft);
		LiveWindow.addActuator("Elevator", "Victor", victor);
		LiveWindow.addSensor("Elevator", "Encoder", elevatorEncoder);
		LiveWindow.addActuator("PID", "Elevator Controller", elevatorControl);
		LiveWindow.addActuator("PID", "Drive Controller", turningControl);
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {
		autoLoopCounter = 0;
		myGyro.reset();

		((AutoMode) autoChooser.getSelected()).autoInit();
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
		elevatorEncoder.reset();
		myGyro.reset();
		LiveWindow.setEnabled(false);

	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		double axis1 = stick.getRawAxis(1);
		double axis3 = stick.getRawAxis(3);

		// **** JPW disable ****
		if (stick.getButtonCount() > 0) {
			driveRobot(axis1 * MAX_DRIVE_SPEED, axis3 * MAX_DRIVE_SPEED);

			if (stick.getRawButton(6)) {
				if (!BAMUpPressed) {
					elevatorControl.setSetpoint(inchesToEncoder(Math.min(
							MAXIMUM_LEVEL,
							encoderToInches(elevatorControl.getSetpoint())
									+ LEVEL_HEIGHT)));
				}
				BAMUpPressed = true;
			} else {
				BAMUpPressed = false;
			}

			if (stick.getRawButton(8)) {
				if (!BAMDownPressed) {
					elevatorControl.setSetpoint(inchesToEncoder(Math.max(
							MINIMUM_LEVEL,
							encoderToInches(elevatorControl.getSetpoint())
									- LEVEL_HEIGHT)));
				}
				BAMDownPressed = true;
			} else {
				BAMDownPressed = false;
			}

			// ***** RAW ELEVATOR CONTROL *****
			if (stick.getRawButton(5)) {
				if (elevatorControl.isEnable()) {
					elevatorControl.disable();
				}
				if (switchTop.get()) {
					victor.set(RAW_ELEVATOR_STEP);
				} else {
					victor.set(0);
				}
			} else if (stick.getRawButton(7)) {
				if (elevatorControl.isEnable()) {
					elevatorControl.disable();
				}
				if (switchBottom.get()) {
					victor.set(-RAW_ELEVATOR_STEP);
				} else {
					victor.set(0);
				}
			} else if (!elevatorControl.isEnable()) {
				victor.set(0);
			}

			if (stick.getRawButton(10)) {
				if (!elevatorControl.isEnable()) {
					elevatorControl.enable();
				}
				elevatorControl.setSetpoint(inchesToEncoder(Math
						.round(encoderToInches(elevatorEncoder.get())
								/ LEVEL_HEIGHT)
						* LEVEL_HEIGHT));
			}
		}
		SmartDashboard.putNumber("Gyro Value", myGyro.getAngle());
		SmartDashboard.putNumber("Elevator Encoder Value",
				(double) elevatorEncoder.get());
		SmartDashboard.putNumber(
				"Left Infrared Value",
				voltageToDistance(infraredSensorLeft.getVoltage(),
						voltagesLeft, distancesLeft));
		SmartDashboard.putNumber(
				"Left Infrared Value",
				voltageToDistance(infraredSensorRight.getVoltage(),
						voltagesRight, distancesRight));
		SmartDashboard.putNumber("Right Encoder Value", rightEncoder.get());
		SmartDashboard.putNumber("Left Encoder Value", leftEncoder.get());
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			System.out.println("HELP!");
		}
		// **** JPW ****
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testInit() {
		myGyro.reset();
		elevatorEncoder.reset();
		leftEncoder.reset();
		rightEncoder.reset();
		turningControl.disable();
		LiveWindow.setEnabled(false);
	}

	public void testPeriodic() {
		SmartDashboard.putNumber("leftEncoder", leftEncoder.get());
		SmartDashboard.putNumber("rightEncoder", rightEncoder.get());
		SmartDashboard.putNumber("Setpoint", drivingControl.getSetpoint());
		SmartDashboard.putBoolean("In tolerance", drivingControl.onTarget());
		SmartDashboard.putNumber("elevatorEncoder", elevatorEncoder.get());
		SmartDashboard.putNumber("gyro angle", myGyro.getAngle());
		SmartDashboard.putNumber("gyro rate", myGyro.getRate());
		SmartDashboard.putNumber("Gyro Input", gyroInput.getVoltage());

		SmartDashboard.putNumber("left IR cm", leftIRToDistance());
		SmartDashboard.putNumber("right IR cm", rightIRToDistance());
		SmartDashboard.putNumber("left IR V",
				infraredSensorLeft.getAverageVoltage());
		SmartDashboard.putNumber("right IR V",
				infraredSensorRight.getAverageVoltage());

		SmartDashboard.putBoolean("top switch", switchTop.get());

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

		if (stick.getRawButton(5)) {
			if (!drivingControl.isEnable()) {
				rightEncoder.reset();
				myGyro.reset();
				drivingControl.setSetpoint(-90);
				drivingControl.setPID(SmartDashboard.getNumber("P"),
						SmartDashboard.getNumber("I"),
						SmartDashboard.getNumber("D"));
				drivingControl.enable();
			}
			// drivingControl.setSetpoint(drivingControl.getSetpoint() + 256);

		} else if (stick.getRawButton(7)) {
			drivingControl.disable();
		}

		/*
		 * // ****JPW disable **** if (stick.getRawButton(7)) { driveRobot(-0.3,
		 * -0.3, -1); } else { // **** JPW disable ****
		 */
		if (!drivingControl.isEnable()) {
			talon1.set(rightaxis);
			talon2.set(rightaxis);
			talon3.set(-leftaxis);
			talon4.set(-leftaxis);

			// **** JPW disable ****
		}
		/*
		 * } // **** JPW disable ****
		 */

	}

	public void driveRobot(double left, double right) {

		SmartDashboard.putNumber("Left Motors", left);
		SmartDashboard.putNumber("Right Motors", right);
		SmartDashboard.putNumber("Global Angle Desired", globalAngleDesired);
		SmartDashboard.putNumber("Straight Mode Heading", gyroResetAngle);
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

		if (i == 0 || i >= voltages.length) {
			return Double.NaN;
		}

		double d2 = distances[i];
		double d1 = distances[i - 1];
		double v2 = voltages[i];
		double v1 = voltages[i - 1];
		return (d2 - d1) * (v - v1) / (v2 - v1) + d1;
	}

	public double inchesToEncoder(double i) {
		return i * (ENCODER_RESOLUTION * GEAR_RATIO) / (2 * Math.PI * R);
	}

	public double encoderToInches(double e) {
		return e * 2 * Math.PI * R / (ENCODER_RESOLUTION * GEAR_RATIO);
	}

	public class GyroPIDOutput implements PIDOutput {

		@Override
		public void pidWrite(double output) {
			SmartDashboard.putNumber("Gyro PIDOutput", output);

			talon1.set(output);
			talon2.set(output);
			talon3.set(output);
			talon4.set(output);
		}
	}

	public class EncoderPIDOutput implements PIDOutput {

		@Override
		public void pidWrite(double output) {
			SmartDashboard.putNumber("Encoder PIDOutput", output);

			// output /= 10000;

			/*
			 * if(output > talon3.get()) { output = Math.min(output -
			 * talon3.get(), DELTA_UP) + talon3.get(); } else if(output <
			 * talon3.get()) { output = Math.max(output - talon3.get(),
			 * DELTA_DOWN) + talon3.get(); }
			 */

			output /= -100;
			talon1.set(-output);
			talon2.set(-output);
			talon3.set(-.75 * output);
			talon4.set(-.75 * output);
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
				if (switchTop.get()) { // not pressed
					safeOutput.pidWrite(output);
				} else {
					safeOutput.pidWrite(0);
				}
			} else {// -
				if (switchBottom.get()) {// notpressed
					safeOutput.pidWrite(output);
				} else {
					safeOutput.pidWrite(0);
				}
			}
		}
	}
}
