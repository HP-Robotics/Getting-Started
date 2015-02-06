package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
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
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
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
	static final int MAXIMUM_LEVEL = 78; // inches
	static final int MINIMUM_LEVEL = 0; // inches
	static final double RAW_ELEVATOR_STEP = 0.05; // inches per update
	static final double TURN_SPEED = 0.20; // talon motor input

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
	AnalogInput infraredSensor; // used for testing the IR sensor
	AnalogInput infraredSensor2;// TODO assign one for left and one for right
								// and then use them for auto stuff and other
								// stuff
	Gyro myGyro;
	PIDController elevatorControl;
	PIDController turningControl;

	// TODO add PID Controllers for special approach tote mode.

	Command autoCommand;
	SendableChooser autoChooser;

	double motorScale = 0.97;
	double currentAngle = 0;
	double gyroResetAngle = -1;
	double globalAngleDesired = -1;

	final static double[] voltages = new double[] { 2.36, 2.18, 2.02, 1.87,
			1.76, 1.66, 1.57, 1.49, 1.42, 1.36, 1.30, 1.24, 1.20, 1.15, 1.11,
			1.07, 1.03, 0.99, 0.96, 0.93, 0.91, 0.89, 0.86, 0.83, 0.82, 0.81,
			0.79, 0.77, 0.76, 0.75, 0.73 };
	final static double[] distances = new double[] { 10, 11, 12, 13, 14, 15,
			16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
			33, 34, 35, 36, 37, 38, 39, 40 };

	boolean BAMUpPressed = false;
	boolean BAMDownPressed = false;
	boolean rawElevatorPressed = false;
	boolean StraightMode = false;
	boolean rawElevator = false;
	boolean messagePrinted = false;
	boolean autoChosen = false;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		// myRobot = new RobotDrive(0,1);
		stick = new Joystick(0);
		talon1 = new Talon(1);
		talon2 = new Talon(2);
		talon3 = new Talon(3);
		talon4 = new Talon(4);
		victor = new VictorSP(0);
		elevatorEncoder = new Encoder(4, 5, true, EncodingType.k4X);
		rightEncoder = new Encoder(0, 1, true, EncodingType.k4X);
		leftEncoder = new Encoder(2, 3, true, EncodingType.k4X);
		infraredSensor = new AnalogInput(1);
		infraredSensor2 = new AnalogInput(2);
		myGyro = new Gyro(0);
		switchTop = new DigitalInput(6);
		switchBottom = new DigitalInput(7);
		elevatorControl = new PIDController(0.1, 0.0, 0.0, elevatorEncoder,
				new PIDOutputClamp(victor, 0.5));
		turningControl = new PIDController(1.0/180.0, 0.0, 0.0, myGyro,
				new PIDOutputClamp(new GyroPIDOutput(), 0.25));
		turningControl.setPercentTolerance(2);
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
		LiveWindow.addSensor("Infrared", "IR1", infraredSensor);
		LiveWindow.addSensor("Infrared", "IR2", infraredSensor2);
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
		double infrared = infraredSensor.getAverageVoltage();
		System.out.printf("infrared : %.4f distance: %.4f cm\n", infrared,
				voltageToDistance(infrared));

		double axis1 = stick.getRawAxis(1);
		double axis3 = stick.getRawAxis(3);
		// TODO make sure that pov angle makes sense

		driveRobot(axis1, axis3, stick.getPOV());
		if (!rawElevator) {
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
		} else {
			if (stick.getRawButton(6)) {

				elevatorControl.setSetpoint(inchesToEncoder(Math.min(
						MAXIMUM_LEVEL,
						encoderToInches(elevatorControl.getSetpoint())
								+ RAW_ELEVATOR_STEP)));

			}

			if (stick.getRawButton(8)) {

				elevatorControl.setSetpoint(inchesToEncoder(Math.max(
						MINIMUM_LEVEL,
						encoderToInches(elevatorControl.getSetpoint())
								- RAW_ELEVATOR_STEP)));

			}
		}

		// TODO Make sure raw elevator mode works with the original mode
		// (including setting levels correctly).
		// We also need to make sure that you can't move the elevator too high
		// or too low.

		// While driving the elevator for testing, this is useful. Consider only
		// using raw mode for small corrections.
		if (stick.getRawButton(3)) {
			if (!rawElevatorPressed) {
				toggleRawElevator();
			}
			rawElevatorPressed = true;
		} else {
			rawElevatorPressed = false;
		}

		SmartDashboard.putNumber("Gyro Value", myGyro.getAngle());
		SmartDashboard.putNumber("Elevator Encoder Value",
				(double) elevatorEncoder.get());
		SmartDashboard.putNumber("Infrared Value",
				voltageToDistance(infraredSensor.getVoltage()));
		SmartDashboard.putNumber("Right Encoder Value", rightEncoder.get());
		SmartDashboard.putNumber("Left Encoder Value", leftEncoder.get());

		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			System.out.println("HELP!");
		}

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
		SmartDashboard.putNumber("elevatorEncoder", elevatorEncoder.get());
		double leftaxis = stick.getRawAxis(1);
		leftaxis = Math.min(Math.abs(leftaxis), 0.25)*Math.signum(leftaxis);
		double rightaxis = stick.getRawAxis(3);
		rightaxis = Math.min(Math.abs(rightaxis), 0.25)*Math.signum(rightaxis);
		SmartDashboard.putNumber("gyro", myGyro.getAngle());

		/*if (stick.getPOV() != -1) {
			double target = 90 - stick.getPOV()
					+ Math.floor(myGyro.getAngle() / 360) * 360;
			turningControl.setSetpoint(target);
			SmartDashboard.putNumber("Target", target);
			if (!turningControl.isEnable()) {
				turningControl.enable();
			}

		} else {
			if (turningControl.onTarget()) {
				turningControl.disable();
			}
		}*/
		stick.
		
		if (stick.getRawButton(6)) {
			victor.set(0.5);

		} else if (stick.getRawButton(8)) {
			victor.set(-0.5);
		} else {
			victor.set(0);
		}
		if (stick.getRawButton(5)) {

			leftaxis = -0.3;
			rightaxis = -0.3;
		}

		if (stick.getRawButton(7)) {
			driveRobot(-0.3, -0.3, -1);
		} else {

			if (!turningControl.isEnable()) {
				talon1.set(rightaxis);
				talon2.set(rightaxis);
				talon3.set(-leftaxis);
				talon4.set(-leftaxis);
			}

		}

	}

	public void driveRobot(double left, double right, double angle) {
		if (angle == -1 && globalAngleDesired < 0) {
			if (left > 0) {
				left = Math.floor(8 * left) / 8;
			} else {
				left = Math.ceil(8 * left) / 8;
			}
			if (right > 0) {
				right = Math.floor(8 * right) / 8;
			} else {
				right = Math.ceil(8 * right) / 8;
			}

			if (Math.abs(left - right) <= 0.125) {
				// we're going straight and we're going to check if
				// this is the beginning of our straight section
				if (StraightMode == false) {
					gyroResetAngle = myGyro.getAngle();
					currentAngle = gyroResetAngle;
					StraightMode = true;
				}
				if (myGyro.getAngle() > gyroResetAngle + 2) {
					// The robot is veering to the right
					if (myGyro.getAngle() > currentAngle) {
						currentAngle = myGyro.getAngle();
						motorScale -= 0.01; // reduce speed of motor
					}
					if (right < -0.01) {
						left *= motorScale;
					} else {
						right *= motorScale;
					}
				} else if (myGyro.getAngle() < gyroResetAngle - 2) {
					// The robot is veering to the left
					if (myGyro.getAngle() < currentAngle) {
						currentAngle = myGyro.getAngle();
						motorScale -= 0.01;
					}
					if (right < -0.01) {
						right *= motorScale;
					} else {
						left *= motorScale;
					}
				} else {
					motorScale = 1;
				}
			} else {
				StraightMode = false;
				gyroResetAngle = -1;

			}
		} else {
			// TODO we might have to update this to handle degree wrap-around or
			// however the gyro is orientated

			if (angle > -1) {
				globalAngleDesired = angle;
			}
			if (globalAngleDesired - myGyro.getAngle() > 0.01) {
				left = -TURN_SPEED;
				right = TURN_SPEED;
			}

			else if (globalAngleDesired - myGyro.getAngle() < -0.01) {
				left = TURN_SPEED;
				right = -TURN_SPEED;
			}

			else {
				left = 0;
				right = 0;
				globalAngleDesired = -1;
			}

		}

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

	public void toggleRawElevator() {
		if (rawElevator) {
			rawElevator = false;
		} else {
			rawElevator = true;
		}
	}

	public double voltageToDistance(double v) {
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
		return i * ENCODER_RESOLUTION / (2 * Math.PI * R);
	}

	public double encoderToInches(double e) {
		return e * 2 * Math.PI * R / ENCODER_RESOLUTION;
	}

	public class GyroPIDOutput implements PIDOutput {

		@Override
		public void pidWrite(double output) {
			SmartDashboard.putNumber("PIDOutput", output);

			talon1.set(output);
			talon2.set(output);
			talon3.set(output);
			talon4.set(output);

			// TODO Auto-generated method stub

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
}
