package org.usfirst.frc.team2823.robot;

import org.usfirst.frc.team2823.robot.Robot.ShimmyMode;

import edu.wpi.first.wpilibj.Timer;

// total 1: 7.628
// total 2: 
public class DefaultAuto implements AutoMode {
	Robot myBot;

	double stageTimeouts[] = { 1.0, 2.0, 2.0, 2.0, 1.5, 0.5, 2.0, 1.25, 1.0, 1.25, 0.5, }; // lift back
																// turn drive
																// lift turn
																// align
													 		// shimmy/lift
	int stageCounts[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	boolean stageTimeoutFailure[] = { false, false, false, false, false, false, false, false, false, false, false };
	int ontarget;
	int stage = 0;
	Timer tick;
	Timer stageTime;
	int stageLast = 0;

	// *****MOVE UP OR DOWN AS NECESSARY ONCE RECALIBRATED*****
	//
	// *****CODE IS CURRENTLY IDENTICAL TO ALTERNATEAUTO CODE*****
	// pick up tote
	// back up 34.11514859600516 in.
	// turn right 30 degrees
	// drive forward 13.525 in.
	// pick up can
	// turn left 30 degrees
	// back up 84.59784499017937 in.
	// turn right 90 degrees
	// drop tote
	// move back 24.5 ft.
	// drop can
	// move back enough

	public DefaultAuto(Robot myBot) {
		this.myBot = myBot;
	}

	public void autoInit() {
		tick = new Timer();
		stageTime = new Timer();
		tick.reset();
		tick.start();
		stage = 0;

		for (int i = 0; i < stageCounts.length; i++)
			stageCounts[i] = 0;

		System.out
				.println("Hey, you chose the alternate autonomous mode. Good job!");
	}

	public void autoPeriodic() {

		if (stage < 0 || stage >= stageCounts.length)
			return;

		if (stageCounts[stage] == 0)
			System.out.println(stage);

		if (tick.get() > stageTimeouts[stage]) {

			System.out.printf("stage %d timed out\n", stage);

			myBot.disableAllPIDControllers();

			if (stageTimeoutFailure[stage]) {
				System.out.printf("stage %d failed!\n", stage);
				stage = -1;
				return;
			}

			System.out.println("Continuing anyway...");
			System.out.printf("%f Exiting Stage %d\n", tick.get(), stage);
			tick.reset();
			stage++;
			return;
		}

		// pick up a tote
		if (stage == 0) {
			if (stageCounts[stage] == 0) {
				myBot.elevatorControl.enable();
				myBot.elevatorUp();
				myBot.elevatorUp();
				System.out.println("stage 0 succeeded!");
				LEDSignboard.sendTextMessage("TOTE ");

				
			}
		}

		// turn right 90 degrees
		if (stage == 1) {
			if (stageCounts[stage] == 0) {
				myBot.myGyro.reset();
				myBot.turningControl.setSetpoint(90);
				myBot.turningControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("TURN ");

			}

			if (Math.abs(myBot.myGyro.getAngle() - 90) < 6)
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > 10) {
				System.out.printf("%f Exiting Stage %d\n", tick.get(), stage);
				tick.reset();
				myBot.turningControl.disable();
				System.out.println("stage 1 succeeded!");
				tick.reset();
				stage++;
				return;
			}

		}

		// drive forward 76 inches
		if (stage == 2) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(-76);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(76);
				myBot.rightDrivingControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("GO! ");

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - 76) < 4) && (Math.abs(l - (-76)) < 4))
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > 10) {
				tick.reset();
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				System.out.println("stage 2 succeeded!");
				System.out.printf("%f Exiting Stage %d\n", tick.get(), stage);
				tick.reset();
				stage++;
				return;
			}

		}

		// turn left 90 degrees
		if (stage == 3) {
			if (stageCounts[stage] == 0) {
				myBot.myGyro.reset();
				myBot.turningControl.setSetpoint(-90);
				myBot.turningControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("PI OVER TWOOOOO! ");

			}

			if (Math.abs(myBot.myGyro.getAngle() - (-90)) < 6)
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > 10) {
				System.out.printf("%f Exiting Stage %d\n", tick.get(), stage);
				tick.reset();
				myBot.turningControl.disable();
				System.out.println("stage 3 succeeded!");
				tick.reset();
				stage++;
				return;
			}

		}

		// approach the tote
		if (stage == 4) {
			if (stageCounts[stage] == 0) {
				myBot.leftIRControl.setSetpoint(0.25);
				myBot.rightIRControl.setSetpoint(0.25);
				myBot.leftIRControl.enable();
				myBot.rightIRControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("*APPROACHES CAUTIOUSLY* ");

			}

			if ((Math
					.abs(myBot.infraredSensorLeft.getAverageVoltage() - (1.70)) < 0.3)
					&& (Math.abs(myBot.infraredSensorRight.getAverageVoltage() - 1.70) < 0.3))
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > 10) {
				tick.reset();
				myBot.rightIRControl.disable();
				myBot.leftIRControl.disable();
				System.out.println("stage 4 succeeded!");
				System.out.printf("%f Exiting Stage %d\n", tick.get(), stage);
				tick.reset();
				stage++;
				return;
			}

		}

		// pick up tote #2
		if (stage == 5) {
			if (stageCounts[stage] == 0) {
				myBot.shimmyInit();
				
			}

			if (myBot.doShimmy() == ShimmyMode.FINISHED) {
				System.out.println("stage 5 succeeded!");
				System.out.printf("%f Exiting Stage %d\n", tick.get(), stage);
				tick.reset();
				stage++;
			}
		}
		// drive forward 83 inches
		if (stage == 6) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(-83);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(83);
				myBot.rightDrivingControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("WALK THE PLANK!");

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - 83) < 4) && (Math.abs(l - (-83)) < 4))
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > 10) {
				tick.reset();
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				System.out.println("stage 6 succeeded!");
				System.out.printf("%f Exiting Stage %d\n", tick.get(), stage);
				tick.reset();
				stage++;
				return;
			}

		}
		// put down totes
		if (stage == 7) {
			if (stageCounts[stage] == 0) {
				myBot.elevatorControl.enable();
				myBot.elevatorDown();
				myBot.elevatorDown();
				System.out.println("stage 7 succeeded!");
				LEDSignboard.sendTextMessage("ANCHORS AWAY! ");

			}
		}
		
		// drive forward 24 inches
		if (stage == 8) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(-24);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(24);
				myBot.rightDrivingControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("YARRRRRR! ");

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - 24) < 4) && (Math.abs(l - (-24)) < 4))
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > 10) {
				tick.reset();
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				System.out.println("stage 8 succeeded!");
				System.out.printf("%f Exiting Stage %d\n", tick.get(), stage);
				tick.reset();
				stage++;
				return;
			}

		}
		
		// move the elevator up 2 levels
		if (stage == 9) {
			if (stageCounts[stage] == 0) {
				myBot.elevatorControl.enable();
				myBot.elevatorUp();
				myBot.elevatorUp();
				System.out.println("stage 9 succeeded!");
			}
		}
		
		// back up 2 pi inches
		if (stage == 10) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(2*Math.PI);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(-2*Math.PI);
				myBot.rightDrivingControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("F ARRRRR Sea! ");

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - (-2*Math.PI)) < 4) && (Math.abs(l - (2*Math.PI)) < 4))
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > 10) {
				tick.reset();
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				System.out.println("stage 10 succeeded!");
				System.out.printf("%f Exiting Stage %d\n", tick.get(), stage);
				tick.reset();
				stage++;
				return;
			}

		}
		
		if(stage != stageLast)
		{
			stageTime.stop();
			System.out.println("Stage time: " + stageTime.get());
			stageTime.reset();
			stageTime.start();
		}
		
		stageLast = stage;
		stageCounts[stage]++;
	}

}
