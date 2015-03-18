package org.usfirst.frc.team2823.robot;

import org.usfirst.frc.team2823.robot.Robot.ShimmyMode;

import edu.wpi.first.wpilibj.Timer;

// total 1: 7.628
// total 2: 
public class DefaultAuto implements AutoMode {
	Robot myBot;

	double stageTimeouts[] = { 0.2, 2.0, 2.0, 2.0, 1.5, 0.2, 2.0, 2.0, 2.0, 1.5, 3.5, 1.5, 0.5 }; // total 20.9
	//lift, turn, drive 78, turn, approach, lift, turn, drive 78, turn, approach, drive 144, drop, drive back pi
	int stageCounts[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	boolean stageTimeoutFailure[] = { false, false, false, false, false, false,
			false, false, false, false, false, false, false };
	int ontarget;
	int stage = 0;
	Timer tick;
	Timer stageTime;
	int stageLast = 0;
	boolean runFirstHalf = true;
	boolean runSecondHalf = true;
	final static int SECOND_HALF_START = 5;

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
		
		if(runSecondHalf)
		{
			stage = SECOND_HALF_START;
		}
		if(runFirstHalf)
		{
			stage = 0;
		}

		for (int i = 0; i < stageCounts.length; i++)
			stageCounts[i] = 0;

		System.out.println("Hey, you chose the alternate autonomous mode. Good job!");
		stageTime.reset();
		stageTime.start();
	}

	public void autoPeriodic() {

		if (stage < 0 || stage >= stageCounts.length)
			return;
		
		if((runFirstHalf == true) && (runSecondHalf == false) && (stage >= SECOND_HALF_START))
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
			myBot.elevatorControl.enable();
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
				myBot.elevatorControl.disable();
				tick.reset();
				stage++;
				return;
			}

		}

		// drive forward 78 inches
		if (stage == 2) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(-78);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(78);
				myBot.rightDrivingControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("GO! ");

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - 78) < 4) && (Math.abs(l - (-78)) < 4))
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
		
		// pick up a tote
		if (stage == 5) {
			if (stageCounts[stage] == 0) {
				myBot.elevatorControl.enable();
				myBot.elevatorUp();
				System.out.println("stage 5 succeeded!");
				LEDSignboard.sendTextMessage("TOTE ");

			}
		}
		
		// turn right 90 degrees
		if (stage == 6) {
			myBot.elevatorControl.enable();
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
				System.out.println("stage 6 succeeded!");
				myBot.elevatorControl.disable();
				tick.reset();
				stage++;
				return;
			}

		}
		
		// drive forward 78 inches
		if (stage == 7) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(-78);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(78);
				myBot.rightDrivingControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("GO! ");

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - 78) < 4) && (Math.abs(l - (-78)) < 4))
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > 10) {
				tick.reset();
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				System.out.println("stage 7 succeeded!");
				System.out.printf("%f Exiting Stage %d\n", tick.get(), stage);
				tick.reset();
				stage++;
				return;
			}

		}

		// turn left 90 degrees
		if (stage == 8) {
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
				System.out.println("stage 8 succeeded!");
				tick.reset();
				stage++;
				return;
			}

		}

		// approach the tote
		if (stage == 9) {
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
				System.out.println("stage 9 succeeded!");
				System.out.printf("%f Exiting Stage %d\n", tick.get(), stage);
				tick.reset();
				stage++;
				return;
			}
		}
		
		// drive forward 144 inches
		if (stage == 10) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(-144);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(144);
				myBot.rightDrivingControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("WALK THE PLANK!");

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - 144) < 4) && (Math.abs(l - (-144)) < 4))
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
		// put down totes
		if (stage == 11) {
			if (stageCounts[stage] == 0) {
				myBot.elevatorControl.disable();
				myBot.slowElevatorControl.enable();
				myBot.slowElevatorControl.setSetpoint(19.4);
				myBot.elevatorIndex = -1;
				System.out.println("stage 11 succeeded!");
				LEDSignboard.sendTextMessage("ANCHORS AWAY! ");

			}
		}
		
		// drive back pi inches
		if (stage == 12) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(Math.PI);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(-Math.PI);
				myBot.rightDrivingControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("WALK THE PLANK!");

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - (-Math.PI)) < 4) && (Math.abs(l - (Math.PI)) < 4))
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > 10) {
				tick.reset();
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				System.out.println("stage 12 succeeded!");
				System.out.printf("%f Exiting Stage %d\n", tick.get(), stage);
				tick.reset();
				stage++;
				return;
			}

		}

		if (stage != stageLast) {
			stageTime.stop();
			System.out.println("Stage time: " + stageTime.get());
			stageTime.reset();
			stageTime.start();
		}

		stageLast = stage;
		stageCounts[stage]++;

	}

}
