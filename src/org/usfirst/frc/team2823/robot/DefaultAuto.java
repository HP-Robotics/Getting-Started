package org.usfirst.frc.team2823.robot;

import org.usfirst.frc.team2823.robot.Robot.ShimmyMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

// total 1: 7.628
// total 2: 
public class DefaultAuto implements AutoMode {
	Robot myBot;

	double[] stageTimeouts = { 0.1, 2.0, 2.0, 2.0, 0.4, 0.1, 0.4, 2.0, 2.0, 1.5, 1.0, 1.5, 0.5, 0.5, 9001, 3.5, 0.5, 0.1 }; // total 20.3, used 0.5 for stage 13
	//lift, turn, drive 74, turn, drive pi, lift, drive -pi, turn, drive 74, turn, drive 10*sqrt(2), turn, drive 5, drive -1, drop, drive 130, drive -1, lift
	int[] stageCounts = new int[18];
	int ontarget;
	int stage = 0;
	Timer tick;
	int stageLast = 0;
	boolean runFirstHalf = true;
	boolean runSecondHalf = false;
	final static int SECOND_HALF_START = 5;
	final static int ONTARGET_THRESHOLD = 10; // the minimum number of loops on target required to move to the next stage
	final static double TURN_PRECISION = 6;		//threshold for counting it as on target (degrees)
	final static double LONG_DISTANCE_PRECISION = 4;	//threshold for counting it as on target (inches)
	final static double SHORT_DISTANCE_PRECISION = 0.5;	//threshold for counting it as on target (inches)


	public DefaultAuto(Robot myBot) {
		this.myBot = myBot;
	}

	public void autoInit() {
		tick = new Timer();
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
	}

	public void autoPeriodic() {

		if (stage < 0 || stage >= stageCounts.length)
			return;
		
		if((runFirstHalf == true) && (runSecondHalf == false) && (stage >= SECOND_HALF_START))
			return;

		if (tick.get() > stageTimeouts[stage]) {

			System.out.printf("stage %d timed out\n", stage);

			myBot.disableAllPIDControllers();
			myBot.elevatorControl.enable();
			nextStage();
			return;
		}

		// pick up a tote
		if (stage == 0) {
			if (stageCounts[stage] == 0) {
				myBot.elevatorControl.enable();
				myBot.elevatorUp();
				LEDSignboard.sendTextMessage("* #TEAM20POINTAUTO");

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
			}

			if (Math.abs(myBot.myGyro.getAngle() - 90) < TURN_PRECISION)
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.turningControl.disable();
				nextStage();
			}

		}

		// drive forward 81 inches
		if (stage == 2) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(-81);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(81);
				myBot.rightDrivingControl.enable();
				ontarget = 0;

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - 81) < LONG_DISTANCE_PRECISION) && (Math.abs(l - (-81)) < LONG_DISTANCE_PRECISION))
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				nextStage();
			}

		}

		// turn left 90 degrees
		if (stage == 3) {
			if (stageCounts[stage] == 0) {
				myBot.myGyro.reset();
				myBot.turningControl.setSetpoint(-90);
				myBot.turningControl.enable();
				ontarget = 0;
			}

			if (Math.abs(myBot.myGyro.getAngle() - (-90)) < TURN_PRECISION)
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.turningControl.disable();
				nextStage();
			}

		}

		// drive forward PI"
		if (stage == 4) {
			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(-Math.PI);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(Math.PI);
				myBot.rightDrivingControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("DONE!");

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - (Math.PI)) < SHORT_DISTANCE_PRECISION) && (Math.abs(l - (-Math.PI)) < SHORT_DISTANCE_PRECISION))
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				nextStage();
			}

		}

		// shimmy
		//TODO: Maybe delay .1 sec to allow tote to load
		if (stage == 5) {
			if (stageCounts[stage] == 0) {
				myBot.elevatorControl.enable();
				myBot.elevatorUp();
				LEDSignboard.sendTextMessage("INSTANT MESSAGE... YOU'LL NEVER READ THIS PART!");
			}
		}
		
		// drive back pi
		if (stage == 6) {
			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(Math.PI);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(-Math.PI);
				myBot.rightDrivingControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("DONE!");

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - (-Math.PI)) < SHORT_DISTANCE_PRECISION) && (Math.abs(l - (Math.PI)) < SHORT_DISTANCE_PRECISION))
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				nextStage();
			}

		}
		
		// turn right 90 degrees
		if (stage == 7) {
			myBot.elevatorControl.enable();
			if (stageCounts[stage] == 0) {
				myBot.myGyro.reset();
				myBot.turningControl.setSetpoint(90);
				myBot.turningControl.enable();
				ontarget = 0;

			}

			if (Math.abs(myBot.myGyro.getAngle() - 90) < TURN_PRECISION)
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.turningControl.disable();
				nextStage();
			}

		}
		
		// drive forward 74 inches
		if (stage == 8) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(-74);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(74);
				myBot.rightDrivingControl.enable();
				ontarget = 0;

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - 74) < LONG_DISTANCE_PRECISION) && (Math.abs(l - (-74)) < LONG_DISTANCE_PRECISION))
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				nextStage();
			}

		}

		// turn left 45 degrees
		if (stage == 9) {
			if (stageCounts[stage] == 0) {
				myBot.myGyro.reset();
				myBot.turningControl.setSetpoint(-45);
				myBot.turningControl.enable();
				ontarget = 0;
			}

			if (Math.abs(myBot.myGyro.getAngle() - (-45)) < TURN_PRECISION)
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.turningControl.disable();
				nextStage();
			}

		}
		
		// drive forward 7*sqrt(2)... inches
		if (stage == 10) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(-7*Math.sqrt(2));
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(7*Math.sqrt(2));
				myBot.rightDrivingControl.enable();
				ontarget = 0;
			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - (7*Math.sqrt(2))) < SHORT_DISTANCE_PRECISION) && (Math.abs(l - (-7*Math.sqrt(2))) < SHORT_DISTANCE_PRECISION))
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				nextStage();
			}

		}

		// turn left 45 degrees
		if (stage == 11) {
			if (stageCounts[stage] == 0) {
				myBot.myGyro.reset();
				myBot.turningControl.setSetpoint(-45);
				myBot.turningControl.enable();
				ontarget = 0;
			}

			if (Math.abs(myBot.myGyro.getAngle() - (-45)) < TURN_PRECISION)
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.turningControl.disable();
				nextStage();
			}

		}
		
		//drive forward 5 inches
		if (stage == 12) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(-5);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(5);
				myBot.rightDrivingControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("**ALIGNING...");

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - 5) < SHORT_DISTANCE_PRECISION) && (Math.abs(l - (-5)) < SHORT_DISTANCE_PRECISION))
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				nextStage();
			}
		}
		
		// drive back 1 inch
		if (stage == 13) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(1);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(-1);
				myBot.rightDrivingControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("**ALIGNING...");

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - (-1)) < SHORT_DISTANCE_PRECISION) && (Math.abs(l - (1)) < SHORT_DISTANCE_PRECISION))
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				nextStage();
			}
		}
		
		// drop totes
		if (stage == 14) {
			if (stageCounts[stage] == 0) {
				myBot.elevatorControl.enable();
				myBot.elevatorControl.setSetpoint(22.6);
				myBot.elevatorIndex = -1;
				LEDSignboard.sendTextMessage("ANCHORS AWAY! ");
			}
			
			if (myBot.encoderToInches(myBot.elevatorEncoder.get()) < 19.4) {
				nextStage();
			}
		}
		
		// drive forward 130 inches
		if (stage == 15) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(-130);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(130);
				myBot.rightDrivingControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("**GO! GO! GOOOOOOOOOOO!");

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - 134) < LONG_DISTANCE_PRECISION) && (Math.abs(l - (-134)) < LONG_DISTANCE_PRECISION))
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				nextStage();
			}
		}
		
		// drive back 1 inch
		if (stage == 16) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(1);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(-1);
				myBot.rightDrivingControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("DISENGAGE...");

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - (-1)) < SHORT_DISTANCE_PRECISION) && (Math.abs(l - (1)) < SHORT_DISTANCE_PRECISION))
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				nextStage();
			}

		}
		
		if (stage == 17) {
			if (stageCounts[stage] == 0) {
				myBot.elevatorControl.enable();
				myBot.elevatorUp();
				myBot.elevatorUp();
				LEDSignboard.sendTextMessage("!");
			}
		}

		if(stage < stageCounts.length) {
			stageCounts[stage]++;
		}

	}

	public void nextStage()
	{
		System.out.printf("Stage Finished: %d\tTime: %f\tTotal Time:%f\n",stage,tick.get(),DriverStation.getInstance().getMatchTime());
		tick.reset();
		stage++;
	}

}