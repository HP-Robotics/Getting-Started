package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

// total 1: 7.628
// total 2: 
public class DefaultAuto implements AutoMode {
	Robot myBot;

	double[] stageTimeouts = { 0.1, 0.8, 2.0, 2.0, 2.0, 3.5, 5.0, 2.0}; // total 20.3, used 0.5 for stage 13
	//lift, back, turn, drive 74, turn, drive pi, lift, drive -pi, turn, drive 74, turn, drive 10*sqrt(2), turn, drive 5, drive -1, drop, drive 130, drive -1, lift
	int[] stageCounts = new int[8];
	int ontarget;
	int stage = 0;
	Timer tick;
	int stageLast = 0;
	boolean runFirstHalf = true;
	boolean runSecondHalf = true;
	final static int SECOND_HALF_START = 3;
	final static int ONTARGET_THRESHOLD = 10; // the minimum number of loops on target required to move to the next stage
	final static double TURN_PRECISION = 2;		//threshold for counting it as on target (degrees)
	final static double LONG_DISTANCE_PRECISION = 3;	//threshold for counting it as on target (inches)
	final static double SHORT_DISTANCE_PRECISION = 1.0;	//threshold for counting it as on target (inches)
    final static double ANGLE_TO_TURN_TO = 90.0; // Target angle
    final static int BACK_UP_DISTANCE = 4;

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
				LEDSignboard.sendTextMessage("*GOING FOR 2!");

			}
		}

		// back up
		
		if (stage == 1){
			
			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(BACK_UP_DISTANCE);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(-BACK_UP_DISTANCE);
				myBot.rightDrivingControl.enable();
				ontarget = 0;

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - (-BACK_UP_DISTANCE)) < SHORT_DISTANCE_PRECISION) && (Math.abs(l - (BACK_UP_DISTANCE)) < SHORT_DISTANCE_PRECISION))
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				nextStage();
				
			}

		}
		
		
			
		// turn right 
		if (stage == 2) {
			myBot.elevatorControl.enable();
			if (stageCounts[stage] == 0) {
				myBot.myGyro.reset();
				myBot.turningControl.setSetpoint(ANGLE_TO_TURN_TO);
				myBot.turningControl.enable();
				ontarget = 0;
			}

			if (Math.abs(myBot.myGyro.getAngle() - ANGLE_TO_TURN_TO) < TURN_PRECISION)
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.turningControl.disable();
				nextStage();
				return;
			}

		}
	

		// drive forward 81 inches
		if (stage == 3) {

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
		if (stage == 4) {
			if (stageCounts[stage] == 0) {
				myBot.myGyro.reset();
				myBot.turningControl.setSetpoint(-ANGLE_TO_TURN_TO);
				myBot.turningControl.enable();
				ontarget = 0;
			}

			if (Math.abs(myBot.myGyro.getAngle() - (-ANGLE_TO_TURN_TO)) < TURN_PRECISION)
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.turningControl.disable();
				nextStage();
			}

		}


		

		



		
		

		
		// drop totes
		if (stage == 5) {
			if (stageCounts[stage] == 0) {
				myBot.elevatorControl.enable();
				myBot.elevatorControl.setSetpoint(10.8);
				myBot.elevatorIndex = -1;
				LEDSignboard.sendTextMessage("ANCHORS AWAY! ");
			}
			
			if (myBot.encoderToInches(myBot.elevatorEncoder.get()) < 15.0) {
				nextStage();
			}
		}
		
		// drive forward 130 inches
		if (stage == 6) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(-144);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(144);
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
		
		// move elevator to 20 inches
		if (stage == 7) {
			if (stageCounts[stage] == 0) {
				myBot.elevatorControl.enable();
				myBot.elevatorControl.setSetpoint(20);
				myBot.elevatorIndex = -1;
				LEDSignboard.sendTextMessage("RELEASE! ");
			}
			
			if (myBot.encoderToInches(myBot.elevatorEncoder.get()) > 19.9) {
				nextStage();
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