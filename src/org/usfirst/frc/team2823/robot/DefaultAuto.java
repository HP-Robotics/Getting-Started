package org.usfirst.frc.team2823.robot;

import org.usfirst.frc.team2823.robot.Robot.ShimmyMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

// total 1: 7.628
// total 2: 
public class DefaultAuto implements AutoMode {
	Robot myBot;

	double stageTimeouts[] = { 0.2, 2.0, 2.0, 2.0, 0.1, 0.2, 1.0, 2.0, 2.0, 1.5, 1.0, 1.5, 3.5, 9001, 0.5 }; // total 20.0, used 0.5 for stage 13
	//lift, turn, drive 78, turn, save position, lift, return, turn, drive 78, turn, drive 10*sqrt(2), turn, drive 144, drop, drive back pi
	int stageCounts[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	int ontarget;
	int stage = 0;
	Timer tick;
	int stageLast = 0;
	boolean runFirstHalf = true;
	boolean runSecondHalf = true;
	final static int SECOND_HALF_START = 5;
	final static int ONTARGET_THRESHOLD = 10; // the minimum number of loops on target required to move to the next stage



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

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.turningControl.disable();
				nextStage();
			}

		}

		// drive forward 78 inches
		if (stage == 2) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(-81);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(81);
				myBot.rightDrivingControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("GO! ");

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - 81) < 4) && (Math.abs(l - (-81)) < 4))
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
				LEDSignboard.sendTextMessage("PI OVER TWOOOOO! ");

			}

			if (Math.abs(myBot.myGyro.getAngle() - (-90)) < 6)
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.turningControl.disable();
				nextStage();
			}

		}

		// save wheel positions
		if (stage == 4) {
			if (stageCounts[stage] == 0) {
				myBot.saveWheelPositions();
				nextStage();
			}
		}

		// shimmy
		if (stage == 5) {
			
			if (myBot.shimmy != ShimmyMode.FINISHED) {
				double axis1 = 0;
				double axis3 = 0;
				
				if (myBot.shimmy == ShimmyMode.DISABLED) {
					myBot.shimmyInit();
				}

				myBot.doShimmy();

				if (myBot.shimmy == ShimmyMode.LEFT)
					axis1 = -1 * myBot.shimmyPower;
				if (myBot.shimmy == ShimmyMode.RIGHT)
					axis3 = -1 * myBot.shimmyPower;
				
				myBot.driveRobot(axis1, axis3);
			}
			else {
				nextStage();
			}
		}
		
		// return to saved wheel positions
		if (stage == 6) {
			if (stageCounts[stage] == 0){
				myBot.rewinding = true;
				myBot.returnToWheelPositions();
				ontarget = 0;
			}
			
			if ((myBot.encoderToInches(Math.abs(myBot.leftEncoder.get() - myBot.leftWheelPosition)) < 0.5) && (myBot.encoderToInches(Math.abs(myBot.rightEncoder.get() - myBot.rightWheelPosition)) < 0.5))
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.turningControl.disable();
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
				LEDSignboard.sendTextMessage("TURN ");

			}

			if (Math.abs(myBot.myGyro.getAngle() - 90) < 6)
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.turningControl.disable();
				nextStage();
			}

		}
		
		// drive forward 68 inches
		if (stage == 8) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(-68);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(68);
				myBot.rightDrivingControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("GO! ");

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - 68) < 4) && (Math.abs(l - (-68)) < 4))
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
				LEDSignboard.sendTextMessage("PI OVER TWOOOOO! ");

			}

			if (Math.abs(myBot.myGyro.getAngle() - (-45)) < 6)
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.turningControl.disable();
				nextStage();
			}

		}
		
		// drive forward 14.14... inches
		if (stage == 10) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(-10*Math.sqrt(2));
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(10*Math.sqrt(2));
				myBot.rightDrivingControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("GO! ");

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - (10*Math.sqrt(2))) < 4) && (Math.abs(l - (-10*Math.sqrt(2))) < 4))
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
				LEDSignboard.sendTextMessage("PI OVER TWOOOOO! ");

			}

			if (Math.abs(myBot.myGyro.getAngle() - (-45)) < 6)
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.turningControl.disable();
				nextStage();
			}

		}
		
		// drive forward 144 inches
		if (stage == 12) {

			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(-134);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(134);
				myBot.rightDrivingControl.enable();
				ontarget = 0;
				LEDSignboard.sendTextMessage("WALK THE PLANK!");

			}

			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());

			if ((Math.abs(r - 134) < 4) && (Math.abs(l - (-134)) < 4))
				ontarget++;
			else
				ontarget = 0;

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				nextStage();
			}

		}
		// put down totes
		if (stage == 13) {
			if (stageCounts[stage] == 0) {
				myBot.elevatorControl.enable();
				myBot.elevatorControl.setSetpoint(12);
				myBot.elevatorIndex = -1;
				LEDSignboard.sendTextMessage("ANCHORS AWAY! ");
			}
			
			if (myBot.encoderToInches(myBot.elevatorEncoder.get()) < 19.4) {
				nextStage();
			}
		}
		
		// drive back pi inches
		if (stage == 14) {

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

			if (ontarget > ONTARGET_THRESHOLD) {
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				nextStage();
			}

		}

		stageCounts[stage]++;

	}

	public void nextStage()
	{
		System.out.printf("Stage Finished: %d\tTime: %f\tTotal Time:%f\n",stage,tick.get(),DriverStation.getInstance().getMatchTime());
		tick.reset();
		stage++;
	}

}