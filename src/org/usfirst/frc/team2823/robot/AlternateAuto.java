package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;


public class AlternateAuto implements AutoMode {
	Robot myBot;
	
	double 	stageTimeouts[] = {0.5, 2.0, 2.0, 3.0, 0.5, 2.0, 1.5, 0.5};
	int     stageCounts[] = {0, 0, 0, 0, 0, 0, 0, 0 };
	boolean stageTimeoutFailure[] = { false, false, false, false, false, false, false, false };
	int ontarget;
	int stage = 0;
	Timer tick;
	
	public AlternateAuto(Robot myBot) {
		this.myBot = myBot;
	}
	
	public void autoInit() {
		tick = new Timer();
		tick.reset();
		tick.start();
		stage = 0;

		for (int i = 0; i < stageCounts.length; i++ )
			stageCounts[i] = 0;
		
		System.out.println("Hey, you chose the alternate autonomous mode. Good job!");
	}
	
	public void autoPeriodic() {
		
		if (stage < 0 || stage >= stageCounts.length)
			return;
		
		

		if (stageCounts[stage] == 0)
			System.out.println(stage);

	
		if (tick.get() > stageTimeouts[stage]) {
			
			System.out.printf("stage %d timed out\n", stage);

			myBot.rightDrivingControl.disable();
			myBot.leftDrivingControl.disable();
			myBot.turningControl.disable();
			myBot.rightIRControl.disable();
			myBot.leftIRControl.disable();
			
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
			if(stageCounts[stage] == 0) {
				myBot.elevatorControl.enable();
				myBot.elevatorUp();
				System.out.println("stage 0 succeeded!");
			}
		}
		
		//drive back 12 inches
		if (stage == 1) {
			
			if (stageCounts[stage] == 0) {
				myBot.leftEncoder.reset();
				myBot.rightEncoder.reset();
				myBot.leftDrivingControl.setSetpoint(12);
				myBot.leftDrivingControl.enable();
				myBot.rightDrivingControl.setSetpoint(-12);
				myBot.rightDrivingControl.enable();
				ontarget = 0;
			}
			
			double l = myBot.driveEncoderToInches(myBot.leftEncoder.get());
			double r = myBot.driveEncoderToInches(myBot.rightEncoder.get());
			
			if ((Math.abs(r - (-12)) < 2) && (Math.abs(l - 12) < 2))
				ontarget++;
			else
				ontarget = 0;
			
			if (ontarget > 10) {
				System.out.printf("%f Exiting Stage %d\n", tick.get(), stage);
				tick.reset();
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				System.out.println("stage 1 succeeded!");
				tick.reset();
				stage++;
				return;
			}
			
		}
		
		// turn right 90 degrees
		if (stage == 2) {
			if (stageCounts[stage] == 0) {
				myBot.myGyro.reset();
				myBot.turningControl.setSetpoint(90);
				myBot.turningControl.enable();
				ontarget = 0;
			}
			
			if (Math.abs(myBot.myGyro.getAngle() - 90) < 3)
				ontarget++;
			else
				ontarget = 0;
			
			if (ontarget > 10) {
				System.out.printf("%f Exiting Stage %d\n", tick.get(), stage);
				tick.reset();
				myBot.turningControl.disable();
				System.out.println("stage 2 succeeded!");
				tick.reset();
				stage++;
				return;
			}
			
		}
		
		//drive forward 81 inches
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
			
			if ((Math.abs(r - 81) < 2) && (Math.abs(l - (-81)) < 2))
				ontarget++;
			else
				ontarget = 0;
			
			if (ontarget > 10) {
				tick.reset();
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				System.out.println("stage 3 succeeded!");
				System.out.printf("%f Exiting Stage %d\n", tick.get(), stage);
				tick.reset();
				stage++;
				return;
			}
			
		}
		
		// go up a level
		if (stage == 4) {
			if(stageCounts[stage] == 0) {
				myBot.elevatorUp();
				System.out.println("stage 4 succeeded!");
			}
		}
		
		// turn left 90 degrees
		if (stage == 5) {
			if (stageCounts[stage] == 0) {
				myBot.myGyro.reset();
				myBot.turningControl.setSetpoint(-90);
				myBot.turningControl.enable();
				ontarget = 0;
			}
			
			if (Math.abs(myBot.myGyro.getAngle() - (-90)) < 3)
				ontarget++;
			else
				ontarget = 0;
			
			if (ontarget > 10) {
				tick.reset();
				myBot.turningControl.disable();
				System.out.println("stage 5 succeeded!");
				System.out.printf("%f Exiting Stage %d\n", tick.get(), stage);
				tick.reset();
				stage++;
				return;
			}
			
		}
		
		//approach the tote
		if (stage == 6) {
			
			if (stageCounts[stage] == 0) {
				myBot.leftIRControl.setSetpoint(1.68);
				myBot.rightIRControl.setSetpoint(1.68);
				myBot.leftIRControl.enable();
				myBot.rightIRControl.enable();
				ontarget = 0;
			}
			
			if ((Math.abs(myBot.infraredSensorLeft.getAverageVoltage() - (1.68)) < 0.1) && (Math.abs(myBot.infraredSensorRight.getAverageVoltage() - 1.68) < 0.1))
				ontarget++;
			else
				ontarget = 0;
			
			if (ontarget > 10) {
				tick.reset();
				myBot.rightIRControl.disable();
				myBot.leftIRControl.disable();
				System.out.println("stage 6 succeeded!");
				System.out.printf("%f Exiting Stage %d\n", tick.get(), stage);
				tick.reset();
				stage++;
				return;
			}
			
		}
		
		// pick up tote #2
		if (stage == 7) {
			if(stageCounts[stage] == 0) {
				myBot.shimmyInit();
			}
			
			if(!myBot.doShimmy())
			{
				myBot.elevatorUp();
				myBot.elevatorUp();
				System.out.println("stage 7 succeeded!");
				System.out.printf("%f Exiting Stage %d\n", tick.get(), stage);
				tick.reset();
				stage++;
			}
		}
		
		stageCounts[stage]++;
	}

}
