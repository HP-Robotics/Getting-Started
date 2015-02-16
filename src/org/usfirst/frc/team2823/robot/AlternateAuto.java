package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;


public class AlternateAuto implements AutoMode {
	Robot myBot;
	
	double 	stageTimeouts[] = {1.0,2.0,2.0};
	int     stageCounts[] = {0, 0, 0 };
	boolean stageTimeoutFailure[] = { false, false, true };
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
		
		if (stage < 0)
			return;
		
		

		if (stageCounts[stage] == 0)
			System.out.println(stage);

	
		if (tick.get() > stageTimeouts[stage]) {
			
			System.out.printf("stage %d timed out\n", stage);

			myBot.rightDrivingControl.disable();
			myBot.leftDrivingControl.disable();
			myBot.turningControl.disable();
			
			if (stageTimeoutFailure[stage]) {
				System.out.printf("stage %d failed!\n", stage);
				stage = -1;
				return;
			}
			
			System.out.println("Continuing anyway...");
			tick.reset();
			stage++;
			return;
		}
		
		if (stage == 0) {
			//myBot.elevatorControl.enable();
			//myBot.elevatorUp();
			System.out.println("stage 0 succeeded!");

			tick.reset();
			stage++;
			return;
		}
		
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
			
			if (myBot.rightDrivingControl.onTarget() && myBot.leftDrivingControl.onTarget())
				ontarget++;
			else
				ontarget = 0;
			
			if (ontarget > 10) {
				tick.reset();
				myBot.rightDrivingControl.disable();
				myBot.leftDrivingControl.disable();
				System.out.println("stage 1 succeeded!");
				tick.reset();
				stage++;
				return;
			}
			
		}
		
		if (stage == 2) {
			if (stageCounts[stage] == 0) {
				myBot.myGyro.reset();
				myBot.turningControl.setSetpoint(90);
				myBot.turningControl.enable();
				ontarget = 0;
			}
			
			if (myBot.turningControl.onTarget())
				ontarget++;
			else
				ontarget = 0;
			
			if (ontarget > 10) {
				tick.reset();
				myBot.turningControl.disable();
				System.out.println("stage 2 succeeded!");
				tick.reset();
				stage++;
				return;
			}
			
		}
		
		stageCounts[stage]++;
	}

}
