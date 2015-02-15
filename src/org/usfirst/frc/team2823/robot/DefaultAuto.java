package org.usfirst.frc.team2823.robot;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
public class DefaultAuto implements AutoMode {
	Robot myBot;
	Timer timer=new Timer();
	public DefaultAuto(Robot myBot){
		this.myBot = myBot;
	}
	
	public void autoInit() {
		System.out.println("Hello Jeremy!");
		timer.reset();
		timer.start();
	}
	
	public void autoPeriodic() {
		if (timer.get()<myBot.myDriveTime) {
		myBot.talon1.set(-0.5);
		myBot.talon2.set(-0.5);
		myBot.talon3.set(0.5);
		myBot.talon4.set(0.5);
		}
		else {
			myBot.talon1.set(0);
			myBot.talon2.set(0);
			myBot.talon3.set(0);
			myBot.talon4.set(0);
		}
	}
}
