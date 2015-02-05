package org.usfirst.frc.team2823.robot;

public class DefaultAuto implements AutoMode {
	Robot myBot;
	public DefaultAuto(Robot myBot){
		this.myBot = myBot;
	}
	
	public void autoInit() {
		System.out.println("Hello Jeremy!");
	}
	
	public void autoPeriodic() {

	}
}
