package org.usfirst.frc.team2823.robot;

public class AlternateAuto implements AutoMode {
	Robot myBot;
	public AlternateAuto(Robot myBot) {
		this.myBot = myBot;
	}
	
	public void autoInit() {
		System.out.println("Hey, you chose the alternate autonomous mode. Good job!");
	}
	
	public void autoPeriodic() {
		
	}

}
