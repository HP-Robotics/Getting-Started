package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	static final double R = 2;
	static final double LEVEL_HEIGHT = 14;
	static final double ENCODER_RESOLUTION = 2048;
	static final double ELEVATOR_SPEED = 0.05;
	static final int MAXIMUM_LEVEL = 4;
	
	//RobotDrive myRobot;
	Joystick stick;
	int autoLoopCounter;
	Talon talon1;
	Talon talon2;
	Talon talon3;
	Talon talon4;
	Talon talon5;
	Encoder encoder;
	DigitalInput di;
	Gyro myGyro;
	int newlevel;
	int oldlevel;

	boolean BAMUpPressed = false;
	boolean BAMDownPressed = false;
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	//myRobot = new RobotDrive(0,1);
    	stick = new Joystick(0);
    	talon1 = new Talon(0);
    	talon2 = new Talon(1);
    	talon3 = new Talon(2);
    	talon4 = new Talon(3);
    	talon5 = new Talon(4);
    	encoder = new Encoder(0, 1, true, EncodingType.k4X);
    	di = new DigitalInput(2);
    	myGyro = new Gyro(0);
    	
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	autoLoopCounter = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		{
			//myRobot.drive(-0.5, 0.0); 	// drive forwards half speed
			autoLoopCounter++;
			} else {
			//myRobot.drive(0.0, 0.0); 	// stop robot
		}
    }
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit(){
    	encoder.reset();
    	myGyro.reset();
    
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	double axis1 = stick.getRawAxis(1);
    	double axis3 = stick.getRawAxis(3);
    	driveRobot(axis1, axis3);
    	if (stick.getRawButton(6))
    	{
    		if(!(newlevel >= MAXIMUM_LEVEL) && !BAMUpPressed)
    		{
        		newlevel += 1;	
    		}
    		BAMUpPressed = true;
    	}
    	else
    	{
    		BAMUpPressed = false;
    	}
    	
    	if (stick.getRawButton(8))
    	{
    		if(!(newlevel <= 1) && !BAMDownPressed)
    		{
        		newlevel -= 1;
    		}
    	}
    	else
    	{
    		BAMDownPressed = false;
    	}
    	
    	updateElevator();
    	
    	//DEBUG
    	//System.out.printf("%f \t %f \n", axis1, axis3);
    	//System.out.printf("%b \n", di.get());
    	//System.out.println("Gyro: " + myGyro.getAngle() + " Encoder: " + encoder.get());
        //myRobot.arcadeDrive(stick);
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
    public void driveRobot(double a, double b)
    {
    	talon1.set(a);
    	talon2.set(a);
    	// Values are multiplied by -1 to ensure that the motors on the right spin opposite the motors on the left.
    	talon3.set(-1*b);
    	talon4.set(-1*b);
    }
    
    public void updateElevator()
    {
    	int difference = newlevel - oldlevel;
    	if (difference == 0)
    	{
    		talon5.set(0);
    	}
    	else 
    	{
    		if (Math.abs(encoder.get()) >= Math.abs((difference*ENCODER_RESOLUTION*LEVEL_HEIGHT)/(2*Math.PI*R)))
    		{
    			talon5.set(0);
    			oldlevel = newlevel;
    		}
    		else 
    		{
    			talon5.set(ELEVATOR_SPEED*(Math.signum(difference)));
    		}
    		
    	}
    }
}
  