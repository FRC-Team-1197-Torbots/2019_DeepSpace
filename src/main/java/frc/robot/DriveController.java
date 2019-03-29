package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

//this only runs in auto and is in auto periodic
public abstract class DriveController {
	
	public DriveController(DriveHardware hardware, Joystick player1) {
	}
	
	public abstract double getLeftOutput();
	public abstract void setLeftOutput(double left);
	public abstract double getRightOutput();
	public abstract void setRightOutput(double right);
	public abstract void run();
	public abstract void limeLightTop(boolean top);
	public abstract void init();
}
