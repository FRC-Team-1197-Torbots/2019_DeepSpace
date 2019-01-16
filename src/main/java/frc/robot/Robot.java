package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
	private DriveHardware hardware;
	private TorDrive drive;
	private Joystick player1;
	private boolean test;
	
	public Robot() {
		test = false;
		hardware = new DriveHardware();																																																																																																					
		player1 = new Joystick(0);
		drive = new TorDrive(hardware, player1);
	}
	@Override
	public void robotInit() {
		hardware.init();
	}
	@Override
	public void autonomousInit() {
		hardware.init();
	}
	@Override
	public void autonomousPeriodic() {
		drive.Run(test, true);//IT IS NOW TELEOP IN AUTO
	}
	@Override
	public void teleopPeriodic() {
		drive.Run(test, true);//IT IS TELEOP
	}
	@Override
	public void testPeriodic() {
		drive.Run(true, false);//whether or not it is teleop in test mode does not matter
	}
	/*
	 *  The following are a bunch of accessor methods to obtain input from the controller.
	 */
	public double getLeftX(){
		return player1.getRawAxis(0);
	}

	public double getLeftY(){
		return player1.getRawAxis(1);
	}

	public double getRightX(){
		return player1.getRawAxis(4);
	}

	public double getRightTrigger(){
		return player1.getRawAxis(3);
	}

	public boolean getShiftButton(){
		return player1.getRawButton(5);
	}

	public boolean getRightBumper(){
		return player1.getRawButton(6);
	}

	public boolean getButtonA(){
		return player1.getRawButton(1);
	}

	public boolean getButtonB(){
		return player1.getRawButton(2);
	}

	public boolean getButtonX(){
		return player1.getRawButton(3);
	}

	public boolean getButtonY(){
		return player1.getRawButton(4);
	}
}
