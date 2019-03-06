package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Elevator.*;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

public class Robot extends TimedRobot {
	private DriveHardware hardware;
	private TorDrive drive;
	private Joystick player1;
	private boolean test;
	private Elevator elevator;
	
	public Robot() {
		test = false;
		hardware = new DriveHardware();																																																																																																					
		player1 = new Joystick(0);
		drive = new TorDrive(hardware, player1);
		elevator = new Elevator(player1, drive);


		UsbCamera hatchSideCam = CameraServer.getInstance().startAutomaticCapture(0);
		hatchSideCam.setBrightness(50);
		hatchSideCam.setWhiteBalanceAuto();
		hatchSideCam.setResolution(420, 240);
		hatchSideCam.setFPS(30);
		// hatchSideCam.setExposureManual();
		CvSink cvsink1 = new CvSink("Hatch Side Cam");
		cvsink1.setSource(hatchSideCam);
		cvsink1.setEnabled(true);

		UsbCamera cargoSideCam = CameraServer.getInstance().startAutomaticCapture(1);
		cargoSideCam.setBrightness(50);
		cargoSideCam.setWhiteBalanceAuto();
		cargoSideCam.setResolution(420, 240);
		cargoSideCam.setFPS(30);

		CvSink cvsink2 = new CvSink("Cargo Side Cam");
		cvsink2.setSource(cargoSideCam);
		cvsink2.setEnabled(true);
	}
	@Override
	public void robotInit() {
		hardware.init();
	}
	@Override
	public void autonomousInit() {
		elevator.init();
		hardware.init();
	}
	@Override
	public void autonomousPeriodic() {
		drive.Run(test, true);//IT IS NOW TELEOP IN AUTO
		elevator.update();
	}
	@Override
	public void teleopPeriodic() {
		if(!elevator.climbing()) {
			drive.Run(test, true);//IT IS TELEOP
		}
		elevator.update();
	}
	@Override
	public void testPeriodic() {
		if(!elevator.climbing()) {
			drive.Run(true, false);//whether or not it is teleop in test mode does not matter
		}
		elevator.update();
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
