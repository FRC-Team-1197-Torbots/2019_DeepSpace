package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class TestDriveController extends DriveController {
	private double leftOutput;
	private double rightOutput;
	public TestDriveController(DriveHardware hardware, Joystick player1) {
		super(hardware, player1);
	}
	
	@Override
	public void run() {
		//We can do whatever we want but we have to make sure we set these speeds so we don't get null errors
		setLeftOutput(0);
		setRightOutput(0);
	}

	@Override
	public double getLeftOutput() {
		return leftOutput;
	}

	@Override
	public void setLeftOutput(double left) {
		leftOutput = left;
	}

	@Override
	public double getRightOutput() {
		return rightOutput;
	}

	@Override
	public void setRightOutput(double right) {
		rightOutput = right;
	}
}
