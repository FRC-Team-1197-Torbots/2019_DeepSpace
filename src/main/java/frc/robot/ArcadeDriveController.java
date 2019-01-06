package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class ArcadeDriveController extends DriveController {
	
	private double throttleAxis;
	private double arcadeSteerAxis;
	private double leftOutput;
	private double rightOutput;
	private Joystick player1;
	public ArcadeDriveController(DriveHardware hardware, Joystick player1) {
		super(hardware, player1);
		this.player1 = player1;
	}
	
	@Override
	public void run(){
		throttleAxis = player1.getRawAxis(1);
		arcadeSteerAxis = player1.getRawAxis(0);
		if (Math.abs(arcadeSteerAxis) <= 0.1) {
			arcadeSteerAxis = 0.0D;
		}
		if (Math.abs(throttleAxis) <= 0.2D) {
			throttleAxis = 0.0D;
		}

		if (arcadeSteerAxis >= 0.0D) {
			arcadeSteerAxis *= arcadeSteerAxis;
		} 
		else {
			arcadeSteerAxis = -(arcadeSteerAxis * arcadeSteerAxis);
		}
		
		if (throttleAxis >= 0.0D) {
			throttleAxis *= throttleAxis;
		} 
		else {
			throttleAxis = -(throttleAxis * throttleAxis);
		}
		
		double rightMotorSpeed;
		double leftMotorSpeed;

		if (throttleAxis > 0.0D) {
			if (arcadeSteerAxis > 0.0D) {
				leftMotorSpeed = throttleAxis - arcadeSteerAxis;
				rightMotorSpeed = Math.max(throttleAxis, arcadeSteerAxis);
			}
			else {
				leftMotorSpeed = Math.max(throttleAxis, -arcadeSteerAxis);
				rightMotorSpeed = throttleAxis + arcadeSteerAxis;
			}
		}
		else {
			if (arcadeSteerAxis > 0.0D) {
				leftMotorSpeed = -Math.max(-throttleAxis, arcadeSteerAxis);
				rightMotorSpeed = throttleAxis + arcadeSteerAxis;
			}
			else {
				leftMotorSpeed = throttleAxis - arcadeSteerAxis;
				rightMotorSpeed = -Math.max(-throttleAxis, -arcadeSteerAxis);
			}
		}
		
		setRightOutput(rightMotorSpeed);
		setLeftOutput(leftMotorSpeed);
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
