package frc.robot;

import frc.robot.PID_Tools.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class ArcadeDriveController extends DriveController {

	private double throttleAxis;
	private double arcadeSteerAxis;
	private double leftOutput;
	private double rightOutput;
	private double rightMotorSpeed;
	private double leftMotorSpeed;
	private Joystick player1;

	// for the limelight
	private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	private NetworkTableEntry tx = table.getEntry("tx");
	private NetworkTableEntry ta = table.getEntry("ta");
	private double x;
	private double speedChange;
	private double area;
	private boolean buttonAPressed;
	private double distance;//maybe later?
	// private final double maxOmega = 0;//maybe later?
	// private final double maxVelocity= 0;//maybe later?
	private BantorPID limeLightPID;
	private double currentVelocity;
	// private double omegaOverVelocity;
	private TorDerivative findCurrentVelocity;
	private final boolean goCurve = false;

	//time stuff to make sure only goes in correct intervals
	private long currentTime;
	private long startTime = (long) (Timer.getFPGATimestamp() * 1000);
	private double timeInterval = 0.005;
	private double dt = timeInterval;
	private long lastCountedTime;
	private boolean starting = true;

	/*
	 * tuneable
	 * things---------------------------------------------------------------->>>>>>>
	 * >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	 */
	private final double areaAt1Foot = 1.4;//in percent//maybe later?

	// limelight PID
	// Stuff------------------------------>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

	private final double positionkP = 0.0;
	private final double positionkI = 0.0;
	private final double positionkD = 0.0;
	private final double positionTolerance = 0.5 * Math.PI / 180.0;// for thePID
	private final double velocitykP = 0.0;// velocity stuff probably not needed at all and should keep 0
	private final double velocitykI = 0.0;
	private final double velocitykD = 0.0;
	private final double kV = 0.0;// this should definitely stay at 0
	private final double kA = 0.0;// this should definitely stay at 0
	private final double velocityTolerance = 0.0;
	private final double targetVelocity = 0.0;// probably won't need
	private final double targetAcceleration = 0.0;// probably won't need

	// ------------------------------------------------->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

	//for the curving stuff
	private final double goForwardValue = 0.3;

	/*
	 * no more tuneable
	 * things-------------------------------------------------------->>>>>>>>>>>>>>>
	 * >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	 */

	public ArcadeDriveController(DriveHardware hardware, Joystick player1) {
		super(hardware, player1);
		this.player1 = player1;

		// this is the PID
		limeLightPID = new BantorPID(kV, kA, positionkP, positionkI, positionkD, velocitykP, velocitykI, velocitykD, dt,
				positionTolerance, velocityTolerance);
		limeLightPID.reset();
		findCurrentVelocity = new TorDerivative(dt);
		findCurrentVelocity.resetValue(0);
	}

	@Override
	public void run() {
		currentTime = (long) (Timer.getFPGATimestamp() * 1000);
		// this handles it so that it will only tick in the time interval so that the derivatives and the integrals are correct
		if (((currentTime - startTime) - ((currentTime - startTime) % (dt * 1000))) > // has current time minus start time to see the relative time the trajectory has been going
		((lastCountedTime - startTime) - ((lastCountedTime - startTime) % (dt * 1000))) // subtracts that mod dt times 1000 so that it is floored to
		// the nearest multiple of dt times 1000 then checks if that is greater than the last one to see if it is time to move on to the next tick
		|| starting) {
			starting = false;
			lastCountedTime = currentTime;

			buttonAPressed = player1.getRawButton(1);
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
			} else {
				arcadeSteerAxis = -(arcadeSteerAxis * arcadeSteerAxis);
			}

			if (throttleAxis >= 0.0D) {
				throttleAxis *= throttleAxis;
			} else {
				throttleAxis = -(throttleAxis * throttleAxis);
			}

			if (throttleAxis > 0.0D) {
				if (arcadeSteerAxis > 0.0D) {
					leftMotorSpeed = throttleAxis - arcadeSteerAxis;
					rightMotorSpeed = Math.max(throttleAxis, arcadeSteerAxis);
				} else {
					leftMotorSpeed = Math.max(throttleAxis, -arcadeSteerAxis);
					rightMotorSpeed = throttleAxis + arcadeSteerAxis;
				}
			} else {
				if (arcadeSteerAxis > 0.0D) {
					leftMotorSpeed = -Math.max(-throttleAxis, arcadeSteerAxis);
					rightMotorSpeed = throttleAxis + arcadeSteerAxis;
				} else {
					leftMotorSpeed = throttleAxis - arcadeSteerAxis;
					rightMotorSpeed = -Math.max(-throttleAxis, -arcadeSteerAxis);
				}
			}

			// get all the values from the limelight
			x = tx.getDouble(0.0);
			area = ta.getDouble(0.0);//maybe later?

			// convert the angles into radians
			x *= ((Math.PI) / 180.0);
			distance = areaAt1Foot / area;//maybe later?

			if(goCurve) {
				currentVelocity = findCurrentVelocity.estimate(x);
				limeLightPID.updateTargets(0, targetVelocity, targetAcceleration);
				limeLightPID.updateCurrentValues(x, currentVelocity);
				speedChange = limeLightPID.update();
	
				if (buttonAPressed) {
					rightMotorSpeed += speedChange;
					leftMotorSpeed -= speedChange;
				}

			} else {
				if(buttonAPressed) {
					if(distance < 1.5) {//we just pretty much turn towards it and go forwards
						currentVelocity = findCurrentVelocity.estimate(x);
						limeLightPID.updateTargets(0, targetVelocity, targetAcceleration);
						limeLightPID.updateCurrentValues(x, currentVelocity);
						speedChange = limeLightPID.update();
						rightMotorSpeed = goForwardValue + speedChange;
						leftMotorSpeed = goForwardValue - speedChange;
					} else {
						//well we'll do this later cuz we might not even need it. go curve is just false for now
					}
				}
			}

			setRightOutput(rightMotorSpeed);
			setLeftOutput(leftMotorSpeed);
		}
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

	public void setTargets(double velocity, double omega) {
		//sets leftMotorSpeed and rightMotorSpeed

	}
}
