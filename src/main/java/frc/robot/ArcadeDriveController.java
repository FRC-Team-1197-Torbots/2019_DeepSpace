package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ArcadeDriveController extends DriveController {
	
	private double throttleAxis;
	private double arcadeSteerAxis;
	private double leftOutput;
	private double rightOutput;
	private double rightMotorSpeed;
	private double leftMotorSpeed;
	private Joystick player1;

	//for the limelight
	private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	private NetworkTableEntry tx = table.getEntry("tx");
	// private NetworkTableEntry ta = table.getEntry("ta");
	private double x;
	private double speedChange;
	// private double area;
	private boolean buttonAPressed;
	// private double distance;//maybe later?
	// private final double maxOmega = 0;//maybe later?
	// private final double maxVelocity= 0;//maybe later?	
	private BantorPID limeLightPID;
	private double currentVelocity;
	private TorDerivative findCurrentVelocity;

	/*
	tuneable things---------------------------------------------------------------->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	*/
	// private final double areaAt2Feet = 2.8;//in percent//maybe later?
	private final double dt = 0.005;

	//limelight PID Stuff------------------------------>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

	private final double positionkP = 0.0;
    private final double positionkI = 0.0;
    private final double positionkD = 0.0;
    private final double positionTolerance = 0.5 * Math.PI / 180.0;//for thePID
    private final double velocitykP = 0.0;//velocity stuff probably not needed at all and should keep 0
    private final double velocitykI = 0.0;
    private final double velocitykD = 0.0;
    private final double kV = 0.0;//this should definitely stay at 0
    private final double kA = 0.0;//this should definitely stay at 0
    private final double velocityTolerance = 0.0;
    private final double targetVelocity = 0.0;//probably won't need
	private final double targetAcceleration = 0.0;//probably won't need

	//------------------------------------------------->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

	/*
	no more tuneable things-------------------------------------------------------->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	*/

	public ArcadeDriveController(DriveHardware hardware, Joystick player1) {
		super(hardware, player1);
		this.player1 = player1;

		//this is the PID
		limeLightPID = new BantorPID(kV, kA, positionkP, positionkI, positionkD, velocitykP,
				velocitykI, velocitykD, dt, positionTolerance, velocityTolerance);
		limeLightPID.reset();
		findCurrentVelocity.resetValue(0);
	}
	
	@Override
	public void run(){
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

		//get all the values from the limelight
		x = tx.getDouble(0.0);
		// area = ta.getDouble(0.0);//maybe later?

		//convert the angles into radians
		x *= ((Math.PI) / 180.0);
		// distance = areaAt2Feet / area;//maybe later?

		
        currentVelocity = findCurrentVelocity.estimate(x);
        limeLightPID.updateTargets(0, targetVelocity, targetAcceleration);
        limeLightPID.updateCurrentValues(x, currentVelocity);
        speedChange = limeLightPID.update();

		if(buttonAPressed) {
			rightMotorSpeed += speedChange;
			leftMotorSpeed -= speedChange;
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
