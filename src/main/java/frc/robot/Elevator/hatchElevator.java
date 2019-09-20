package frc.robot.Elevator;
import frc.robot.PID_Tools.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

public class hatchElevator {
    private final boolean testMode = false;

    private double currentRunningSpeed;
    private double currentRunningAngle;

    private long currentTime;
	private long startTime = (long)(Timer.getFPGATimestamp() * 1000);
	private double timeInterval = 0.005;
	private double dt = timeInterval;
	private long lastCountedTime;
	private boolean starting = true;
    private BantorPID positionPID;
    private double controlPower;//this is the amount of power the PID is giving out
	private TorDerivative findCurrentVelocity;
	private double currentVelocity;

    /*
    tuneable variables------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    */
    //our variables
    private final double positionkP = 3;
    private final double positionkI = 0.0;
    private final double positionkD = 0.0;
    private final double positionTolerance = 0.01;//for thePID
    private final double velocitykP = 0.0;//velocity stuff probably not needed at all and should keep 0
    private final double velocitykI = 0.0;
    private final double velocitykD = 0.0;
    private final double kV = 0.0;
    private final double kA = 0.0;//this should definitely stay at 0
    private final double velocityTolerance = 0.0;
    private final double targetVelocity = 0.0;//probably won't need
    private final double targetAcceleration = 0.0;//probably won't need

    private final double encoderTicksPerMeter = 892;//this is how many ticks there are per meter the elevator goes up

    private final double holdHatchPosition = 0.1;
    private final double intakeHatchPosition = 0.26;
    private final double lowHatchPosition = 0.24;
    private final double midHatchPosition = 0.835;
    private final double highHatchPosition = 0.835;

    private final double absoluteMaxUpwardVelocity = 0.6;//don't make it higher than 1.0 POSITIVE
    private final double absoluteMaxDownwardVelocity = 0.6;//don't make it higher than 1.0 POSITIVE

    //ball arm
    private final double intakeHatchAngle = 5;
    private final double lowHatchAngle = 5;
    private final double midHatchAngle = 30;
    private final double highHatchAngle = 30;
    private final double holdingAngle = 64;

    private ballArm ballArm;
    //this is since if you retract really fast before you give the elvator time to lift up, it won't grab the hatch
    /*
    -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    */

    private int initialTicks;
    private double currentTarget;

    private long lastTimeAPressed = 0;
    private long lastTimeXPressed = 0;
    private long lastTimeBPressed = 0;
    private long lastTimeYPressed = 0;

    //our hardware
    private CANSparkMax talon1;
    private CANSparkMax talon2;
    private Encoder encoder;
    private Joystick player1;
    private Joystick player2;
    private DigitalInput hatchLimitSwitch;
    private statusLights statusLights;

    public static enum theElevator {
        IDLE,
        intakeHatchPID, 
        lowHatchPID,
        midHatchPID,
        highHatchPID,
        holdingPID;
        private theElevator() {}
    }

    private theElevator elevator = theElevator.IDLE;

    public hatchElevator(CANSparkMax talon1, CANSparkMax talon2, Encoder encoder, Joystick player1, Joystick player2, 
        boolean talon2Inverted, ballArm ballArm, statusLights statusLights, DigitalInput hatchLimitSwitch) {
        this.talon1 = talon1;
        this.talon2 = talon2;
        this.encoder = encoder;
        this.talon2.setInverted(talon2Inverted);
        this.player1 = player1;
        this.player2 = player2;
        this.statusLights = statusLights;
        this.hatchLimitSwitch = hatchLimitSwitch;

        //this is the PID
        positionPID = new BantorPID(kV, kA, positionkP, positionkI, positionkD, velocitykP,
            velocitykI, velocitykD, dt, positionTolerance, velocityTolerance);
        positionPID.reset();

        findCurrentVelocity = new TorDerivative(dt);
        findCurrentVelocity.resetValue(0);
        this.ballArm = ballArm;
    }

    public void update(boolean running) {
        // SmartDashboard.putBoolean("limit switch being hit", limitSwitchBeingHit);
        talon2.follow(talon1);
		currentTime = (long)(Timer.getFPGATimestamp() * 1000);
		
		//this starting boolean makes it so that it will still do the first value in the trajectory
		
		//this handles it so that it will only tick in the time interval so that the derivatives and the integrals are correct
		if(((currentTime - startTime) - ((currentTime - startTime) % (dt * 1000))) > //has current time minus start time to see the relative time the trajectory has been going
		((lastCountedTime - startTime) - ((lastCountedTime - startTime) % (dt * 1000))) //subtracts that mod dt times 1000 so that it is floored to the nearest multiple of dt times 1000
		//then checks if that is greater than the last one to see if it is time to move on to the next tick
		|| starting) {
            SmartDashboard.putString("hatch elevator state:", elevator.toString());
			starting = false;
			lastCountedTime = currentTime;

            if(getButtonA()) {
                if((currentTime - lastTimeAPressed) > 250) {
                    if(!(elevator == theElevator.intakeHatchPID)) {
                        elevator = theElevator.intakeHatchPID;
                    } else {
                        elevator = theElevator.holdingPID;
                    }
                    lastTimeAPressed = currentTime;
                }
            } else if(getButtonX()) {
                if((currentTime - lastTimeXPressed) > 250) {
                    if(!(elevator == theElevator.lowHatchPID)) {
                        elevator = theElevator.lowHatchPID;
                    } else {
                        elevator = theElevator.holdingPID;
                    }
                    lastTimeXPressed = currentTime;
                }
            } else if(getButtonB()) {
                if((currentTime - lastTimeBPressed) > 250) {
                    if(!(elevator == theElevator.midHatchPID)) {
                        elevator = theElevator.midHatchPID;
                    } else {
                        elevator = theElevator.holdingPID;
                    }
                    lastTimeBPressed = currentTime;
                }
            } else if(getButtonY()) {
                if((currentTime - lastTimeYPressed) > 250) {
                    if(!(elevator == theElevator.highHatchPID)) {
                        elevator = theElevator.highHatchPID;
                    } else {
                        elevator = theElevator.holdingPID;
                    }
                    lastTimeYPressed = currentTime;
                }
            } else if(!running) {
                elevator = theElevator.IDLE;
            }

            if(testMode) {
                SmartDashboard.putNumber("encoder value", encoder.get());
                SmartDashboard.putNumber("height", height());
            } else {
                SmartDashboard.putNumber("current running speed:", currentRunningSpeed);
                SmartDashboard.putNumber("encoder value", encoder.get());
                SmartDashboard.putNumber("height", height());
                
                handleElevatorPosition();
                handleBallArmAngle();
                PIDRun();//this only updates what value the elevator should be going
                if(running) {
                    handleHatchShooter();
                    handleLimitSwitch();
                    setPercentSpeed(controlPower);
                    talon1.set(currentRunningSpeed);
                    talon2.set(currentRunningSpeed);
                    ballArm.update(currentRunningAngle);
                }
                SmartDashboard.putNumber("current running speed:", currentRunningSpeed);
                SmartDashboard.putNumber("current Position", height());
            }        
        }
    }

    public void handleLimitSwitch() {
        if(!hatchLimitSwitch.get() && elevator == theElevator.intakeHatchPID) {
            elevator = theElevator.holdingPID;
        }
    }

    public void handleElevatorPosition() {
        //this sets the current target
        //this should be low so that the elevator has a low "waiting" cg when you just enter hatch mode
        if(elevator == theElevator.intakeHatchPID) {
            currentTarget = intakeHatchPosition;
        } else if(elevator == theElevator.holdingPID || elevator == theElevator.IDLE) {
            currentTarget = holdHatchPosition;
        } else if(elevator == theElevator.lowHatchPID) {
            currentTarget = lowHatchPosition;
        } else if(elevator == theElevator.midHatchPID) {
            currentTarget = midHatchPosition;
        } else if(elevator == theElevator.highHatchPID) {
            currentTarget = highHatchPosition;
        }
    }

    public void handleBallArmAngle() {
        ballArm.setMode(0);
        if(elevator == theElevator.holdingPID || elevator == theElevator.IDLE) {
            currentRunningAngle = holdingAngle;
        } else if(elevator == theElevator.intakeHatchPID) {
            currentRunningAngle = intakeHatchAngle;
        } else if(elevator == theElevator.lowHatchPID) {
            currentRunningAngle = lowHatchAngle;
        } else if(elevator == theElevator.midHatchPID) {
            currentRunningAngle = midHatchAngle;
        } else if(elevator == theElevator.highHatchPID) {
            currentRunningAngle = highHatchAngle;
        }
    }

    public boolean topLimeLight() {
        return (elevator == theElevator.intakeHatchPID || elevator == theElevator.lowHatchPID
            || elevator == theElevator.holdingPID || elevator == theElevator.IDLE);
    }

    public void handleHatchShooter() {
        if(elevator == theElevator.intakeHatchPID) {
            ballArm.setMode(1, true);
        } else {
            if(getLeftBumper()) {
                ballArm.setMode(-3, true);
            } else {
                ballArm.setMode(0, true);
            }
        }
    }

    public void PIDRun() {//finds out the current velocity
        SmartDashboard.putNumber("hatch current target", currentTarget);
        currentVelocity = findCurrentVelocity.estimate(height());
        positionPID.updateTargets(currentTarget, targetVelocity, targetAcceleration);
        positionPID.updateCurrentValues(height(), currentVelocity);
        controlPower = positionPID.update();
        if(controlPower > absoluteMaxUpwardVelocity) {
            controlPower = absoluteMaxUpwardVelocity;
        } else if(controlPower < -absoluteMaxDownwardVelocity) {
            controlPower = -absoluteMaxDownwardVelocity;
        }
    }
    
    public void init(int initialValue) {
        initialTicks = encoder.get() - initialValue;
    }

    public void setPercentSpeed(double speed) {
        currentRunningSpeed = speed;
    }

    public double height() {
        return ((encoder.get() - initialTicks)/encoderTicksPerMeter);
    }


    /*
	 *  The following are a bunch of accessor methods to obtain input from the controller.
	 */
	public double getLeftX(){
		return player2.getRawAxis(0);
	}

	public double getLeftY(){
		return player2.getRawAxis(1);
	}

	public double getRightX(){
		return player2.getRawAxis(4);
	}

	public double getRightTrigger(){
		return player2.getRawAxis(3);
	}

	public boolean getLeftBumper(){
		return player2.getRawButton(5);
	}

	public boolean getRightBumper(){
		return player2.getRawButton(6);
	}

	public boolean getButtonA(){
		return player2.getRawButton(1);
	}

	public boolean getButtonB(){
		return player2.getRawButton(2);
	}

	public boolean getButtonX(){
		return player2.getRawButton(3);
	}

	public boolean getButtonY(){
		return player2.getRawButton(4);
	}
}
