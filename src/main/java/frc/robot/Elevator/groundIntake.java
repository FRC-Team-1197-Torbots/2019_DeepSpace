package frc.robot.Elevator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

import frc.robot.Elevator.ballElevator.theElevator;
import frc.robot.PID_Tools.*;

import edu.wpi.first.wpilibj.Encoder;

public class groundIntake {
    //test or not?
    private final boolean testMode = false;

    private double currentRunningSpeed;

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

    private ballArm ballArm;
    private TalonSRX elevatorTalon1;
    private TalonSRX elevatorTalon2;
    private Joystick player2;
    private Encoder encoder;
    private int initialTicks;
    private double currentTarget;

    //elevator PID Stuff (should be same as the elevator PID in Hatch and Ball)
    private final double velocityTolerance = 0.0;
    private final double targetVelocity = 0.0;//probably won't need
    private final double targetAcceleration = 0.0;//probably won't need
    private final double positionkP = -2.1;
    private final double positionkI = 0.0;
    private final double positionkD = 0.0;
    private final double positionTolerance = 0.01;//for thePID
    private final double velocitykP = 0.0;//velocity stuff probably not needed at all and should keep 0
    private final double velocitykI = 0.0;
    private final double velocitykD = 0.0;
    private final double kV = 0.0;
    private final double kA = 0.0;//this should definitely stay at 0
    private final double encoderTicksPerMeter = 897;//this is how many ticks there are per meter the elevator goes up
    private final double absoluteMaxUpwardVelocity = 0.45;//don't make it higher than 1.0 POSITIVE
    private final double absoluteMaxDownwardVelocity = 1.0;//don't make it higher than 1.0 POSITIVE

    /*
    tunable values for this class (top should be the same)
    ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    */

    private final double intakeArmAngle = -2;//should stay flat when we intake the hatch
    private final double defaultArmAngle = 70;//this is the in angle
    private final double highPositionArmAngle = 60;
    private final double mediumPositionArmAngle = 70;
    private final double highPositionReleaseArmAngle = 50;
    private final double mediumPositionReleaseAngle = 50;

    private final double intakePosition = 0.0;
    private final double defaultPosition = 0.25;
    private final double highPosition = 0.68;
    private final double mediumPosition = 0.225;
    private final double highReleasePosition = 0.48;
    private final double mediumReleasePosition = 0.05;
    /*
    end of tunable values for this class
    ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    */
    private long lastTimeAPressed = 0;
    private long lastTimeBPressed = 0;
    private long lastTimeXPressed = 0;
    private long lastTimeYPressed = 0;

    public static enum theElevator {
        IDLE, 
        defaultPosition,
        intakePID,
        mediumPID,
        highPID,
        mediumReleasePID, highReleasePID;
        private theElevator() {}
    }

    public theElevator elevator = theElevator.defaultPosition;


    public groundIntake(TalonSRX elevatorTalon1, TalonSRX elevatorTalon2, ballArm ballArm, Joystick player2, Encoder encoder) {
        this.ballArm = ballArm;
        this.elevatorTalon1 = elevatorTalon1;
        this.elevatorTalon2 = elevatorTalon2;
        this.player2 = player2;
        this.encoder = encoder;

        //this is the PID
        positionPID = new BantorPID(kV, kA, positionkP, positionkI, positionkD, velocitykP,
        velocitykI, velocitykD, dt, positionTolerance, velocityTolerance);
        positionPID.reset();
     
        findCurrentVelocity = new TorDerivative(dt);
        findCurrentVelocity.resetValue(0);
    }

    public void update(boolean running, boolean limitSwitchBeingHit) {
        currentTime = (long)(Timer.getFPGATimestamp() * 1000);
		SmartDashboard.putString("ballElevator state", elevator.toString());
		//this starting boolean makes it so that it will still do the first value in the trajectory
		
		//this handles it so that it will only tick in the time interval so that the derivatives and the integrals are correct
		if(((currentTime - startTime) - ((currentTime - startTime) % (dt * 1000))) > //has current time minus start time to see the relative time the trajectory has been going
		((lastCountedTime - startTime) - ((lastCountedTime - startTime) % (dt * 1000))) //subtracts that mod dt times 1000 so that it is floored to the nearest multiple of dt times 1000
		//then checks if that is greater than the last one to see if it is time to move on to the next tick
		|| starting) {
			starting = false;
			lastCountedTime = currentTime;
            if(testMode) {
                SmartDashboard.putNumber("encoder value", encoder.get());
                SmartDashboard.putNumber("height", height());
            } else {
                SmartDashboard.putNumber("encoder value", encoder.get());
                SmartDashboard.putNumber("height", height());

                if(getButtonA() && (currentTime - lastTimeAPressed > 250)) {
                    lastTimeAPressed = currentTime;
                    if(elevator == theElevator.intakePID) {
                        elevator = theElevator.defaultPosition;
                    } else {
                        elevator = theElevator.intakePID;
                    }
                } else if(getButtonB() && (currentTime - lastTimeBPressed > 250)) {
                    lastTimeBPressed = currentTime;
                    elevator = theElevator.mediumPID;
                } else if(getButtonY() && (currentTime - lastTimeYPressed > 250)) {
                    lastTimeYPressed = currentTime;
                    elevator = theElevator.highPID;
                } else if(getButtonX() && (currentTime - lastTimeXPressed > 250)) {
                    lastTimeXPressed = currentTime;
                    if(elevator == theElevator.mediumPID) {
                        elevator = theElevator.mediumReleasePID;
                    } else if(elevator == theElevator.highPID) {
                        elevator = theElevator.highReleasePID;
                    } else if(elevator == theElevator.highReleasePID || elevator == theElevator.mediumReleasePID) {
                        elevator = theElevator.defaultPosition;
                    }
                    //those are all the states you should be in when you press X
                    //otherwise it won't work since it is only to outake the ground hatch
                }

                //handles the current target of the elevator PID
                if(elevator == theElevator.IDLE) {
                    currentTarget = -0.1;//this should just be greater than 0 so it doesn't hit anything
                } else if(elevator == theElevator.mediumPID) {
                    currentTarget = mediumPosition;
                } else if(elevator == theElevator.intakePID) {
                    currentTarget = intakePosition;
                } else if(elevator == theElevator.highPID) {
                    currentTarget = highPosition;
                } else if(elevator == theElevator.mediumReleasePID) {
                    currentTarget = mediumReleasePosition;
                } else if(elevator == theElevator.highReleasePID) {
                    currentTarget = highReleasePosition;
                } else if(elevator == theElevator.defaultPosition) {
                    currentTarget = defaultPosition;
                }
                PIDRun();//this only updates what value the elevator should be going at
                if(running) {
                    handleIntake();
                    handleArm();
                    SmartDashboard.putBoolean("Limitswitch being hit", limitSwitchBeingHit);
                    if(limitSwitchBeingHit) {
                        elevatorTalon1.set(ControlMode.PercentOutput, currentRunningSpeed);
                        elevatorTalon2.set(ControlMode.PercentOutput, currentRunningSpeed);
                    
                    }
                } else {
                    elevator = theElevator.defaultPosition;
                }
            }
        }
    }

    public void PIDRun() {//finds out the current velocity
        currentVelocity = findCurrentVelocity.estimate(height());
        positionPID.updateTargets(currentTarget, targetVelocity, targetAcceleration);
        positionPID.updateCurrentValues(height(), currentVelocity);
        controlPower = positionPID.update();
        if(controlPower > absoluteMaxUpwardVelocity) {
            controlPower = absoluteMaxUpwardVelocity;
        } else if(controlPower < -absoluteMaxDownwardVelocity) {
            controlPower = -absoluteMaxDownwardVelocity;
        }
        currentRunningSpeed = controlPower;
    }
    
    public void handleArm() {
        if(elevator == theElevator.IDLE || elevator == theElevator.defaultPosition) {
            ballArm.update(defaultArmAngle);
        } else if(elevator == theElevator.highPID) {
            ballArm.update(highPositionArmAngle);
        } else if(elevator == theElevator.highReleasePID) {
            ballArm.update(highPositionReleaseArmAngle);
        } else if(elevator == theElevator.mediumPID) {
            ballArm.update(mediumPositionArmAngle);
        } else if(elevator == theElevator.mediumReleasePID) {
            ballArm.update(mediumPositionReleaseAngle);
        } else if(elevator == theElevator.intakePID) {
            ballArm.update(intakeArmAngle);
        }
    }

    

    public void handleIntake() {
        //we should not run the ball intake spin motors when we do ground hatch
        ballArm.setMode(0);
    }


    public void init(int initialValue) {
        initialTicks = encoder.get() - initialValue;
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

	public boolean getShiftButton(){
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

