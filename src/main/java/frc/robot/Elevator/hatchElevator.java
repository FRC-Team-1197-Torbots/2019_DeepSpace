package frc.robot.Elevator;
import frc.robot.PID_Tools.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

public class hatchElevator {
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
    private final double pidGoTolerance = 1000;//this is in meters and should be kind of large as we are using bang bang till PID turns on
    private final double velocityTolerance = 0.0;
    private final double targetVelocity = 0.0;//probably won't need
    private final double targetAcceleration = 0.0;//probably won't need

    private final double encoderTicksPerMeter = 892;//this is how many ticks there are per meter the elevator goes up
    private final double intakeHatchPosition = 0.135;
    private final double highHatchPosition = 0.835;
    private final double intakeHatchExtendPosition = 0.055;//should be lower than intakeHatchPosition
    private final double highHatchExtendPosition = 0.755;//should be lower than highHatchPosition
    private final double absoluteMaxUpwardVelocity = 0.45;//don't make it higher than 1.0 POSITIVE
    private final double absoluteMaxDownwardVelocity = 1.0;//don't make it higher than 1.0 POSITIVE

    private final double pneumaticIntakeWaitTime = 1 * 1000;//you need to have a small pause between the pneumatic and elevator when you intake
    private final double pneumaticOutakeWaitTime = 2.0 * 1000;

    //ball arm
    private final double upAngleForBallArm = 62;
    private final double downAngleForIntakeArm = 59;
    private final double downAngleForExtendArm = 56;
    private ballArm ballArm;
    //this is since if you retract really fast before you give the elvator time to lift up, it won't grab the hatch
    /*
    -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    */

    private final boolean powerDrive = false;//this boolean is here so that we will go at a set speed when we are far away
    //if it is false then it will only use PID for power

    private int initialTicks;
    private double currentTarget;

    private long lastTimeAPressed = 0;
    private long lastTimeYPressed = 0;
    private long hatchUpStartTime = 0;
    private long lowHatchDownStartTime = 0;
    private long highHatchDownStartTime = 0;

    private boolean XPressedLast = false;
    private boolean BPressedLast = false;

    //our hardware
    private CANSparkMax talon1;
    private CANSparkMax talon2;
    private Encoder encoder;
    private Joystick player1;
    private Joystick player2;
    private Solenoid piston;
    private statusLights statusLights;

    public static enum theElevator {
        IDLE,
        intakeHatchPID, intakeHatchExtendPID, intakeHatchUpPID, //we need this extra state here since pneumatics are faster than the elevator
        //if we don't have it, the pneumatic will retract before the elevator goes up and it won't work
        lowHatchDownPID,
        highHatchPID, highHatchExtendPID,
        highHatchDownPID,
        goTointakeHatchPID, goToHighHatchPID;
        private theElevator() {}
    }

    private theElevator elevator = theElevator.intakeHatchPID;

    public hatchElevator(CANSparkMax talon1, CANSparkMax talon2, Encoder encoder, Joystick player1, Joystick player2, boolean talon2Inverted, Solenoid piston, ballArm ballArm, statusLights statusLights) {
        this.talon1 = talon1;
        this.talon2 = talon2;
        this.encoder = encoder;
        this.talon2.setInverted(talon2Inverted);
        this.player1 = player1;
        this.player2 = player2;
        this.piston = piston;
        this.statusLights = statusLights;

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
            SmartDashboard.putBoolean("got here 1", true);
            SmartDashboard.putString("hatch elevator state:", elevator.toString());
			starting = false;
			lastCountedTime = currentTime;

            if(testMode) {
                SmartDashboard.putNumber("encoder value", encoder.get());
                SmartDashboard.putNumber("height", height());
            } else {
                SmartDashboard.putNumber("current running speed:", currentRunningSpeed);
                SmartDashboard.putNumber("encoder value", encoder.get());
                SmartDashboard.putNumber("height", height());
                if(getButtonX()) {
                    XPressedLast = true;
                    //button X will be our extend position
                    //if it is intake mode, it will go down and extend
                    //if it is in a mode where it will put the hatch in, it will up and extend to let it in
                    //when you let go of X in intake mode, it will go up and retract to take the hatch
                    //when you let go of X in outake mode, it will go down and retract to let the mechanism come out
                    if(elevator == theElevator.intakeHatchPID || elevator == theElevator.goTointakeHatchPID) {
                        elevator = theElevator.intakeHatchExtendPID;
                    }
                    // } else if(elevator == theElevator.highHatchPID || elevator == theElevator.goToHighHatchPID) {
                    //     elevator = theElevator.highHatchExtendPID;
                    // }
                } else if(XPressedLast) {
                    if(elevator == theElevator.intakeHatchExtendPID) {
                        hatchUpStartTime = currentTime;
                        elevator = theElevator.intakeHatchUpPID;
                    }
                    // else if(elevator == theElevator.highHatchExtendPID) {
                    //     elevator = theElevator.goToHighHatchPID;
                    // }
                    XPressedLast = false;
                } else if(getButtonB()) {
                    BPressedLast = true;
                    //button X will be our extend position
                    //if it is intake mode, it will go down and extend
                    //if it is in a mode where it will put the hatch in, it will up and extend to let it in
                    //when you let go of X in intake mode, it will go up and retract to take the hatch
                    //when you let go of X in outake mode, it will go down and retract to let the mechanism come out
                    lowHatchDownStartTime = currentTime;
                    highHatchDownStartTime = currentTime;
                    if(elevator == theElevator.intakeHatchPID || elevator == theElevator.goTointakeHatchPID) {
                        elevator = theElevator.lowHatchDownPID;
                    } else if(elevator == theElevator.highHatchPID || elevator == theElevator.goToHighHatchPID) {
                        elevator = theElevator.highHatchDownPID;
                    }
                } else if(getButtonA() && ((currentTime - lastTimeAPressed) > 250) && !(elevator == theElevator.goToHighHatchPID 
                    || elevator == theElevator.goTointakeHatchPID || elevator == theElevator.intakeHatchUpPID)) {
                    
                    lastTimeAPressed = currentTime;
                    elevator = theElevator.goTointakeHatchPID;
                } else if(getButtonY() && ((currentTime - lastTimeYPressed) > 250) && !(elevator == theElevator.goToHighHatchPID 
                || elevator == theElevator.goTointakeHatchPID || elevator == theElevator.intakeHatchUpPID)) {
                    
                    lastTimeYPressed = currentTime;
                    elevator = theElevator.goToHighHatchPID;
                } else if(!getButtonB()) {
                    if(elevator == theElevator.lowHatchDownPID && (currentTime - lowHatchDownStartTime > pneumaticOutakeWaitTime)) {
                        // lowHatchDownStartTime = currentTime;
                        elevator = theElevator.intakeHatchPID;
                    } else if(elevator == theElevator.highHatchDownPID && 
                        ((currentTime - highHatchDownStartTime) > pneumaticOutakeWaitTime)) {
                        // highHatchDownStartTime = currentTime;
                        elevator = theElevator.highHatchPID;
                    }
                    BPressedLast = false;
                }
        
                //this sets the current target
                if(elevator == theElevator.IDLE) {
                    currentTarget = intakeHatchPosition;//this should be low so that the elevator has a low "waiting" cg when you just enter hatch mode
                } else if(elevator == theElevator.intakeHatchPID) {
                    currentTarget = intakeHatchPosition;
                } else if(elevator == theElevator.intakeHatchExtendPID) {
                    currentTarget = intakeHatchExtendPosition;
                } else if(elevator == theElevator.intakeHatchUpPID) {
                    currentTarget = intakeHatchPosition;
                } else if(elevator == theElevator.highHatchPID) {
                    currentTarget = highHatchPosition;
                } else if(elevator == theElevator.highHatchExtendPID) {
                    currentTarget = highHatchExtendPosition;
                } else if(elevator == theElevator.goTointakeHatchPID) {
                    currentTarget = intakeHatchPosition;
                } else if(elevator == theElevator.goToHighHatchPID) {
                    currentTarget = highHatchPosition;
                } else if(elevator == theElevator.highHatchDownPID) {
                    if(getButtonB()) {
                        currentTarget = highHatchPosition;
                    } else {
                        currentTarget = highHatchExtendPosition;
                    }
                } else if(elevator == theElevator.lowHatchDownPID) {
                    if(getButtonB()) {
                        currentTarget = intakeHatchPosition;
                    } else {
                        currentTarget = intakeHatchExtendPosition;
                    }
                }
                PIDRun();//this only updates what value the elevator should be going
                stateRun();
                if(running) {
                    handleSolenoid();
                }
                SmartDashboard.putNumber("current running speed:", currentRunningSpeed);
                SmartDashboard.putNumber("current Position", height());
                if(running) {
                    ballArm.setMode(0);
                    if(elevator == theElevator.highHatchDownPID || elevator == theElevator.highHatchExtendPID
                    || elevator == theElevator.intakeHatchExtendPID || elevator == theElevator.lowHatchDownPID) {
                        if(elevator == theElevator.intakeHatchExtendPID) {
                            ballArm.update(downAngleForIntakeArm);
                        } else {
                            ballArm.update(downAngleForExtendArm);
                        }
                    } else {
                        ballArm.update(upAngleForBallArm);
                    }
                    talon1.set(currentRunningSpeed);
                    talon2.set(currentRunningSpeed);
                } else {
                    elevator = theElevator.IDLE;
                }
            }        
        }
    }

    public boolean topLimeLight() {
        return (elevator == theElevator.goTointakeHatchPID || elevator == theElevator.intakeHatchExtendPID
            || elevator == theElevator.intakeHatchPID || elevator == theElevator.intakeHatchUpPID
            || elevator == theElevator.lowHatchDownPID);
    }

    public void handleSolenoid() {
        //this sets the piston 
        // red if actuated, green extended
        // if(elevator == theElevator.IDLE) {
        //     piston.set(false);
        //     statusLights.displayGreenLights();
        // } else if(elevator == theElevator.intakeHatchPID) {
        //     piston.set(false);
        //     statusLights.displayGreenLights();
        // } else if(elevator == theElevator.intakeHatchExtendPID) {
        //     piston.set(true);
        //     statusLights.displayRedLights();
        // } else if(elevator == theElevator.intakeHatchUpPID) {
        //     piston.set(true);
        //     statusLights.displayRedLights();
        // } else if(elevator == theElevator.highHatchPID) {
        //     piston.set(false);
        //     statusLights.displayGreenLights();
        // } else if(elevator == theElevator.highHatchExtendPID) {
        //     piston.set(true);
        //     statusLights.displayRedLights();
        // } else if(elevator == theElevator.goTointakeHatchPID) {
        //     piston.set(false);
        //     statusLights.displayGreenLights();
        // } else if(elevator == theElevator.goToHighHatchPID) {
        //     piston.set(false);
        //     statusLights.displayGreenLights();
        // }
        if(getButtonX() || getButtonB()) {
            piston.set(true);
            statusLights.displayRedLights();
        } else {
            if(elevator == theElevator.highHatchDownPID || elevator == theElevator.lowHatchDownPID) {
                if(currentTime - lowHatchDownStartTime < 1000 || currentTime - highHatchDownStartTime < 1000) {
                    piston.set(true);
                } else {
                    piston.set(false);
                }
            } else {
                if(elevator == theElevator.intakeHatchUpPID) {
                    piston.set(true);
                } else {
                    piston.set(false);
                }
            }
            statusLights.displayGreenLights();
            
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

    public void stateRun() {
        switch(elevator) {
            case IDLE:
                setPercentSpeed(controlPower);
                break;
            case lowHatchDownPID:
                setPercentSpeed(controlPower);
                break;
            case highHatchDownPID:
                setPercentSpeed(controlPower);
                break;
            case intakeHatchPID:
                setPercentSpeed(controlPower);
                break;
            case intakeHatchExtendPID:
                setPercentSpeed(controlPower);
                break;
            case intakeHatchUpPID:
                setPercentSpeed(controlPower);
                if((currentTime - hatchUpStartTime) >= pneumaticIntakeWaitTime) {
                    elevator = theElevator.intakeHatchPID;
                }
                break;
            case highHatchPID:
                setPercentSpeed(controlPower);
                break;
            case highHatchExtendPID:
                setPercentSpeed(controlPower);
                break;
            case goTointakeHatchPID:
                if(Math.abs(height() - currentTarget) > pidGoTolerance) {
                    if((height() - currentTarget) > 0) {
                        if(powerDrive) {
                            setPercentSpeed(-absoluteMaxDownwardVelocity);
                        } else {
                            setPercentSpeed(controlPower);
                        }
                    } else {
                        if(powerDrive) {
                            setPercentSpeed(absoluteMaxUpwardVelocity);
                        } else {
                            setPercentSpeed(controlPower);
                        }
                    }
                } else {
                    setPercentSpeed(controlPower);
                    elevator = theElevator.intakeHatchPID;
                }
                break;
            case goToHighHatchPID:
                if(Math.abs(height() - currentTarget) > pidGoTolerance) {
                    if((height() - currentTarget) > 0) {
                        if(powerDrive) {
                            setPercentSpeed(-absoluteMaxDownwardVelocity);
                        } else {
                            setPercentSpeed(controlPower);
                        }
                    } else {
                        if(powerDrive) {
                            setPercentSpeed(absoluteMaxUpwardVelocity);
                        } else {
                            setPercentSpeed(controlPower);
                        }
                    }
                } else {
                    setPercentSpeed(controlPower);
                    elevator = theElevator.highHatchPID;
                }
                break;
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
