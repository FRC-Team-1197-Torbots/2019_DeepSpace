package frc.robot.Elevator;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
// import com.sun.jndi.url.rmi.rmiURLContext;

import frc.robot.PID_Tools.*;



import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

public class ballElevator {
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
    private final double lowBallPosition = 0.17;//these three are the heights of what we want to go to
    private final double mediumBallPosition = 0.85;
    private final double intakeBallPosition = 0.275;
    private final double highBallPosition = 0.85;
    private final double cargoBallPosition = 0.85;
    private final double defaultPosition = 0.3;//should be low so limelight can see and center of gravity isn't too high
    private final double absoluteMaxUpwardVelocity = 0.5;//don't make it higher than 1.0 POSITIVE
    private final double absoluteMaxDownwardVelocity = 1.0;//don't make it higher than 1.0 POSITIVE

    //for the ballArm positions
    private final double intakeBallAngle = -35;//we want to intake at a downwards angle to minimize grabbing more than one ball
    private final double highBallAngle = 70;
    private final double mediumBallAngle = 31;
    private final double lowBallAngle = 27.5;
    private final double cargoBallAngle = -2;
    private final double pulledInAngle = 68;//inside the frame for protection

    private final boolean powerDrive = false;//this boolean is here so that we will go at a set speed when we are far away
    //if it is false then it will only use PID for power

    /*
    -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    */
    private int initialTicks;
    private double currentTarget;

    private long lastTimeAPressed = 0;
    private long lastTimeBPressed = 0;
    private long lastTimeXPressed = 0;
    private long lastTimeYPressed = 0;
    private long lastTimeDPressed = 0;
    //our hardware
    // private TalonSRX talon1;
    // private TalonSRX talon2;
    private CANSparkMax talon1;
    private CANSparkMax talon2;
    private ballArm ballArm;
    private statusLights statusLights;
    private DigitalInput ballBreakBeam;
    
    private Encoder encoder;
    private Joystick player2;

    public static enum theElevator {
        IDLE, lowBallPID,
        intakeBallPID,
        highBallPID,
        cargoBallPID,
        mediumBallPID,
        defaultPosition,
        goTolowBallPID, goTointakeBallPID, goToHighBallPID, goTomediumBallPID, goToCargoBallPID;
        private theElevator() {}
    }

    public theElevator elevator = theElevator.defaultPosition;

    public ballElevator(CANSparkMax talon1, CANSparkMax talon2, Encoder encoder, Joystick player2, 
        boolean talon2Inverted, DigitalInput ballBreakBeam, ballArm ballArm, statusLights statusLights) {
        this.talon1 = talon1;
        this.talon2 = talon2;
        this.encoder = encoder;
        this.talon2.follow(this.talon1);
        this.talon2.setInverted(talon2Inverted);
        this.player2 = player2;
        this.ballBreakBeam = ballBreakBeam;
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
        SmartDashboard.putString("ball elevator state:", elevator.toString());
        talon2.follow(talon1);
		currentTime = (long)(Timer.getFPGATimestamp() * 1000);
        SmartDashboard.putString("ballElevator state", elevator.toString());
        SmartDashboard.putBoolean("d pad: ", getDpad());
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
                if(getButtonA() && ((currentTime - lastTimeAPressed) > 250)) {
                    
                    lastTimeAPressed = currentTime;
                    if(elevator == theElevator.goTointakeBallPID || elevator == theElevator.intakeBallPID) {
                        elevator = theElevator.defaultPosition;
                    } else {
                        elevator = theElevator.goTointakeBallPID;
                    }
                } else if(getButtonB() && ((currentTime - lastTimeBPressed) > 250)) {
                    
                    lastTimeBPressed = currentTime;
                    if(elevator == theElevator.goTolowBallPID || elevator == theElevator.lowBallPID) {
                        elevator = theElevator.defaultPosition;
                    } else {
                        elevator = theElevator.goTolowBallPID;
                    }
                } else if(getButtonX() && ((currentTime - lastTimeXPressed) > 250)) {
                    
                    lastTimeXPressed = currentTime;
                    if(elevator == theElevator.goTomediumBallPID || elevator == theElevator.mediumBallPID) {
                        elevator = theElevator.defaultPosition;
                    } else {
                        elevator = theElevator.mediumBallPID;
                    }
                } else if(getButtonY() && ((currentTime - lastTimeYPressed) > 250)) {
                    
                    lastTimeYPressed = currentTime;
                    if(elevator == theElevator.goToHighBallPID || elevator == theElevator.highBallPID) {
                        elevator = theElevator.defaultPosition;
                    } else {
                        elevator = theElevator.goToHighBallPID;
                    }
                } else if(getDpad() && ((currentTime - lastTimeDPressed) > 250)) {

                    lastTimeDPressed = currentTime;
                    if(elevator == theElevator.goToCargoBallPID || elevator == theElevator.cargoBallPID) {
                        elevator = theElevator.defaultPosition;
                    } else {
                        elevator = theElevator.goToCargoBallPID;
                    }
                }
    
        
                //this sets the current target
                if(elevator == theElevator.IDLE) {
                    currentTarget = intakeBallPosition;//this should just be greater than 0 so it doesn't hit anything
                } else if(elevator == theElevator.lowBallPID) {
                    currentTarget = lowBallPosition;
                } else if(elevator == theElevator.intakeBallPID) {
                    currentTarget = intakeBallPosition;
                } else if(elevator == theElevator.highBallPID) {
                    currentTarget = highBallPosition;
                } else if(elevator == theElevator.mediumBallPID) {
                    currentTarget = mediumBallPosition;
                } else if(elevator == theElevator.goTointakeBallPID) {
                    currentTarget = intakeBallPosition;
                } else if(elevator == theElevator.goToHighBallPID) {
                    currentTarget = highBallPosition;
                } else if(elevator == theElevator.goTolowBallPID) {
                    currentTarget = lowBallPosition;
                } else if(elevator == theElevator.goTomediumBallPID) {
                    currentTarget = mediumBallPosition;
                } else if(elevator == theElevator.defaultPosition) {
                    currentTarget = defaultPosition;
                } else if(elevator == theElevator.goToCargoBallPID) {
                    currentTarget = cargoBallPosition;
                } else if(elevator == theElevator.cargoBallPID) {
                    currentTarget = cargoBallPosition;
                }
                PIDRun();//this only updates what value the elevator should be going at
                stateRun();
                if(running) {
                    handleIntake();
                    handleArm();
                    // if(!limitSwitchBeingHit) {
                        talon1.set(currentRunningSpeed);
                        talon2.set(currentRunningSpeed);

                    // }
                } else {
                    elevator = theElevator.defaultPosition;
                }
            }        
        }
    }

    public void handleArm() {
        if(elevator == theElevator.IDLE || elevator == theElevator.defaultPosition || elevator == theElevator.goToCargoBallPID) {
            ballArm.update(pulledInAngle);
        } else if(elevator == theElevator.goToHighBallPID || elevator == theElevator.highBallPID) {
            ballArm.update(highBallAngle);
        } else if(elevator == theElevator.goTomediumBallPID || elevator == theElevator.mediumBallPID) {
            ballArm.update(mediumBallAngle);
        } else if(elevator == theElevator.goTolowBallPID || elevator == theElevator.lowBallPID) {
            ballArm.update(lowBallAngle);
        } else if(elevator == theElevator.intakeBallPID || elevator == theElevator.goTointakeBallPID) {
            ballArm.update(intakeBallAngle);
        } else if(elevator == theElevator.cargoBallPID) {
            ballArm.update(cargoBallAngle);
        }
    }

    public void handleIntake() {
        //this handles the intake motors intake motor speed to be set
        if(elevator == theElevator.IDLE || elevator == theElevator.defaultPosition) {
            ballArm.setMode(0);
            statusLights.displayWhiteLights();
        } else if(elevator == theElevator.intakeBallPID || elevator == theElevator.goTointakeBallPID) {
            ballArm.setMode(-1); //if in intake position, spin wheels in
            statusLights.displayCyanLights();
        } else if (elevator == theElevator.lowBallPID || elevator == theElevator.goTolowBallPID){
            statusLights.displayCyanLights();
            if(player2.getRawButton(5)) {
                ballArm.setMode(2); //if pressed button spin out ball slowly
            } else if (Math.abs(player2.getRawAxis(2)) > 0.1){
                ballArm.setMode(-2); // if pressed LT, spin in to suck in ball
                
            } else {
                ballArm.setMode(0);
            }
        } else if (elevator == theElevator.mediumBallPID || elevator == theElevator.goTomediumBallPID){
            statusLights.displayCyanLights();
            if(player2.getRawButton(5)) {
                ballArm.setMode(2); // if pressed button LB, spin out ball slowly
            } else if (Math.abs(player2.getRawAxis(2)) > 0.1){
                ballArm.setMode(-2); // if pressed LT, spin in to suck in ball
                
            } else {
                ballArm.setMode(0);
            }
        } else if (elevator == theElevator.highBallPID || elevator == theElevator.goToHighBallPID){
            statusLights.displayCyanLights();
            if(player2.getRawButton(5)) {
                ballArm.setMode(1); // if pressed LB, spin out ball fast since 3rd level
            } else if (Math.abs(player2.getRawAxis(2)) > 0.1){
                ballArm.setMode(-2); // if pressed LT, spin in to suck in ball
                
            } else {
                ballArm.setMode(0); 
            }
        } else if (elevator == theElevator.cargoBallPID || elevator == theElevator.goToCargoBallPID) {
            statusLights.displayCyanLights();
            if(player2.getRawButton(5)) {
                ballArm.setMode(2); // if pressed LB, spin out ball slowly
            } else if (Math.abs(player2.getRawAxis(2)) > 0.1){
                ballArm.setMode(-2); // if pressed LT, spin in to suck in ball
                
            } else {
                ballArm.setMode(0); 
            }
        } else {
            ballArm.setMode(0);
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
    }

    public void stateRun() {
        switch(elevator) {
            case IDLE:
                break;
            case defaultPosition:
                setPercentSpeed(controlPower);
                break;
            case lowBallPID:
                setPercentSpeed(controlPower);
                break;
            case mediumBallPID:
                setPercentSpeed(controlPower);
                break;
            case intakeBallPID:
                setPercentSpeed(controlPower);
                if(!ballBreakBeam.get()) {
                    elevator = theElevator.defaultPosition;
                }
                break;
            case highBallPID:
                setPercentSpeed(controlPower);
                break;
            case cargoBallPID:
                setPercentSpeed(controlPower);
                break;
            case goTolowBallPID:
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
                    elevator = theElevator.lowBallPID;
                }
                break;
            case goTomediumBallPID:
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
                    elevator = theElevator.lowBallPID;
                }
                break;
            case goTointakeBallPID:
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
                    elevator = theElevator.intakeBallPID;
                }
                break;
            case goToHighBallPID:
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
                    elevator = theElevator.highBallPID;
                }
                break;
            case goToCargoBallPID:
                // if(Math.abs(height() - currentTarget) > pidGoTolerance) {
                //     if((height() - currentTarget) > 0) {
                //         if(powerDrive) {
                //             setPercentSpeed(-absoluteMaxDownwardVelocity);
                //         } else {
                //             setPercentSpeed(controlPower);
                //         }
                //     } else {
                //         if(powerDrive) {
                //             setPercentSpeed(absoluteMaxUpwardVelocity);
                //         } else {
                //             setPercentSpeed(controlPower);
                //         }
                //     }
                // } else {
                setPercentSpeed(controlPower);
                if(height() >= (currentTarget - 0.2)) {
                    elevator = theElevator.cargoBallPID;
                }
                // }
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

    public boolean topLimelight() {
        return (elevator == theElevator.intakeBallPID || elevator == theElevator.goTolowBallPID
            || elevator == theElevator.lowBallPID
            || elevator == theElevator.goTointakeBallPID);
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
    public int getDPad(){
        return player2.getPOV();
    }

    public boolean getDpad() {
        return (player2.getPOV() >= 0);
    }
}
