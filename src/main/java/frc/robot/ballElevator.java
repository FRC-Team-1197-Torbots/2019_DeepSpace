package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ballElevator {
    //test or not?
    private final boolean testMode = true;

    private double currentRunningSpeed;

    private long currentTime;
	private long startTime = (long)(Timer.getFPGATimestamp() * 1000);
	private double timeInterval = 0.005;
	private double dt = timeInterval;
	private long lastCountedTime;
	private boolean starting = true;
    private BantorPID positionPID;
    private BantorPID overPositionPID;
    private double controlPower;//this is the amount of power the PID is giving out
    private TorDerivative findCurrentVelocity;
    private TorDerivative overfindCurrentVelocity;
    private double currentVelocity;
    private double overcurrentVelocity;

    /*
    tuneable variables------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    */
    //our variables
    private final double positionkP = 0.0;
    private final double positionkI = 0.0;
    private final double positionkD = 0.0;
    private final double positionTolerance = 0.01;//for thePID
    private final double velocitykP = 0.0;//velocity stuff probably not needed at all and should keep 0
    private final double velocitykI = 0.0;
    private final double velocitykD = 0.0;
    private final double kV = 0.0;
    private final double kA = 0.0;//this should definitely stay at 0
    private final double pidGoTolerance = 0.05;//this is in meters and should be kind of large as we are using bang bang till PID turns on
    private final double velocityTolerance = 0.0;
    private final double targetVelocity = 0.0;//probably won't need
    private final double targetAcceleration = 0.0;//probably won't need

    private final double overpositionkP = 0.0;
    private final double overpositionkI = 0.0;
    private final double overpositionkD = 0.0;
    private final double overpositionTolerance = 0.01;//for the overPID
    private final double overvelocitykP = 0.0;//velocity stuff probably not needed at all and should keep 0
    private final double overvelocitykI = 0.0;
    private final double overvelocitykD = 0.0;
    private final double overkV = 0.0;
    private final double overkA = 0.0;//this should definitely stay at 0
    private final double overvelocityTolerance = 0.0;
    private final double overtargetVelocity = 0.0;//probably won't need
    private final double overtargetAcceleration = 0.0;//probably won't need

    private final double overInAngle = 0;
    private final double overOutAngle = 45;

    private final double encoderTicksPerMeter = 1.0;//this is how many ticks there are per meter the elevator goes up
    private final double lowBallPosition = 1.0;//these three are the heights of what we want to go to
    private final double intakeBallPosition = 2.0;
    private final double highBallPosition = 3.0;
    private final double absoluteMaxUpwardVelocity = 1.0;//don't make it higher than 1.0 POSITIVE
    private final double absoluteMaxDownwardVelocity = 1.0;//don't make it higher than 1.0 POSITIVE

    private final double intakePower = 0.6;
    private final double outakePower = 0.4;
    private final double overIntakePower = -1.0;
    private final double highOutakePower = 0.7;

    private final boolean powerDrive = false;//this boolean is here so that we will go at a set speed when we are far away
    //if it is false then it will only use PID for power

    /*
    -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    */

    private double intakeMotorSpeed = 0.0;
    private int initialTicks;
    private double currentTarget;

    private long lastTimeAPressed = 0;
    private long lastTimeBPressed = 0;
    private long lastTimeYPressed = 0;
    //our hardware
    private TalonSRX talon1;
    private TalonSRX talon2;
    private TalonSRX intakeMotor1;
    private TalonSRX intakeMotor2;
    private TalonSRX overIntake1;//since the 971 intake only matters for this ball elevator
    //we can instantiate it in here
    private TalonSRX overIntake2;
    private TalonSRX overPull;
    private Potentiometer fourtwenty2;//IT IS THE POT

    private double overstartAngle;
    private double overcurrentAngle;
    private double overPower;
    
    private Encoder encoder;
    private Joystick player2;
    private Solenoid upPiston;

    private double overCurrentTarget;

    public static enum theElevator {
        IDLE, lowBallPID,
        intakeBallPID,
        highBallPID,
        goTolowBallPID, goTointakeBallPID, goToHighBallPID;
        private theElevator() {}
    }

    public theElevator elevator = theElevator.IDLE;

    public ballElevator(TalonSRX talon1, TalonSRX talon2, Encoder encoder, Joystick player2, 
        boolean talon2Inverted, TalonSRX intakeMotor1, TalonSRX intakeMotor2, boolean intakeMotor2Inverted, Solenoid upPiston) {
        this.talon1 = talon1;
        this.talon2 = talon2;
        this.encoder = encoder;
        this.talon2.follow(this.talon1);
        this.talon2.setInverted(talon2Inverted);
        this.player2 = player2;
        this.intakeMotor1 = intakeMotor1;
        this.intakeMotor2 = intakeMotor2;
        this.intakeMotor2.follow(this.intakeMotor1);
        this.intakeMotor2.setInverted(intakeMotor2Inverted);
        this.upPiston = upPiston;

        //this is the PID
        positionPID = new BantorPID(kV, kA, positionkP, positionkI, positionkD, velocitykP,
            velocitykI, velocitykD, dt, positionTolerance, velocityTolerance);
        positionPID.reset();

        findCurrentVelocity.resetValue(0);
        overfindCurrentVelocity.resetValue(0);

        //for the over the ball intake
        overIntake1 = new TalonSRX(10);
        overIntake2 = new TalonSRX(11);
        overPull = new TalonSRX(12);
        fourtwenty2 = new AnalogPotentiometer(0, 360, 0);
        overPositionPID = new BantorPID(overkV, overkA, overpositionkP, overpositionkI, overpositionkD, overvelocitykP,
        overvelocitykI, overvelocitykD, dt, overpositionTolerance, overvelocityTolerance);
        overPositionPID.reset();
    }

    public void update(boolean running, boolean limitSwitchBeingHit) {
		currentTime = (long)(Timer.getFPGATimestamp() * 1000);
		
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
                SmartDashboard.putNumber("potentiometer", fourtwenty2.get());
            } else {
                SmartDashboard.putNumber("encoder value", encoder.get());
                SmartDashboard.putNumber("height", height());
                if(getButtonA() && ((currentTime - lastTimeAPressed) > 250) && !(elevator == theElevator.goToHighBallPID || 
                    elevator == theElevator.goTolowBallPID || elevator == theElevator.goTointakeBallPID)) {
                    
                    lastTimeAPressed = currentTime;
                    elevator = theElevator.goTointakeBallPID;
                } else if(getButtonB() && ((currentTime - lastTimeBPressed) > 250) && !(elevator == theElevator.goToHighBallPID || 
                    elevator == theElevator.goTolowBallPID || elevator == theElevator.goTointakeBallPID)) {
                    
                    lastTimeBPressed = currentTime;
                    elevator = theElevator.goTolowBallPID;
                } else if(getButtonY() && ((currentTime - lastTimeYPressed) > 250) && !(elevator == theElevator.goToHighBallPID || 
                    elevator == theElevator.goTolowBallPID || elevator == theElevator.goTointakeBallPID)) {
                    
                    lastTimeYPressed = currentTime;
                    elevator = theElevator.goToHighBallPID;
                }
    
        
                //this sets the current target
                if(elevator == theElevator.IDLE) {
                    currentTarget = 0.1;//this should just be greater than 0 so it doesn't hit anything
                } else if(elevator == theElevator.lowBallPID) {
                    currentTarget = lowBallPosition;
                } else if(elevator == theElevator.intakeBallPID) {
                    currentTarget = intakeBallPosition;
                } else if(elevator == theElevator.highBallPID) {
                    currentTarget = highBallPosition;
                } else if(elevator == theElevator.goTointakeBallPID) {
                    currentTarget = intakeBallPosition;
                } else if(elevator == theElevator.goToHighBallPID) {
                    currentTarget = highBallPosition;
                } else if(elevator == theElevator.goTolowBallPID) {
                    currentTarget = lowBallPosition;
                }
                PIDRun();//this only updates what value the elevator should be going
                stateRun();
                handleIntake();
                if(running && !limitSwitchBeingHit) {
                    talon1.set(ControlMode.PercentOutput, currentRunningSpeed);
                } else {
                    talon1.set(ControlMode.PercentOutput, 0);
                }

                if(running) {
                    if(elevator == theElevator.intakeBallPID || elevator == theElevator.goTointakeBallPID) {
                        intakeMotor1.set(ControlMode.PercentOutput, intakeMotorSpeed);//auto intake for you
                    } else {
                        //you have to press a trigger to make it fire
                        if(player2.getRawButton(5)) {
                            intakeMotor1.set(ControlMode.PercentOutput, intakeMotorSpeed);
                        }
                    }
                }

                if(running) {
                    handlePneumatics();
                }

                //we don't need to check if we are running to do the 971 intake
                //it doesn't get into the way of anything
                handleOver();
            }        
        }
    }

    public void handleOver() {
        overcurrentAngle = fourtwenty2.get() - overstartAngle;

        overcurrentVelocity = overfindCurrentVelocity.estimate(overcurrentAngle);

        if(elevator == theElevator.intakeBallPID || elevator == theElevator.goTointakeBallPID) {
            overCurrentTarget = overOutAngle;
            overPull.set(ControlMode.PercentOutput, overIntakePower);
        } else {
            overCurrentTarget = overInAngle;
            overPull.set(ControlMode.PercentOutput, 0);
        }

        overPositionPID.updateTargets(overCurrentTarget, overtargetVelocity, overtargetAcceleration);
        overPositionPID.updateCurrentValues(overcurrentAngle, overcurrentVelocity);
        overPower = overPositionPID.update();

        overIntake1.set(ControlMode.PercentOutput, overPower);
        overIntake2.set(ControlMode.PercentOutput, -overPower);
    }

    public void handlePneumatics() {
        //this is as very simple class that just makes the piston for the elvator
        //turn up when player 2 presses X
        if(getButtonX()) {
            upPiston.set(true);
        } else {
            upPiston.set(false);
        }
    }

    public void handleIntake() {
        //this handles the intake motors intake motor speed to be set
        if(elevator == theElevator.IDLE) {
            intakeMotorSpeed = outakePower;
        } else if(elevator == theElevator.lowBallPID) {
            intakeMotorSpeed = outakePower;
        } else if(elevator == theElevator.intakeBallPID) {
            intakeMotorSpeed = -intakePower;
        } else if(elevator == theElevator.highBallPID) {
            if(getButtonX()) {
                intakeMotorSpeed = highOutakePower;
            } else {
                intakeMotorSpeed = outakePower;
            }
        } else if(elevator == theElevator.goTointakeBallPID) {
            intakeMotorSpeed = -intakePower;
        } else if(elevator == theElevator.goToHighBallPID) {
            if(getButtonX()) {
                intakeMotorSpeed = highOutakePower;
            } else {
                intakeMotorSpeed = outakePower;
            }
        } else if(elevator == theElevator.goTolowBallPID) {
            intakeMotorSpeed = outakePower;
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
            case lowBallPID:
                setPercentSpeed(controlPower);
                break;
            case intakeBallPID:
                setPercentSpeed(controlPower);
                break;
            case highBallPID:
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
        }
    }
    
    public void init() {
        initialTicks = encoder.get();
        overstartAngle = fourtwenty2.get();
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
