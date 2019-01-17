package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class Elevator {
    private long currentTime;
	private long startTime = (long)(Timer.getFPGATimestamp() * 1000);
	private double timeInterval = 0.005;
	private double dt = timeInterval;
	private long lastCountedTime;
	private boolean starting = true;
    private BantorPID positionPID;
    private double controlPower;//this is the amount of power the PID is giving out

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
    private final double pidGoTolerance = 0.3;//this is in meters and should be kind of large as we are using bang bang till PID turns on
    private final double velocityTolerance = 0.0;

    private final double encoderTicksPerMeter = 1.0;//this is how many ticks there are per meter the elevator goes up
    private final double lowHatchPosition = 1.0;//these three are the heights of what we want to go to
    private final double intakeHatchPosition = 2.0;
    private final double highHatchPosition = 3.0;
    private final double lowHatchExtendPosition = 0.8;//should be lower than lowHatchPosition
    private final double intakeHatchExtendPosition = 2.2;//should be higher than intakeHatchPosition
    private final double highHatchExtendPosition = 2.8;//should be low than highHatchPosition
    private final double absoluteMaxUpwardVelocity = 1.0;//don't make it higher than 1.0
    private final double absoluteMaxDownwardVelocity = 1.0;//don't make it higher than 1.0
    /*
    -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    */

    private int initialTicks;
    private double currentTarget;

    //our hardware
    private TalonSRX talon1;
    private TalonSRX talon2;
    private Encoder encoder;
    private Joystick player2;

    public static enum theElevator {
        IDLE, lowHatchPID, lowHatchExtendPID,
        intakeHatchPID, intakeHatchExtendPID, intakeHatchUpPID, //we need this extra state here since pneumatics are faster than the elevator
        //if we don't have it, the pneumatic will retract before the elevator goes up and it won't work
        highHatchPID, highHatchExtendPID,
        goTolowHatchPID, goTointakeHatchPID, goToHighHatchPID;
        private theElevator() {}
    }

    public theElevator elevator = theElevator.IDLE;

    public Elevator(int motorPort1, int motorPort2, int encoderPort1, int encoderPort2, Joystick player2, boolean talon2Inverted) {
        this.talon1 = new TalonSRX(motorPort1);
        this.talon2 = new TalonSRX(motorPort2);
        this.encoder = new Encoder(encoderPort1, encoderPort2);
        this.talon2.follow(talon1);
        this.talon2.setInverted(talon2Inverted);
        this.player2 = player2;

        //this is the PID
        positionPID = new BantorPID(kV, kA, positionkP, positionkI, positionkD, velocitykP,
            velocitykI, velocitykD, dt, positionTolerance, velocityTolerance);
    }

    public void update() {
		currentTime = (long)(Timer.getFPGATimestamp() * 1000);
		
		//this starting boolean makes it so that it will still do the first value in the trajectory
		
		//this handles it so that it will only tick in the time interval so that the derivatives and the integrals are correct
		if(((currentTime - startTime) - ((currentTime - startTime) % (dt * 1000))) > //has current time minus start time to see the relative time the trajectory has been going
		((lastCountedTime - startTime) - ((lastCountedTime - startTime) % (dt * 1000))) //subtracts that mod dt times 1000 so that it is floored to the nearest multiple of dt times 1000
		//then checks if that is greater than the last one to see if it is time to move on to the next tick
		|| starting) {
			starting = false;
			lastCountedTime = currentTime;

            if(getButtonX()) {
                //button X will be our extend position
                //if it is intake mode, it will go down and extend
                //if it is in a mode where it will put the hatch in, it will up and extend to let it in
                //when you let go of X in intake mode, it will go up and retract to take the hatch
                //when you let go of X in outake mode, it will go down and retract to let the mechanism come out

            }

    
            //this sets the current target
            if(elevator == theElevator.IDLE) {
                currentTarget = 0.1;//this should just be greater than 0 so it doesn't hit anything
            } else if(elevator == theElevator.lowHatchPID) {
                currentTarget = lowHatchPosition;
            } else if(elevator == theElevator.lowHatchExtendPID) {
                currentTarget = lowHatchExtendPosition;
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
            } else if(elevator == theElevator.goTolowHatchPID) {
                currentTarget = lowHatchPosition;
            }
            PIDRun();//this only updates what value the elevator should be going
            stateRun();
        }
    }

    public void PIDRun() {
        
    }

    public void stateRun() {
        switch(elevator) {
            case IDLE:
                break;
            case lowHatchPID:
                break;
            case lowHatchExtendPID:
                break;
            case intakeHatchPID:
                break;
            case intakeHatchExtendPID:
                break;
            case intakeHatchUpPID:
                break;
            case highHatchPID:
                break;
            case highHatchExtendPID:
                break;
            case goTolowHatchPID:
                break;
            case goTointakeHatchPID:
                break;
            case goToHighHatchPID:
                break;
        }
    }
    
    public void init() {
        initialTicks = encoder.get();
    }

    public void setPercentSpeed(double speed) {
        talon1.set(ControlMode.PercentOutput, speed);
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
