package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class MotionProfileDriveController extends DriveController {
	private DriveHardware hardware;
	
	private double leftOutput;
	private double rightOutput;
	
	//compensates for lower batteries by multiplying it by 12(max voltage)/current voltage
	private double currentVoltage;
	private double conversionFactor;
	
	//the Derivatives to figure out the current Velocity
	private TorDerivative findCurrentVelocity;
	
	private TorDerivative findCurrentOmega;
	
	//the PIDs for the angles and the position
	private BantorPID positionPID;
	private BantorDriveHeadingPID headingPID;
	
	//these are the tolerance variables
	private double positionTolerance;
	private double velocityTolerance;
	
	private double headingTolerance;
	private double omegaTolerance;
	
	//this exists so that it will not miscount the first values on the trajectory
	private boolean starting = true;
	
	//since we want to allow it to get velocity and omega values without activating a trajectory, we need this boolean variable
	private boolean runningTrajectory = true;
	
	private long currentTime;
	private long startTime = (long)(Timer.getFPGATimestamp() * 1000);
	private double timeInterval = 0.005;
	private double dt = timeInterval;
	private long lastCountedTime;
	private long lookUpTime;
	private long timeOutTime;
	
	//the one trajectory that is running
	private TorTrajectory activeTrajectory;
	
	//these will be all the active real world things that are happening
	private double currentVelocity;
	
	private double currentOmega;
	
	//these will be all the active targets when running
	private double targetPosition;
	private double targetVelocity;
	private double targetAcceleration;
	
	private double targetHeading;
	private double targetOmega;
	private double targetAlpha;
	
	//the "instantiation" of all the closed loop constants
	private double positionkV;
	private double positionkA;
	private double headingkV;
	private double headingkA;

	private double PositionkP;
	private double PositionkI;
	private double PositionkD;
	
	private double VelocitykP;
	private double VelocitykI;
	private double VelocitykD;
	
	private double HeadingkP;
	private double HeadingkI;
	private double HeadingkD;
	
	private double OmegakP;
	private double OmegakI;
	private double OmegakD;
	
	//the end of it
	
	private double positionWayPoint = 0.0;
	private double headingWayPoint = 0.0;//for going to the next trajectories
	
	//the closed loop constants: 
	//the eighteen values to tune/calculate
	public void setClosedLoopConstants() {
		/*--------------------------------------------
		 * THE MOST IMPORTANT ONES TO TUNE FIRST
		 */
		positionkV = 1.0/hardware.getAbsoluteMaxVelocity();
		positionkA = 1.0/hardware.getAbsoluteMaxAcceleration();
		headingkV = 1.0/hardware.getAbsoluteMaxOmega();
		headingkA = 1.0/hardware.getAbsoluteMaxAcceleration();
		/*--------------------------------------------
		 * 
		 */
		PositionkP = 0.0;
		PositionkI = 0.0;
		PositionkD = 0.0;
		
		VelocitykP = 0.0;
		VelocitykI = 0.0;
		VelocitykD = 0.0;
		
		HeadingkP = 0.0;
		HeadingkI = 0.0;
		HeadingkD = 0.0;
		
		OmegakP = 0.0;
		OmegakI = 0.0;
		OmegakD = 0.0;
		
		positionTolerance = 0.0;
		velocityTolerance = 0.0;
		
		headingTolerance = 0.0;
		omegaTolerance = 0.0;
	}
	
	public MotionProfileDriveController(DriveHardware hardware, Joystick player1) {
		super(hardware, player1);	
		this.hardware = hardware;

		//sets all the tuning values so all the tunes will be in the setClosedLoopConstants void
		setClosedLoopConstants();
		
		//the PID's instantiation has to be after closed loop constants are set
		positionPID = new BantorPID(positionkV, positionkA, PositionkP, PositionkI, PositionkD,
				VelocitykP, VelocitykI, VelocitykD,
				dt, positionTolerance, velocityTolerance);
		positionPID.reset();
		
		headingPID = new BantorDriveHeadingPID(headingkV, headingkA, HeadingkP, HeadingkI, HeadingkD,
				OmegakP, OmegakI, OmegakD,
				dt, headingTolerance, omegaTolerance);
		headingPID.reset();
		
		//instantiates all the derivatives that will find the current values
		findCurrentVelocity = new TorDerivative(dt);
		
		findCurrentOmega = new TorDerivative(dt);
		
		//has to reset all the derivatives to 0 before we start
		findCurrentVelocity.resetValue(0);
		
		findCurrentOmega.resetValue(0);		
	}
	
	@Override
	public void run() {
		currentTime = (long)(Timer.getFPGATimestamp() * 1000);
		
		//this starting boolean makes it so that it will still do the first value in the trajectory
		
		//this handles it so that it will only tick in the time interval so that the derivatives and the integrals are correct
		if(((currentTime - startTime) - ((currentTime - startTime) % (dt * 1000))) > //has current time minus start time to see the relative time the trajectory has been going
		((lastCountedTime - startTime) - ((lastCountedTime - startTime) % (dt * 1000))) //subtracts that mod dt times 1000 so that it is floored to the nearest multiple of dt times 1000
		//then checks if that is greater than the last one to see if it is time to move on to the next tick
		|| starting) {
			starting = false;
			lastCountedTime = currentTime;
			
			//the lookUpTime is the current time minus the start time of the trajectory so that you have relative time
			//but, then you need to subtract what is is modulo dt since you need it to be a multiple of dt
			lookUpTime = (currentTime - startTime) - ((currentTime - startTime) % ((long)(dt * 1000)));
			
			if(runningTrajectory) {
				//figures out all the targets
				targetPosition = activeTrajectory.lookUpPosition(lookUpTime);
				targetPosition += positionWayPoint;//have to add the position of the last trajectory to move on
				targetVelocity = activeTrajectory.lookUpVelocity(lookUpTime);
				targetAcceleration = activeTrajectory.lookUpAcceleration(lookUpTime);
				
				targetHeading = activeTrajectory.lookUpHeading(lookUpTime);
				targetHeading += headingWayPoint;//have to add the heading of the last trajectory to move on
				targetOmega = activeTrajectory.lookUpOmega(lookUpTime);
				targetAlpha = activeTrajectory.lookUpAlpha(lookUpTime);
				
				//updates the targets on the PID's
				positionPID.updateTargets(targetPosition, targetVelocity, targetAcceleration);
				headingPID.updateTargets(targetHeading, targetOmega, targetAlpha);
				
				//finds out the current velocity
				currentVelocity = findCurrentVelocity.estimate(hardware.getPosition());
				
				//finds out the current omega
				currentOmega = findCurrentOmega.estimate(hardware.getHeading());
				
				//updates the PID's with the current velocity and omega.
				positionPID.updateCurrentValues(hardware.getPosition(), currentVelocity);
				headingPID.updateCurrentValues(hardware.getHeading(), currentOmega);
				
				//creates the conversion factor
				currentVoltage = hardware.getCurrentVoltage();
				conversionFactor = 12 / currentVoltage;
				
				//finally sets the hardware's targets
				setLeftOutput((positionPID.update() - headingPID.update()) * conversionFactor);
				setRightOutput((positionPID.update() + headingPID.update()) * conversionFactor);
				
				//when it finishes the PID's are on target and the lookUp is last
				//or it times out
				if(activeTrajectory.lookUpIsLast(lookUpTime) && ((positionPID.isOnTarget() && headingPID.isOnTarget()) || lookUpTime > timeOutTime)) {
					positionWayPoint = targetPosition;
					headingWayPoint = targetHeading;
					activeTrajectory.setComplete(true);
					positionPID.reset();
					headingPID.reset();
				}
			} else {
				//still figures out the current position and heading so that you can start the next trajectory easily by using the void execute trajectory
				positionWayPoint = hardware.getPosition();
				headingWayPoint = hardware.getHeading();
				//cuts off the last trajectory
				activeTrajectory.setComplete(true);
				//finds out the current velocity
				currentVelocity = findCurrentVelocity.estimate(hardware.getPosition());
				
				//finds out the current omega
				currentOmega = findCurrentOmega.estimate(hardware.getHeading());
				
				//keeps the PID's in check in case we want to activate them again
				//no way to find the current acceleration but that does not matter since the PID's do not
				//use acceleration in derivatives so we can set it to 0
				positionPID.updateTargets(hardware.getPosition(), currentVelocity, 0);
				headingPID.updateTargets(hardware.getHeading(), currentOmega, 0);
				//updates the PID's with the current velocity and omega.
				positionPID.updateCurrentValues(hardware.getPosition(), currentVelocity);
				headingPID.updateCurrentValues(hardware.getHeading(), currentOmega);			
			}
		}
	}

	@Override
	public double getLeftOutput() {
		// TODO Auto-generated method stub
		return leftOutput;
	}

	@Override
	public void setLeftOutput(double left) {
		// TODO Auto-generated method stub
		leftOutput = left;
	}

	@Override
	public double getRightOutput() {
		// TODO Auto-generated method stub
		return rightOutput;
	}

	@Override
	public void setRightOutput(double right) {
		// TODO Auto-generated method stub
		rightOutput = right;
	}
	
	public boolean activeTrajectoryIsComplete() {
		return activeTrajectory.isComplete();
	}
	
	public void stopRunningTrajectories() {
		runningTrajectory = false;
	}
	
	public void executeTrajectory(TorTrajectory nextTrajectory, long millisecondsToTimeOut) {//milliseconds till the trajectory times out
		nextTrajectory.setComplete(false);
		activeTrajectory = nextTrajectory;
		activeTrajectory.setComplete(false);
		startTime = (long)(Timer.getFPGATimestamp() * 1000);
		lastCountedTime = startTime;
		starting = true;
		timeOutTime = millisecondsToTimeOut;
		runningTrajectory = true;
	}
}
