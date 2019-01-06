package frc.robot;

public class BantorPID {
	//instantiation of all the constants
	private double kV;
	private double kA;
	
	private double PositionkP;
	private double PositionkI;
	private double PositionkD;

	private double output;
	
	private double dt;
	
	//instantiation of the actively changing variables
	private double activePositionTarget;
	private double activeVelocityTarget;
	private double activeAccelerationTarget;
	
	//instantiation of all the actual real world values coming in
	private double currentPosition;
	private double currentVelocity;
	
	//the active results for each PID
	private double PositionPIDResult;
	
	//the derivatives to handle the derivative part
	private TorDerivative positionDerivative;
	private TorDerivative velocityDerivative;
	
	//these are the error calculations to calculate out the current errors
	private double positionError;
	private double velocityError;
	
	//the derivatives
	private double positionDerivativeResult;
	
	//the integral outputs
	private double positionIntegralResult;
	
	//the tolerance variables
	private double positionTolerance;
	private double velocityTolerance;
	
	//the booleans to see if they are on target
	private boolean positionOnTarget;
	private boolean velocityOnTarget;
	
	public BantorPID(double kV, double kA, double PositionkP, double PositionkI, double PositionkD,
			double VelocitykP, double VelocitykI, double VelocitykD,
			double dt, double positionTolerance, double velocityTolerance) {
		this.PositionkP = PositionkP;
		this.PositionkI = PositionkI;
		this.PositionkD = PositionkD;

		this.dt = dt;
		
		//need to make sure that dt is already set above before making the new derivatives
		positionDerivative = new TorDerivative(dt);
		velocityDerivative = new TorDerivative(dt);
		
		//sets the tolerances
		this.positionTolerance = positionTolerance;
		this.velocityTolerance = velocityTolerance;
		this.kV = kV;
		this.kA = kA;
	}
	
	public double update() {
		//figures out the errors first
		positionError = activePositionTarget - currentPosition;
		velocityError = activeVelocityTarget - currentVelocity;
		
		//the position PID
		positionDerivativeResult = positionDerivative.estimate(positionError);
		positionIntegralResult += positionError;
		PositionPIDResult = 
				(positionError * PositionkP) +
				(positionDerivativeResult * PositionkD) +//we subtract the active velocity target since, if it is going perfectly the position Derivative
				//result will just be the expected velocity, so we would want to make the derivative term be 0
				(positionIntegralResult * dt * PositionkI);
		
		//the final output
		//adds them all together
		output = PositionPIDResult + (kV * activeVelocityTarget) + (kA * activeAccelerationTarget);
		
		return output;
	}
	
	//this void updates all the targets to PID to.
	//P:Position, V:Velocity
	public void updateTargets(double p, double v, double a) {
		activePositionTarget = p;
		activeVelocityTarget = v;
		activeAccelerationTarget = a;
	}
	//this void updates what is happening currently with the real world values
	public void updateCurrentValues(double p, double v) {
		currentPosition = p;
		currentVelocity = v;
	}
	//resets all the integrals and derivatives
	public void reset() {
		positionDerivative.reset();
		velocityDerivative.reset();
		positionIntegralResult = 0;
	}
	
	public boolean isOnTarget() {
		//determines if the position is on target
		positionOnTarget = (Math.abs(positionError) < positionTolerance);
		
		//determines if the velocity is on target
		velocityOnTarget = (Math.abs(velocityError) < velocityTolerance);
		
		//if the position and velocity are on target so the PID is on target
		return (positionOnTarget && velocityOnTarget);
	}
	
}
