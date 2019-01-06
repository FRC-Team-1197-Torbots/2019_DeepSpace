package frc.robot;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;

public class DriveHardware {
	
	private ADXRS450_Gyro gyro;

	private final Talon rightMaster;
	private final Talon rightSlave1;
	private final Talon rightSlave2;
	private final Talon leftMaster;
	private final Talon leftSlave1;
	private final Talon leftSlave2;
	private Encoder leftEncoder;
	private Encoder rightEncoder;
	
	private double leftSpeed;
	private double rightSpeed;
	
	private double currentVoltage;
	
	private final Solenoid solenoid;
	
	/***************************/
	
	private boolean leftOutputReversed = true;
	private boolean rightOutputReversed = true;
	
	private double heading = 0.0;
	
	
	/*
	 * THINGS NEEDED TO BE TUNED
	 * --------------------------------------------
	 */
	//need to be in the same gear
	//also need to tune the values in TorTrajectory, to use linear and pivot trajectories
	//also need to tune global motion limits in TorTrajectoryLib (don't need to tune TorTrajectory in TorTrajectoryLib since it is not building any linear or pivots)
	private final double encoderTicksPerMeter = 924.0; //push the robot forward one meter and take the average of the two encoder distances
	private final double absoluteMaxVelocity = 0.0; //use encoder ticks per meter, and using the robot, set it to the max speed on both wheels and see how many encoder ticks it goes forward
	//then using the encoder ticks per meter calculation, calculate its absolute Max Velocity [Units: Meters/Second]
	private final double absoluteMaxAcceleration = 0.0;//[Units:(delta meters/seconds)/seconds
	//just see how many seconds it takes to go from 0 to 100% speed and divide the absolute max velocity by that number of seconds
	private final double absoluteMaxOmega = 0.0;//use the gyro and set one motor to 100% and the other to -100% [Units: Radians/Second]
	private final double absoluteMaxAlpha = 0.0;//[Units:(delta radians/seconds)/seconds]
	/*
	 * --------------------------------------------
	 */
	
	public DriveHardware() {
		gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
		
		solenoid = new Solenoid(0, 1);

		leftMaster = new Talon(1);
		leftSlave1 = new Talon(2);
		leftSlave2 = new Talon(3);  
		rightMaster = new Talon(4);
		rightSlave1 = new Talon(5);
		rightSlave2 = new Talon(6);

		leftEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		rightEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
		
		leftMaster.setInverted(true); // Left master must be attached to the farthest CIM from the output shaft
		leftSlave1.setInverted(false); 
		leftSlave2.setInverted(false);
		
		rightMaster.setInverted(false); // Right master must be attached to the farthest CIM from the output shaft
		rightSlave1.setInverted(true); 
		rightSlave2.setInverted(true);

		resetEncoder();
		resetGyro();
	}

	public void setMotorSpeeds(double rightSpeed, double leftSpeed) {
		if(leftOutputReversed){
			SetLeft(-leftSpeed);
		}
		else{
			SetLeft(leftSpeed);
		}
		if(rightOutputReversed){
			SetRight(-rightSpeed);
		}
		else{
			SetRight(rightSpeed);
		}
	}

	// Setting the left master Talon's speed to the given parameter
	public void SetLeft(double speed) {
		leftMaster.set(speed);
		leftSlave1.set(speed);
		leftSlave2.set(speed);
	}

	// Setting the right master Talon's speed to the given parameter
	public void SetRight(double speed) {
		rightMaster.set(speed);
		rightSlave1.set(speed);
		rightSlave2.set(speed);
	}

	// Getting raw position value from the right encoder
	public double getRightEncoder() {
		return rightEncoder.getRaw();
	}

	// Getting raw position value from the left encoder
	public double getLeftEncoder() {
		return leftEncoder.getRaw();
	}

	// Getting the average encoder position from both encoders
	public double getAverageEncoderPosition() {
		return (rightEncoder.getRaw() + leftEncoder.getRaw()) * 0.5;
	}

	// Getting the position from both encoders in meters
	public double getPosition() {
		return ((rightEncoder.getRaw() + leftEncoder.getRaw()) * 0.5) / encoderTicksPerMeter; // [meters]
	}

	// Getting the angle in radians from the spartan board
	public double getHeading() {
		heading = (gyro.getAngle() * (Math.PI / 180));
		return heading; // [radians]
	}

	// Method to set the the linear and angular speed of the robot
	public void setTargets(double percentV, double percentOmega) {
		leftSpeed = percentV - percentOmega;
		rightSpeed = percentV + percentOmega;
		
		rightMaster.set(rightSpeed);
		rightSlave1.set(rightSpeed);
		rightSlave2.set(rightSpeed);
		leftMaster.set(leftSpeed);
		leftSlave1.set(leftSpeed);
		leftSlave2.set(leftSpeed);
	}

	// Method to reset the encoder values
	public void resetEncoder() {
		leftEncoder.reset();
		rightEncoder.reset();
	}

	// Method to reset the spartan board gyro values
	public void resetGyro() {
		gyro.reset(); 
	}
	
	// Method to shift the drive to low gear
	public void shiftToLowGear() {
		solenoid.set(true);
	}
	
	// Method to shift the drive to high gear
	public void shiftToHighGear() {
		solenoid.set(false);
	}
	
	// Method to initialize 
	public void init(){
		if(gyro == null){ 
			gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
		}
		gyro.calibrate();
	}
	
	public double getAbsoluteMaxVelocity() {
		return absoluteMaxVelocity;
	}
	
	public double getAbsoluteMaxAcceleration() {
		return absoluteMaxAcceleration;
	}
	
	public double getAbsoluteMaxOmega() {
		return absoluteMaxOmega;
	}
	
	public double getAbsoluteMaxAlpha() {
		return absoluteMaxAlpha;
	}
	
	public double getCurrentVoltage() {
		currentVoltage = RobotController.getInputVoltage();
		return currentVoltage;
	}
}