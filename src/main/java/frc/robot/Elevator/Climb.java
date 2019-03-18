package frc.robot.Elevator;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogGyro;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;

import frc.robot.PID_Tools.*;
import frc.robot.TorDrive;

public class Climb {

    //this is the drive hardware
    private TorDrive drive;

    private int initialTicks;

    // Talons
    // private TalonSRX talon1;
    // private TalonSRX talon2;
    private CANSparkMax talon1;
    private CANSparkMax talon2;
    private VictorSPX climberTalon;

    // solenoids
    private Solenoid climbPiston1;
    private Solenoid climbPiston2;

    // Sensors
    private Encoder encoder;
    private AnalogGyro climbGyro;
    private DigitalInput climbBreakBeam1;

    // PID
    private long currentTime;
    private long startTime = (long) (Timer.getFPGATimestamp() * 1000);
    private long lastTime;
    private double timeInterval = 0.005;
    private double dt = timeInterval;
    private long lastCountedTime;
    private boolean starting = true;
    private double controlPower;// this is the amount of power the PID is giving out
    private double currentNormalError;
    private double currentTarget;
    private double currentAngleFromGyro;
    private double tiltError;

    private double tiltIntegral;
    private double normalIntegral;

    private double tiltD;
    private double normalD;

    private double tiltP;
    private double normalP;
    
    private TorDerivative tiltDerivative;
    private TorDerivative normalDerivative;

    /*
     * tuneable
     * variables--------------------------------------------------------------------
     * -----------------------------------------------------------------------------
     * -----------------------
     */
    // our variables
    // ----------------- PID -----------------------------------------
    private double flatGyroValue;

    private final double tiltkP = -0.5;
    private final double tiltkI = 0.0;
    private final double tiltkD = 0.0;
    private final double tiltTolerance = 1;// for thePID
    private final double normalTiltPower = -0.8;

    private final double normalkP = -2;
    private final double normalkI = 0.001;
    private final double normalkD = 0.0;
    private final double normalTolerance = 0.01;// for thePID

    private final double encoderTicksPerMeter = 897;// this is how many ticks there are per meter the elevator goes up

    // ----------------- Elevator -----------------------------------------

    private final double startClimbPosition = 0.625;// this is where the elevator will go to first.
    private final double elevatorBottomPosition = 0.01;//these are in meters from the first hall effect sensor

    // ----------------- Drive -----------------------------------------
    private final double climberTalonLiftSpeed = 0.35;
    private final double climberTalonDriveSpeed = 0.75;
    private final double drivetrainSpeed = -0.25;
    private final long pistonRetractTime = 1000;
    private final long driveOnPlatformTime = 5;

    //ball arm
    private final double upAngle = 60;
    private ballArm ballArm;
    private statusLights statusLights;

    /*
     * -----------------------------------------------------------------------------
     * -----------------------------------------------------------------------------
     * -------------------------------
     */

    public static enum theClimb {
        IDLE, // nothing
        setUp, // ball intake up and go to set position where the climber wheel is on the hab
               // platform
        lift, // drives elevator down at the same speed as piston, all the way to the stop
        driveForward, // drives climber wheels and drivetrain wheels at the same time until the
                      // breakbeam activates
        retractPiston, // when the breakbeam activates, retract the pistons
        driveRobot; // drive robot forward a little bit

        private theClimb() {
        }
    }

    private theClimb climb = theClimb.setUp;

    public Climb(CANSparkMax talon1, CANSparkMax talon2, VictorSPX climberTalon, Encoder encoder, AnalogGyro climbGyro,
            DigitalInput climbBreakBeam1, Solenoid climbPiston1, Solenoid climbPiston2, TorDrive drive, ballArm ballArm, statusLights statusLights) {
                // talons
        this.talon1 = talon1;
        this.talon2 = talon2;
        this.climberTalon = climberTalon;

        // sensors
        this.encoder = encoder;
        this.climbGyro = climbGyro;
        this.climbBreakBeam1 = climbBreakBeam1;

        // Solenoids
        this.climbPiston1 = climbPiston1;
        this.climbPiston2 = climbPiston2;

        // reset gyro so that it is 0 when it is straight
        climbGyro.reset();

        normalDerivative = new TorDerivative(dt);
        normalDerivative.reset();
        tiltDerivative = new TorDerivative(dt);
        tiltDerivative.reset();

        // classes
        this.drive = drive;
        this.ballArm = ballArm;
        this.statusLights = statusLights;

    }

    public void update(boolean running) {
        SmartDashboard.putNumber("height in climb:", height());
        SmartDashboard.putString("Climb state", climb.toString());
        currentTime = (long) (Timer.getFPGATimestamp() * 1000);

        // this starting boolean makes it so that it will still do the first value in
        // the trajectory

        // this handles it so that it will only tick in the time interval so that the
        // derivatives and the integrals are correct
        if (((currentTime - startTime) - ((currentTime - startTime) % (dt * 1000))) > // has current time minus start
                                                                                      // time to see the relative time
                                                                                      // the trajectory has been going
        ((lastCountedTime - startTime) - ((lastCountedTime - startTime) % (dt * 1000))) // subtracts that mod dt times
                                                                                        // 1000 so that it is floored to
                                                                                        // the nearest multiple of dt
                                                                                        // times 1000
                // then checks if that is greater than the last one to see if it is time to move
                // on to the next tick
                || starting) {
            starting = false;
            lastCountedTime = currentTime;

            // this sets the current target
                    
            stateRun();
            if (climb == theClimb.setUp || climb == theClimb.IDLE) {
                currentTarget = startClimbPosition - 0.02;
                normalPidRun();
            } else {
                currentTarget = elevatorBottomPosition;
                gyroPidRun();

            }

            SmartDashboard.putString("Climb state", climb.toString());
            if(running) {
                statusLights.displayRainbowLights();
                SmartDashboard.putBoolean("got here 2", true);
                SmartDashboard.putNumber("control Power for climb", controlPower);
                talon1.set(controlPower);
                talon2.set(controlPower);
                ballArm.update(upAngle);
                ballArm.setMode(0);
            }
        }

    }

    // PID Run

    public void stateRun() {
        switch (climb) {
        case IDLE:
            break;
        case setUp:
            flatGyroValue = climbGyro.getAngle();//this is the value of the gyro you read when you are flat
            // go to start climb position
            if(Math.abs(height() - startClimbPosition)  < 0.08) {
                lastTime = currentTime;
                climb = theClimb.lift;
            }
            break;
        case lift:
            // keep the gyro at 0 while lifting up
            climbPiston1.set(true);
            climbPiston2.set(true);

            climberTalon.set(ControlMode.PercentOutput, climberTalonLiftSpeed);
            // if the position of the elevator is at the bottom, go to drive forward
            if (height() <= elevatorBottomPosition || ((currentTime - lastTime) > 4000)) {
                lastTime = currentTime;
                climb = theClimb.driveForward;
            }

            break;
        case driveForward:
            // run the drive motors and the climber motors
            climberTalon.set(ControlMode.PercentOutput, climberTalonDriveSpeed);
            setDriveSpeed(drivetrainSpeed);
            if (!climbBreakBeam1.get() || ((currentTime - lastTime) > 4000)) { // if the breakbeam is activated and the CG is on the platform,
                // set motor speeds to 0
                setDriveSpeed(0);
                // start retracting the pistons
                lastTime = currentTime;
                climb = theClimb.retractPiston;
            }
            break;
        case retractPiston:
            climbPiston1.set(false);
            climbPiston2.set(false);

            // delay time for piston to retract
            if(currentTime - lastTime > pistonRetractTime) {
                lastTime = currentTime;
                setDriveSpeed(drivetrainSpeed);
                climb = theClimb.driveRobot;
            }
            break;
        case driveRobot:
            // drive robot forward a little bit to stay on platform
            if(currentTime - lastTime > driveOnPlatformTime) {
                setDriveSpeed(0);
                climberTalon.set(ControlMode.PercentOutput, 0);
                climb = theClimb.IDLE;
            }
            break;

        }
    }

    public void normalPidRun() {
        currentNormalError = currentTarget - height();
        normalD = normalDerivative.estimate(currentNormalError);
        if(Math.abs(currentNormalError) < normalTolerance) {
            normalIntegral = 0;
        } else {
            normalIntegral += currentNormalError;
        }
        normalP = currentNormalError;
        controlPower = (normalP * normalkP) + (normalD * normalkD) + (normalIntegral * normalkI);
        SmartDashboard.putNumber("control power normal", controlPower);
    }

    public void gyroPidRun() {
        currentAngleFromGyro = climbGyro.getAngle();
        tiltError = flatGyroValue - currentAngleFromGyro;
        tiltD = tiltDerivative.estimate(tiltError);
        if(Math.abs(tiltError) < tiltTolerance) {
            tiltIntegral = 0;
        } else {
            tiltIntegral += tiltError;
        }
        tiltP = tiltError;
        controlPower = (tiltP * tiltkP) + (tiltD * tiltkD) + (tiltIntegral * tiltkI);
        if(climb == theClimb.lift) {
            controlPower += normalTiltPower;
        }
        SmartDashboard.putNumber("control power gyro running", controlPower);
    }

    
    private void setDriveSpeed(double speed) {
        drive.Run(speed);
    }

    public double height() {
        return ((encoder.get() - initialTicks)/encoderTicksPerMeter);
    }

    public void init(int initialValue) {
        initialTicks = encoder.get() - initialValue;
    }
}
