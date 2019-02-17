/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Elevator;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Ultrasonic;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.PID_Tools.*;

/*
    ***************************** 
    To Do:
    - Make PID for climb and going to position
        - make PIDRun method
    - Make access to drivetrain and put code in setDriveSpeed method
    - How to start climb



    ********************************
    */

public class Climb {

    // Talons
    private TalonSRX talon1;
    private TalonSRX talon2;
    private TalonSRX climberTalon;

    // solenoids
    private Solenoid upPiston;
    private Solenoid climbPiston1;
    private Solenoid climbPiston2;

    // Sensors
    private Encoder encoder;
    private AnalogGyro climbGyro;
    // private Ultrasonic climbUltrasonic1;
    // private Ultrasonic climbUltrasonic2;
    private DigitalInput climbBreakBeam1;
    // private DigitalInput climbBreakBeam2;

    // PID
    private long currentTime;
    private long startTime = (long) (Timer.getFPGATimestamp() * 1000);
    private double timeInterval = 0.005;
    private double dt = timeInterval;
    private long lastCountedTime;
    private boolean starting = true;
    private BantorPID tiltPID;
    private double controlPower;// this is the amount of power the PID is giving out
    private TorDerivative findCurrentVelocity;
    private double currentVelocity;
    private double currentTarget;

    /*
     * tuneable
     * variables--------------------------------------------------------------------
     * -----------------------------------------------------------------------------
     * -----------------------
     */
    // our variables
    // ----------------- PID -----------------------------------------
    private final double tiltkP = 0.0;
    private final double tiltkI = 0.0;
    private final double tiltkD = 0.0;
    private final double tiltTolerance = 0.01;// for thePID
    private final double velocitykP = 0.0;// velocity stuff probably not needed at all and should keep 0
    private final double velocitykI = 0.0;
    private final double velocitykD = 0.0;
    private final double kV = 0.0;
    private final double kA = 0.0;// this should definitely stay at 0
    private final double pidGoTolerance = 100;// this is in meters and should be kind of large as we are using bang bang
                                              // till PID turns on
    private final double velocityTolerance = 0.0;
    private final double targetVelocity = 0.0;// probably won't need
    private final double targetAcceleration = 0.0;// probably won't need

    private final double encoderTicksPerMeter = 1.0;// this is how many ticks there are per meter the elevator goes up

    // ----------------- Elevator -----------------------------------------

    private final double startClimbPosition = 100.0; // this is where the elevator will go to first.
    private final double elevatorBottomPosition = 0;

    private final double absoluteMaxUpwardVelocity = 1.0;// don't make it higher than 1.0 POSITIVE
    private final double absoluteMaxDownwardVelocity = 1.0;// don't make it higher than 1.0 POSITIVE

    // ----------------- Piston -----------------------------------------

    private final double pneumaticWaitTime = 1;// you need to have a small pause between the pneumatic and elevator when
                                               // you intake
    // this is since if you retract really fast before you give the elvator time to
    // lift up, it won't grab the hatch

    // ----------------- Drive -----------------------------------------

    private final double climberTalonSpeed = 0.5;
    private final double drivetrainSpeed = 0.5;

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

    private theClimb climb = theClimb.IDLE;

    public Climb(TalonSRX talon1, TalonSRX talon2, TalonSRX climberTalon, Encoder encoder, AnalogGyro climbGyro,
            DigitalInput climbBreakBeam1, Solenoid upPiston, Solenoid climbPiston1, Solenoid climbPiston2) {
        // talons
        this.talon1 = talon1;
        this.talon2 = talon2;
        this.climberTalon = climberTalon;

        // sensors
        this.encoder = encoder;
        this.climbGyro = climbGyro;
        this.climbBreakBeam1 = climbBreakBeam1;

        // Solenoids
        this.upPiston = upPiston;
        this.climbPiston1 = climbPiston1;
        this.climbPiston2 = climbPiston2;

        tiltPID = new BantorPID(kV, kA, tiltkP, tiltkI, tiltkD, velocitykP, velocitykI, velocitykD, dt, tiltTolerance,
                velocityTolerance);
        tiltPID.reset();

        findCurrentVelocity = new TorDerivative(dt);
        findCurrentVelocity.resetValue(0);

        // reset gyro so that it is 0 when it is straight
        climbGyro.reset();
    }

    public void update() {
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

            if (climb == theClimb.setUp) {
                currentTarget = startClimbPosition;
            } else if (climb == theClimb.lift) {
                currentTarget = elevatorBottomPosition;

            }
        }
        // PIDRun();//this only updates what value the elevator should be going
        stateRun();

        // if(running ) {
        // talon1.set(ControlMode.PercentOutput, );
        // } else {
        // talon1.set(ControlMode.PercentOutput, 0);
        // }

    }

    // PID Run

    public void stateRun() {
        switch (climb) {
        case IDLE:
            break;
        case setUp:
            upPiston.set(true);
            // go to start climb position
            climb = theClimb.lift;
            break;
        case lift:
            // keep the gyro at 0 while lifting up
            climbPiston1.set(true);
            climbPiston2.set(true);

            // if the position of the elevator is at the bottom, go to drive forward
            if (encoder.get() < elevatorBottomPosition) {
                climb = theClimb.driveForward;
            }

            break;
        case driveForward:
            // run the drive motors and the climber motors
            climberTalon.set(ControlMode.PercentOutput, climberTalonSpeed);
            setDriveSpeed(drivetrainSpeed);
            if (climbBreakBeam1.get()) { // if the breakbeam is activated and the CG is on the platform,
                // set motor speeds to 0
                climberTalon.set(ControlMode.PercentOutput, 0);
                setDriveSpeed(0);
                // start retracting the pistons
                climb = theClimb.retractPiston;
            }
            break;
        case retractPiston:
            climbPiston1.set(false);
            climbPiston2.set(false);

            // delay time for piston to retract

            climb = theClimb.driveRobot;

            break;
        case driveRobot:
            // drive robot forward a little bit to stay on platform
            climb = theClimb.IDLE;
            break;

        }
    }

    
    private void setDriveSpeed(double speed) {
        // code to drive robot forward
    }

    public void startClimb(){
        climb = theClimb.setUp;
    }
}
