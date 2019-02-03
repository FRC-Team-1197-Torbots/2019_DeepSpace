package frc.robot.Elevator;
import frc.robot.PID_Tools.*;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class groundIntake {
    //it will need both talons to go up. It will need the ground intake talons. It will need the encoder to make sure the elevator
    //is high enough before the ground intake deploys
    //it will need a potentiometer for the ground intake
    //it will need both joysticks
    //it will need a solenoid to outake

    private TalonSRX elevatorTalon1;//talons for the up and down
    private TalonSRX elevatorTalon2;
    private TalonSRX groundTalon1;
    private TalonSRX groundTalon2;
    private Joystick player1;
    private Joystick player2;
    private Encoder encoder;
    private AnalogPotentiometer fourtwenty;//ITS THE POT
    private long currentTime = (long)(Timer.getFPGATimestamp() * 1000);
    private long lastTime = currentTime;
    private long lastTimeRightTriggerPressed = currentTime;
    private boolean highEnough = false;
    private double currentHeight;
    private TorDerivative findCurrentVelocity;
    private TorDerivative groundfindCurrentVelocity;
    private BantorPID positionPID;
    private BantorPID groundPositionPID;
    private Solenoid groundShootPiston;
    private double currentVelocity;
    private double groundcurrentVelocity;
    private double controlPower;//this is the amount of power the PID is giving out
    
    private double groundCurrentTarget;

    private double groundCurrentAngle;
    private double groundstartAngle;
    private double groundControlPower;

    /*
    tuneable stuff--------------------------------------->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    */
    private final double neededHeight = 1;//in meters
    private final double currentTarget = 1.2;//this is the height the elevator will PID to
    private final double groundIntakeDownAngle = 90;
    private final double groundIntakeInAngle = -10;
    private final double groundDiagonalAngle = 45;

    private final double positionkP = 0.0;
    private final double positionkI = 0.0;
    private final double positionkD = 0.0;
    private final double positionTolerance = 0.01;//for thePID
    private final double velocitykP = 0.0;//velocity stuff probably not needed at all and should keep 0
    private final double velocitykI = 0.0;
    private final double velocitykD = 0.0;
    private final double kV = 0.0;
    private final double kA = 0.0;//this should definitely stay at 0
    private final double velocityTolerance = 0.0;
    private final double targetVelocity = 0.0;//probably won't need
    private final double targetAcceleration = 0.0;//probably won't need
    private final double dt = 0.005;
    private final double encoderTicksPerMeter = 1.0;//this is how many ticks there are per meter the elevator goes up
    private final double absoluteMaxUpwardVelocity = 1.0;//don't make it higher than 1.0 POSITIVE
    private final double absoluteMaxDownwardVelocity = 1.0;//don't make it higher than 1.0 POSITIVE
    
    private final long shootOutTime = 500;//time it takes to fire the pistons to shoot the hatch out

    private final double groundpositionkP = 0.0;
    private final double groundpositionkI = 0.0;
    private final double groundpositionkD = 0.0;
    private final double groundpositionTolerance = 2;//for thePID
    private final double groundvelocitykP = 0.0;//velocity stuff probably not needed at all and should keep 0
    private final double groundvelocitykI = 0.0;
    private final double groundvelocitykD = 0.0;
    private final double groundkV = 0.0;
    private final double groundkA = 0.0;//this should definitely stay at 0
    private final double groundvelocityTolerance = 0.0;
    private final double groundtargetVelocity = 0.0;//probably won't need
    private final double groundtargetAcceleration = 0.0;//probably won't need
    /*
    ----------------------------------------------------->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    */
    private int initialTicks;

    //the state machine
    public static enum ground {
        IDLE, GOINGUP, DOWN, PULLEDBACK, SHOOTPOS, SHOOTING;
        private ground () {}
    }
    private ground groundIntake = ground.IDLE;

    public groundIntake(TalonSRX elevatorTalon1, TalonSRX elevatorTalon2, TalonSRX groundTalon1, TalonSRX groundTalon2,
        Joystick player1, Joystick player2, AnalogPotentiometer fourtwenty, Encoder encoder, Solenoid groundShootPiston) {
        this.elevatorTalon1 = elevatorTalon1;
        this.elevatorTalon2 = elevatorTalon2;
        this.groundTalon1 = groundTalon1;
        this.groundTalon2 = groundTalon2;
        this.player1 = player1;
        this.player2 = player2;
        this.fourtwenty = fourtwenty;
        this.encoder = encoder;
        this.groundShootPiston = groundShootPiston;

        //this is the PID
        positionPID = new BantorPID(kV, kA, positionkP, positionkI, positionkD, velocitykP,
            velocitykI, velocitykD, dt, positionTolerance, velocityTolerance);
        groundPositionPID = new BantorPID(groundkV, groundkA, groundpositionkP, groundpositionkI, groundpositionkD, groundvelocitykP,
            groundvelocitykI, groundvelocitykD, dt, groundpositionTolerance, groundvelocityTolerance);
        groundPositionPID.reset();
        groundfindCurrentVelocity = new TorDerivative(dt);
        groundfindCurrentVelocity.resetValue(0);
        positionPID.reset();
        findCurrentVelocity = new TorDerivative(dt);
        findCurrentVelocity.resetValue(0);
    }

    public void update(boolean elevatorManualOverriding, boolean running) {
        currentTime = (long)(Timer.getFPGATimestamp() * 1000);
        highEnough = (currentHeight >= neededHeight);

        if((Math.abs(player1.getRawAxis(3)) > 0.3) && ((currentTime - lastTimeRightTriggerPressed) > 250)
        && (groundIntake == ground.IDLE)) {//right trigger is pressed
            lastTimeRightTriggerPressed = currentTime;
            if(groundIntake == ground.IDLE) {
                if(elevatorManualOverriding || highEnough) {
                    groundIntake = ground.DOWN;
                } else {
                    groundIntake = ground.GOINGUP;
                }
            }
        }

        stateMachineRun();
        groundPIDRun();
        
        if(running) {
            elevatorTalon1.set(ControlMode.PercentOutput, controlPower);
            elevatorTalon2.set(ControlMode.PercentOutput, controlPower);
            groundTalon1.set(ControlMode.PercentOutput, groundControlPower);
            groundTalon2.set(ControlMode.PercentOutput, groundControlPower);
        }
    }

    public void stateMachineRun() {
        switch(groundIntake) {
            case IDLE:
                groundCurrentTarget = groundIntakeInAngle;
                break;
            case DOWN:
                groundCurrentTarget = groundIntakeDownAngle;
                if(!(Math.abs(player1.getRawAxis(3)) > 0.3)) {
                    groundIntake = ground.SHOOTPOS;
                }
                break;
            case GOINGUP:
                groundCurrentTarget = groundIntakeInAngle;
                if(highEnough) {
                    groundIntake = ground.DOWN;
                }
                break;
            case PULLEDBACK:
                groundCurrentTarget = groundIntakeInAngle;
                if(!(Math.abs(player2.getRawAxis(2)) > 0.3)) {
                    groundIntake = ground.PULLEDBACK;
                }
                break;
            case SHOOTPOS:
                groundCurrentTarget = groundDiagonalAngle;
                if(player2.getRawButton(3)) {//button X
                    lastTime = currentTime;
                    groundShootPiston.set(true);
                    groundIntake = ground.SHOOTING;
                } else if((Math.abs(player2.getRawAxis(2)) > 0.3)) {//player 2 presses left trigger
                    groundIntake = ground.PULLEDBACK;
                }
                break;
            case SHOOTING:
                groundCurrentTarget = groundDiagonalAngle;
                if(currentTime - lastTime > shootOutTime) {
                    groundShootPiston.set(false);
                    groundIntake = ground.IDLE;
                }
                break;
        }
    }

    public void groundPIDRun() {
        groundCurrentAngle = fourtwenty.get() - groundstartAngle;
        groundcurrentVelocity = groundfindCurrentVelocity.estimate(groundCurrentAngle);
        groundPositionPID.updateTargets(groundCurrentTarget, groundtargetVelocity, groundtargetAcceleration);
        groundPositionPID.updateCurrentValues(groundCurrentAngle, groundcurrentVelocity);
        groundControlPower = groundPositionPID.update();
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

    public double height() {
        return ((encoder.get() - initialTicks)/encoderTicksPerMeter);
    }

    public void init() {
        initialTicks = encoder.get();
        groundstartAngle = fourtwenty.get();
    }
}