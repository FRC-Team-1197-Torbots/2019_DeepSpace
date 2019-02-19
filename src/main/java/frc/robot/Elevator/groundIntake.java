package frc.robot.Elevator;
import frc.robot.PID_Tools.*;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;

public class groundIntake {
   //it will need both talons to go up. It will need the ground intake talons. It will need the encoder to make sure the elevator
   //is high enough before the ground intake deploys
   //it will need a potentiometer for the ground intake
   //it will need both joysticks
   //it will need a solenoid to outake

   private TalonSRX elevatorTalon1;//talons for the up and down
   private TalonSRX elevatorTalon2;
   private VictorSPX groundTalon1;
   private Joystick player1;
   private Joystick player2;
   private Encoder encoder;
   private AnalogPotentiometer fourtwenty;//ITS THE POT
   private long currentTime = (long)(Timer.getFPGATimestamp() * 1000);
   private long lastTime = currentTime;
   private boolean highEnough = false;
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
   private double groundControlPower;

   /*
   tuneable stuff--------------------------------------->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
   */

   //this top PID is for the elevator


   private final double neededHeight = -0.02;//in meters
   private final double currentTarget = neededHeight + 0.02;//this is the height the elevator will PID to

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
   private final double encoderTicksPerMeter = 885;//this is how many ticks there are per meter the elevator goes up
   private final double absoluteMaxUpwardVelocity = 1.0;//don't make it higher than 1.0 POSITIVE
   private final double absoluteMaxDownwardVelocity = 1.0;//don't make it higher than 1.0 POSITIVE
  
   private final long shootOutTime = 500;//time it takes to fire the pistons to shoot the hatch out

   private final double groundpositionkP = 0.009;//0.009
   private final double groundpositionkI = 0.0;
   private final double groundpositionkD = 0.0002;
   private final double groundpositionTolerance = 0.0;//for thePID
   private final double groundvelocitykP = 0.0;//velocity stuff probably not needed at all and should keep 0
   private final double groundvelocitykI = 0.0;
   private final double groundvelocitykD = 0.0;
   private final double groundkV = 0.0;
   private final double groundkA = 0.0;//this should definitely stay at 0
   private final double groundvelocityTolerance = 0.0;
   private final double groundtargetVelocity = 0.0;//probably won't need
   private final double groundtargetAcceleration = 0.0;//probably won't need

   private final int potSwitched = -1;
   private final double groundIntakeDownAngle = 80;
   private final double groundIntakeInAngle = 25;
   private final double groundDiagonalAngle = 40;
   private final double groundstartAngle = 50.077898828536675;
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

   public groundIntake(VictorSPX groundTalon1,
       Joystick player1, Joystick player2, AnalogPotentiometer fourtwenty, Solenoid groundShootPiston,
       Encoder ElevatorEncoder) {
       this.groundTalon1 = groundTalon1;
       this.player1 = player1;
       this.player2 = player2;
       this.fourtwenty = fourtwenty;
       this.groundShootPiston = groundShootPiston;
       this.encoder = ElevatorEncoder;

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
       highEnough = (height() >= neededHeight);

       if((Math.abs(player1.getRawAxis(3)) > 0.3)
       && (groundIntake == ground.IDLE)) {//right trigger is pressed
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
           SmartDashboard.putBoolean("key", true);
           elevatorTalon1.set(ControlMode.PercentOutput, controlPower);
           elevatorTalon2.set(ControlMode.PercentOutput, controlPower);

           SmartDashboard.putNumber("groundControlPower:", groundControlPower);
           groundTalon1.set(ControlMode.PercentOutput, groundControlPower);
       }

       SmartDashboard.putBoolean("IDLE:", groundIntake == ground.IDLE);
       SmartDashboard.putBoolean("DOWN:", groundIntake == ground.DOWN);
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
                   groundIntake = ground.SHOOTPOS;
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
       groundCurrentAngle = (fourtwenty.get() - groundstartAngle) * potSwitched;
       groundcurrentVelocity = groundfindCurrentVelocity.estimate(groundCurrentAngle);
       groundPositionPID.updateTargets(groundCurrentTarget, groundtargetVelocity, groundtargetAcceleration);
       groundPositionPID.updateCurrentValues(groundCurrentAngle, groundcurrentVelocity);
       SmartDashboard.putNumber("groundCurrentAngle:", groundCurrentAngle);
       SmartDashboard.putNumber("groundCurrentTarget", groundCurrentTarget);
       groundControlPower = groundPositionPID.update();
       SmartDashboard.putNumber("groundcontrolpower from the PID directly:", groundControlPower);
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

   public void init(int initialValue) {
       initialTicks = encoder.get() - initialValue;
   }
}

