package frc.robot.Elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.TorDrive;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator {

    //this is the state machine for the whole elevator
    public static enum elevatorState {
        ZEROING, RUNNING, CLIMBING;
        private elevatorState() {}
    }

    public elevatorState elevatorStateMachine = elevatorState.ZEROING;

    //zeroing state tuning-----------

    private final double moveUpZeroSpeed = -0.5;
    //-----------

// ---------------    Classes   ------------------------------------------
// this elevator has the hatch elevator and the ball elevator in it
    private hatchElevator hatchElevator;
    private ballElevator ballElevator;
    private groundIntake groundIntake;
    private manualOverride manualOverride;
    private ballArm ballArm;
    private Climb climb;
    private statusLights statusLights;

// ---------------    Hardware   ------------------------------------------
// Talons
    private CANSparkMax talon1;// this talon is the "drive talon for the elevator"
    private CANSparkMax talon2;// this is the second one
    private VictorSPX ballIntake1; //ball intake shooter
    private VictorSPX ballArm1;
    private VictorSPX ballArm2;
    private VictorSPX climberTalon; // wheels on climber to move forware
// Solenoids
    private Solenoid elevatorShifter; // elevator shifter
    private Solenoid hatchPiston; // hatch piston for hatch mech
    private Solenoid climberPiston1;
    private Solenoid climberPiston2;

// Sensors
    private Encoder encoder; //elevator encoder
  
    private DigitalInput limitSwitch; // limit switch to stop the elevator
    
    private DigitalInput ballBreakBeam;

    private AnalogPotentiometer fourtwenty; // pot on the arm

    private DigitalInput climbSwitch1;

    private AnalogGyro climbGyro;

    private DigitalOutput light1;
    private DigitalOutput light2;
    private DigitalOutput light3;

//end of hardware -------------------------------------------

    private long currentTime = (long)(1000.0 * Timer.getFPGATimestamp());
    private long lastTime = currentTime;

// Controllers
    private Joystick player1;
    private Joystick player2;
    private Joystick autoBox;

// Booleans
    private final boolean talon2Inverted = false;
    private final boolean intakeMotor2Inverted = false;
    private boolean starting = true;
    private boolean hatchMode = true;

    public Elevator(Joystick player1, TorDrive drive) {
    // Talons
        talon1 = new CANSparkMax(1, MotorType.kBrushless);
        talon2 = new CANSparkMax(2, MotorType.kBrushless);
        // talon1 = new TalonSRX(1);
        // talon2 = new TalonSRX(2);
        talon2.follow(talon1);
        ballIntake1 = new VictorSPX(9);
        ballArm1 = new VictorSPX(7);
        ballArm2 = new VictorSPX(8);
        climberTalon = new VictorSPX(13);

    // Solenoid
        elevatorShifter = new Solenoid(1);
        hatchPiston = new Solenoid(2);
        climberPiston1 = new Solenoid(3);
        climberPiston2 = new Solenoid(4);
        
    // Sensors 
        fourtwenty = new AnalogPotentiometer(1, 360, 0);
        climbGyro = new AnalogGyro(0);
        encoder = new Encoder(4, 5);
        limitSwitch = new DigitalInput(8);
        ballBreakBeam = new DigitalInput(9);//for the break  beam
        climbSwitch1 = new DigitalInput(23);
        
        light1 = new DigitalOutput(22);
        light2 = new DigitalOutput(24);
        light3 = new DigitalOutput(25);


    // Joysticks 
        this.player1 = player1;
        player2 = new Joystick(1);
        autoBox = new Joystick(2);

    // Classes
        ballArm = new ballArm(ballArm1, ballArm2, ballIntake1, fourtwenty);
        statusLights = new statusLights(light1, light2, light3);
        hatchElevator = new hatchElevator(talon1, talon2, encoder, player1, player2, talon2Inverted, hatchPiston, ballArm, statusLights);
        ballElevator = new ballElevator(talon1, talon2, encoder, player2, talon2Inverted, ballBreakBeam, ballArm, statusLights);
        groundIntake = new groundIntake(talon1, talon2, ballArm, player2, encoder, statusLights);
        manualOverride = new manualOverride(talon1, talon2, player2, talon2Inverted, ballArm1, ballArm2,
             elevatorShifter, climberPiston1, climberPiston2, climberTalon, hatchPiston);
        climb = new Climb(talon1, talon2, climberTalon, encoder, climbGyro, climbSwitch1, climberPiston1, climberPiston2, drive, ballArm, statusLights);
    }

    public boolean climbing() {
        return autoBox.getRawButton(1);//we need to see
    }

    public void init() {
        elevatorStateMachine = elevatorState.ZEROING;
        initValues();
        currentTime = (long)(1000.0 * Timer.getFPGATimestamp());
        lastTime = currentTime;
    }

    public void testArm() {
        if(player2.getRawButton(1)) {
            ballArm.setMode(0);
            ballArm.update(-30);
        } else if(player2.getRawButton(3)) {
            ballArm.setMode(0);
            ballArm.update(60);
        } else {
            ballArm.setMode(0);
            ballArm.update(0);
        }
    }

    public void update() {
        currentTime = (long)(1000 * Timer.getFPGATimestamp());
        if(elevatorStateMachine != elevatorState.CLIMBING) {
            elevatorShifter.set(true);//high gear
        } else {
            elevatorShifter.set(false);
        }


        //we haven't made an autobox yet
        switch(elevatorStateMachine) {
            case ZEROING:
                if(starting) {
                    lastTime = currentTime;
                    starting = false;
                }
                SmartDashboard.putString("state", "zeroing");
                ballArm.update(65);
                if(player2.getRawButton(8) || (currentTime - lastTime > 1000)) {
                    elevatorStateMachine = elevatorState.RUNNING;
                }
                break;
            case RUNNING:
                SmartDashboard.putString("state", "running");
                if(autoBox.getRawButton(1)) {//climbing button
                    elevatorStateMachine = elevatorState.CLIMBING;
                } else {
                    if (Math.abs(getRightTrigger()) > 0.1) {
                        manualOverride.update(true);
                    } else {
                        manualOverride.update(false);
                        if (getRightBumper()) {// ball
                            hatchMode = false;
                            ballElevator.update(true, limitSwitch.get());
                            hatchElevator.update(false, limitSwitch.get());
                            groundIntake.update(false, limitSwitch.get());
                            SmartDashboard.putString("elevator state", "ball");
                        } else if (Math.abs(player2.getRawAxis(2)) > 0.1) { //ground hatch
                            hatchMode = false;
                            groundIntake.update(true, limitSwitch.get());
                            ballElevator.update(false, limitSwitch.get());
                            hatchElevator.update(false, limitSwitch.get());
                        
                        } else {// hatch
                            hatchMode = true;
                            ballElevator.update(false, limitSwitch.get());
                            hatchElevator.update(true, limitSwitch.get());
                            groundIntake.update(false, limitSwitch.get());
                            SmartDashboard.putString("elevator state", "hatch");
                        }
                    }
                }
                break;
            case CLIMBING:
                SmartDashboard.putString("state", "climbing");
                //UPDATE CLIMB 
                climb.update(true);
                break;
        }
    }

    public boolean hatchMode() {
        return hatchMode;
    }

    public void initValues() {
        ballElevator.init(0);
        hatchElevator.init(0);
        climb.init(0);
        groundIntake.init(0);
    }

    /*
     * The following are a bunch of accessor methods to obtain input from the
     * controller.
     */
    public double getLeftX() {
        return player2.getRawAxis(0);
    }

    public double getLeftY() {
        return player2.getRawAxis(1);
    }

    public double getRightX() {
        return player2.getRawAxis(4);
    }

    public double getRightTrigger() {
        return player2.getRawAxis(3);
    }

    public boolean getShiftButton() {
        return player2.getRawButton(5);
    }

    public boolean getRightBumper() {
        return player2.getRawButton(6);
    }

    public boolean getButtonA() {
        return player2.getRawButton(1);
    }

    public boolean getButtonB() {
        return player2.getRawButton(2);
    }

    public boolean getButtonX() {
        return player2.getRawButton(3);
    }

    public boolean getButtonY() {
        return player2.getRawButton(4);
    }
}