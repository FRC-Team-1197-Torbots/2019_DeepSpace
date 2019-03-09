package frc.robot.Elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
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

// ---------------    Hardware   ------------------------------------------
// Talons
    private CANSparkMax talon1;// this talon is the "drive talon for the elevator"
    private CANSparkMax talon2;// this is the second one
    private VictorSPX ballIntake1; //ball intake shooter
    private VictorSPX ballArm1;
    private VictorSPX ballArm2;
    private TalonSRX climberTalon; // wheels on climber to move forware
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

    private DigitalInput light1;
    private DigitalInput light2;
    private DigitalInput light3;

//end of hardware -------------------------------------------

// Controllers
    private Joystick player1;
    private Joystick player2;
    private Joystick autoBox;

// Booleans
    private final boolean talon2Inverted = false;
    private final boolean intakeMotor2Inverted = false;

    public Elevator(Joystick player1, TorDrive drive) {
    // Talons
        talon1 = new CANSparkMax(1, MotorType.kBrushless);
        talon2 = new CANSparkMax(2, MotorType.kBrushless);
        talon2.follow(talon1);
        ballIntake1 = new VictorSPX(9);
        ballArm1 = new VictorSPX(7);
        ballArm2 = new VictorSPX(8);
        climberTalon = new TalonSRX(13);

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
        
        // light1 = new DigitalInput();
        // light2 = new DigitalInput();
        // light3 = new DigitalInput();


    // Joysticks 
        this.player1 = player1;
        player2 = new Joystick(1);
        autoBox = new Joystick(2);

    // Classes
        ballArm = new ballArm(ballArm1, ballArm2, ballIntake1, fourtwenty);
        hatchElevator = new hatchElevator(talon1, talon2, encoder, player1, player2, talon2Inverted, hatchPiston, ballArm);
        ballElevator = new ballElevator(talon1, talon2, encoder, player2, talon2Inverted, ballBreakBeam, ballArm);
        groundIntake = new groundIntake(talon1, talon2, ballArm, player2, encoder);
        manualOverride = new manualOverride(talon1, talon2, player2, talon2Inverted, ballArm1, ballArm2,
             elevatorShifter, climberPiston1, climberPiston2, climberTalon, hatchPiston);
        climb = new Climb(talon1, talon2, climberTalon, encoder, climbGyro, climbSwitch1, climberPiston1, climberPiston2, drive, ballArm);
    }

    public boolean climbing() {
        return autoBox.getRawButton(1);//we need to see
    }

    public void init() {
        elevatorStateMachine = elevatorState.ZEROING;
    }

    public void testArm() {
        if(player2.getRawButton(1)) {
            ballArm.setMode(0);
            ballArm.update(10);
        } else if(player2.getRawButton(3)) {
            ballArm.setMode(0);
            ballArm.update(60);
        } else {
            ballArm.setMode(0);
            ballArm.update(0);
        }
    }

    public void update() {
        if(elevatorStateMachine != elevatorState.CLIMBING) {
            elevatorShifter.set(true);//high gear
        } else {
            elevatorShifter.set(false);
        }



        //we haven't made an autobox yet
        switch(elevatorStateMachine) {
            case ZEROING:
                SmartDashboard.putString("state", "zeroing");
                talon1.set(moveUpZeroSpeed);
                talon2.set(moveUpZeroSpeed);
                if(player2.getRawButton(8)) {
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
                            ballElevator.update(true, !limitSwitch.get());
                            hatchElevator.update(false, !limitSwitch.get());
                            SmartDashboard.putString("elevator state", "ball");
                        } else {// hatch
                            ballElevator.update(false, !limitSwitch.get());
                            hatchElevator.update(true, !limitSwitch.get());
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