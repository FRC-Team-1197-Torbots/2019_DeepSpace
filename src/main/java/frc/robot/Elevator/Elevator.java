package frc.robot.Elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.TorDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Elevator {

    //this is the state machine for the whole elevator
    public static enum elevatorState {
        ZEROING, RUNNING, CLIMBING;
        private elevatorState() {}
    }

    public elevatorState elevatorStateMachine = elevatorState.ZEROING;

    //zeroing state tuning-----------

    private final double moveUpZeroSpeed = 0.3;
    private final int hallEffectSensorOneHeight = 0;//in ticks from this

    //-----------

// ---------------    Classes   ------------------------------------------
// this elevator has the hatch elevator and the ball elevator in it
    private hatchElevator hatchElevator;
    private ballElevator ballElevator;
    private groundIntake groundIntake;
    private manualOverride manualOverride;
    private getGroundIntakeOutOfWay getGroundIntakeOutOfWay;
    private Climb climb;

// ---------------    Hardware   ------------------------------------------
// Talons
    private TalonSRX talon1;// this talon is the "drive talon for the elevator"
    private TalonSRX talon2;// this is the second one
    private TalonSRX ballIntake1; //ball intake shooter
    private TalonSRX ballIntake2; //ball intake shooter motor 2, needs to be flipped
    private VictorSPX groundTalon1; //ground hatch motor
    // private VictorSPX groundTalon2; 
    private TalonSRX overPull; //ball roller to spin wheels
    private TalonSRX climberTalon; // wheels on climber to move forware

    private VictorSPX overIntake1; //ball roller arm motor
    private VictorSPX overIntake2; //ball roller arm motor
// Solenoids
    private Solenoid elevatorShifter; // elevator shifter
    private Solenoid groundShootPiston;  // ground hatch fire pistons
    private Solenoid hatchPiston; // hatch piston for hatch mech
    private Solenoid ballUpPiston; // piston to angle ball intake 
    private Solenoid climberPiston1;
    private Solenoid climberPiston2;

// Sensors
    private DigitalInput hallEffectSensor1;

    private Encoder encoder; //elevator encoder
    private Encoder ballRollerArmEncoder;
  
    private DigitalInput limitSwitch; // limit switch to stop the elevator
    
    private DigitalInput ballBreakBeam;

    private AnalogPotentiometer fourtwenty; // pot on the 

    private DigitalInput climbSwitch1;

    private AnalogGyro climbGyro;

//end of hardware -------------------------------------------

// Controllers
    private Joystick player2;
    private Joystick autoBox;

// Booleans
    private final boolean talon2Inverted = false;
    private final boolean intakeMotor2Inverted = false;

    public Elevator(Joystick player1, TorDrive drive) {
    // Talons
        talon1 = new TalonSRX(5);
        talon2 = new TalonSRX(6);
        talon2.follow(talon1);
        ballIntake1 = new TalonSRX(7);
        ballIntake2 = new TalonSRX(8);
        overIntake1 = new VictorSPX(9);
        overIntake2 = new VictorSPX(10);
        groundTalon1 = new VictorSPX(11);
        overPull = new TalonSRX(12);
        climberTalon = new TalonSRX(13);

    // Solenoid
        elevatorShifter = new Solenoid(1);
        hatchPiston = new Solenoid(5);
        ballUpPiston = new Solenoid(6);
        groundShootPiston = new Solenoid(4);
        climberPiston1 = new Solenoid(2);
        climberPiston2 = new Solenoid(3);
        
    // Sensors 
        fourtwenty = new AnalogPotentiometer(1, 360, 0);
        climbGyro = new AnalogGyro(0);
        encoder = new Encoder(4, 5);
        ballRollerArmEncoder = new Encoder(6, 7);
        limitSwitch = new DigitalInput(8);
        ballBreakBeam = new DigitalInput(9);//for the break  beam
        climbSwitch1 = new DigitalInput(23);
        hallEffectSensor1 = new DigitalInput(22);

    // Joysticks 
        player2 = new Joystick(1);
        autoBox = new Joystick(2);

    // Classes
        hatchElevator = new hatchElevator(talon1, talon2, encoder, player2, talon2Inverted, hatchPiston);
        ballElevator = new ballElevator(talon1, talon2, encoder, player2, talon2Inverted, ballIntake1, ballIntake2,
                intakeMotor2Inverted, ballUpPiston, overIntake1, overIntake2, overPull, ballRollerArmEncoder, ballBreakBeam);
        groundIntake = new groundIntake(groundTalon1, player1, player2, fourtwenty, groundShootPiston, encoder);
        manualOverride = new manualOverride(talon1, talon2, player2, talon2Inverted, ballIntake1, 
                ballIntake2, intakeMotor2Inverted, ballUpPiston, hatchPiston, overIntake1, overIntake2, overPull, elevatorShifter, climberPiston1, climberPiston2, climberTalon);
        getGroundIntakeOutOfWay = new getGroundIntakeOutOfWay(groundTalon1, groundShootPiston);
        climb = new Climb(talon1, talon2, climberTalon, encoder, climbGyro, climbSwitch1, ballUpPiston, climberPiston1, climberPiston2, drive);

        
    }

    public boolean climbing() {
        return autoBox.getRawButton(1);//we need to see
    }

    public void init() {
        climb.init();
        elevatorStateMachine = elevatorState.ZEROING;
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
                talon1.set(ControlMode.PercentOutput, moveUpZeroSpeed);
                talon2.set(ControlMode.PercentOutput, moveUpZeroSpeed);
                if(!hallEffectSensor1.get() || player2.getRawButton(1)) {
                    ballElevator.init(hallEffectSensorOneHeight);
                    hatchElevator.init(hallEffectSensorOneHeight);
                    groundIntake.init(hallEffectSensorOneHeight);
                    elevatorStateMachine = elevatorState.RUNNING;
                }
                break;
            case RUNNING:
                SmartDashboard.putString("state", "running");
                if(autoBox.getRawButton(1)) {//climbing button
                    climb.init(hallEffectSensorOneHeight);
                    elevatorStateMachine = elevatorState.CLIMBING;
                } else {
                    if (autoBox.getRawButton(3)){
                        getGroundIntakeOutOfWay.update(true);
                    } else {
                        getGroundIntakeOutOfWay.update(false);
                    }
        
                    groundIntake.update(Math.abs(getRightTrigger()) > 0.1, autoBox.getRawButton(3));
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