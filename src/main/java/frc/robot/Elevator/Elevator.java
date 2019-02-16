package frc.robot.Elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Elevator {
    

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
    private TalonSRX groundTalon1; //ground hatch motor
    private TalonSRX groundTalon2; 
    private TalonSRX overIntake1; //ball roller arm motor
    private TalonSRX overIntake2; //ball roller arm motor
    private TalonSRX overPull; //ball roller to spin wheels
    private TalonSRX climberTalon; // wheels on climber to move forware

// Solenoids
    private Solenoid elevatorShifter; // elevator shifter
    private Solenoid groundShootPiston;  // ground hatch fire pistons
    private Solenoid hatchPiston; // hatch piston for hatch mech
    private Solenoid ballUpPiston; // piston to angle ball intake 
    private Solenoid climberPiston1;
    private Solenoid climberPiston2;

// Sensors
    private Encoder encoder; //elevator encoder
  
    private DigitalInput limitSwitch; // limit switch to stop the elevator

    private AnalogPotentiometer fourtwenty; // pot on the 
    private AnalogPotentiometer ballRollerArmPot;

    private Ultrasonic climbUltrasonic1;
    private Ultrasonic climbUltrasonic2;

    private AnalogGyro climbGyro;
    
    private DigitalOutput light1;
    private DigitalOutput light2;
    private DigitalOutput light3;

// Controllers
    private Joystick player2;
    private Joystick autoBox;

// Booleans
    private final boolean talon2Inverted = false;
    private final boolean intakeMotor2Inverted = false;

    public Elevator(Joystick player1) {
    // Talons
        talon1 = new TalonSRX(7);
        talon2 = new TalonSRX(8);
        ballIntake1 = new TalonSRX(9);
        ballIntake2 = new TalonSRX(10);
        overIntake1 = new TalonSRX(11);
        overIntake2 = new TalonSRX(12);
        groundTalon1 = new TalonSRX(13);
        // groundTalon2 = new TalonSRX(14); //not using 2 motors 
        overPull = new TalonSRX(14);
        climberTalon = new TalonSRX(15);

    // Solenoid
        elevatorShifter = new Solenoid(1);
        hatchPiston = new Solenoid(2);
        ballUpPiston = new Solenoid(3);
        groundShootPiston = new Solenoid(4);
        climberPiston1 = new Solenoid(5);
        climberPiston2 = new Solenoid(6);
        
    // Sensors 
        fourtwenty = new AnalogPotentiometer(1, 360, 0);
        encoder = new Encoder(4, 5);
        limitSwitch = new DigitalInput(6);
        ballRollerArmPot = new AnalogPotentiometer(0, 360, 0);
        climbUltrasonic1 = new Ultrasonic(8, 9);
        climbUltrasonic2 = new Ultrasonic(10, 11);
        climbGyro = new AnalogGyro(2);
        light1 = new DigitalOutput(12);
        light2 = new DigitalOutput(13);
        light3 = new DigitalOutput(14);

    // Joysticks 
        player2 = new Joystick(1);
        autoBox = new Joystick(2);

    // Classes
        hatchElevator = new hatchElevator(talon1, talon2, encoder, player2, talon2Inverted, hatchPiston);
        ballElevator = new ballElevator(talon1, talon2, encoder, player2, talon2Inverted, ballIntake1, ballIntake2,
                intakeMotor2Inverted, ballUpPiston, overIntake1, overIntake2, overPull);
        groundIntake = new groundIntake( groundTalon1, player1, player2, fourtwenty, groundShootPiston);
        manualOverride = new manualOverride(talon1, talon2, player2, talon2Inverted, ballIntake1, 
                ballIntake2, intakeMotor2Inverted, ballUpPiston, hatchPiston, overIntake1, overIntake2, overPull);
        getGroundIntakeOutOfWay = new getGroundIntakeOutOfWay(groundTalon1, groundTalon2, groundShootPiston);
        climb = new Climb(talon1, talon2, encoder, climbGyro, climbUltrasonic1, climbUltrasonic2);

        
    }

    public void init() {
        ballElevator.init();
        hatchElevator.init();
    }

    public void update() {
        //we haven't made an autobox yet
        //if autobox getting button
        //groundIntakeOutofWay.update(true);
        //else
        //groundIntakeOutOfWay.update(false);
        if (autoBox.getRawButton(1)){
            getGroundIntakeOutOfWay.update(true);
        } else {
            getGroundIntakeOutOfWay.update(false);
        }

        groundIntake.update(Math.abs(getRightTrigger()) > 0.1, autoBox.getRawButton(1));//button 9 = manual
        if (Math.abs(getRightTrigger()) > 0.1) {
            manualOverride.update(true);
            
        } else {
            manualOverride.update(false);
            if (getRightBumper()) {// ball
                ballElevator.update(true, limitSwitch.get());
                hatchElevator.update(false, limitSwitch.get());
            } else {// hatch
                ballElevator.update(false, limitSwitch.get());
                hatchElevator.update(true, limitSwitch.get());
            }
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