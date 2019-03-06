package frc.robot.Elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

public class manualOverride {
    // it will need joystick player2
    // it will need both talons that go up
    // it will need both talons that shoot
    // it will need the solenoid for the ball intake to go up

    // ------------- Joystick ------------------------------------
    // Hardware
    private Joystick player2; // player 2 controls manual override
    // ---------------------------------------------------------

    // ------------- Elevator ------------------------------------
    // Elevator Gearbox motors
    private CANSparkMax talon1;
    private CANSparkMax talon2;

    private Solenoid elevatorShifter;

    private double elevatorAxis; // joystick axis 1 for moving elevator
    private final double elevatorHoldSpeed = 0.1; // << ADJUST, constant speed that will keep the elevator held in spot
    // ---------------------------------------------------------

    // ------------- Ball Elevator ------------------------------------
    // Ball Intake/Shooter motors
    private TalonSRX intakeMotor1;

    // piston to angle ball intake for 2nd rocket level
    private boolean upPistonActive = false;

    private final double intakeSpeed = 1.0; // << ADJUST, speed to intake the ball
    private final double normalOutakeSpeed = -0.5; // << ADJUST, speed to outtake the ball, cargo Bay
    private final double hardOutakeSpeed = -1.0;// << ADJUST, speed to outtake the ball, rocket 2nd level, when tilted
                                                // up
    // ---------------------------------------------------------

    /*
     * // ------------- Hatch Elevator ------------------------------------ //
     * Piston for extending/retracting Hatch Mechanism private Solenoid
     * hatchPiston;//this will just be one button control //
     * ---------------------------------------------------------
     * 
     */

    private double rollerArmAxis; // joystick axis 5 for moving the arm

    private final double rollerArmHoldSpeed = 0.1; // << ADJUST, constant hold Speed for the roller Arm
    private final double ballRollerSpeed = 0.45; // << ADJUST, intake speed for the ball Roller
    // ---------------------------------------------------------

    // ------------- Climb ------------------------------------
    // Climb controls
    private Solenoid climberPiston1;// this will just be one button control
    private Solenoid climberPiston2;
    private boolean climbPistonsActive;


    private TalonSRX climberTalon;
    private final double climberSpeed = 0.2;
    // ---------------------------------------------------------

    public manualOverride(CANSparkMax talon1, CANSparkMax talon2, Joystick player2, boolean talon2Inverted,
            TalonSRX intakeMotor1, boolean intakeMotor2Inverted,
            Solenoid hatchPiston, Solenoid elevatorShifter, Solenoid climberPiston1, Solenoid climberPiston2, TalonSRX climberTalon) {
        this.talon1 = talon1;
        this.talon2 = talon2;
        this.player2 = player2;
        this.talon2.follow(this.talon1);
        this.talon2.setInverted(talon2Inverted);
        this.intakeMotor1 = intakeMotor1;
        this.elevatorShifter = elevatorShifter;

        this.climberPiston1 = climberPiston1;
        this.climberPiston2 = climberPiston2;
        this.climberTalon = climberTalon;

    }

    public void update(boolean running) {
        // Set joystick Axis to variables
        elevatorAxis = player2.getRawAxis(1);
        rollerArmAxis = player2.getRawAxis(5);

        if (running) {// we want to double check in here right trigger is being pressed for the manual
                      // override
            elevatorShifter.set(false); // set elevator to low gear

            // Elevator movement
            if (Math.abs(elevatorAxis) > 0.15) { // if you move the elevator axis to move the elevator manually,
                setElevatorSpeed(elevatorAxis + elevatorHoldSpeed);
            } else {
                setElevatorSpeed(elevatorHoldSpeed);
            }


            // if y is pressed, raise the ball intake and extend the piston
            if (getButtonY()) {
                if (climbPistonsActive) {
                    climberPiston1.set(false);
                    climberPiston2.set(false);
                    climbPistonsActive = false;
                } else {
                    climberPiston1.set(true);
                    climberPiston2.set(true);
                    climbPistonsActive = true;
                }
            } else if (climbPistonsActive) {
                climberPiston1.set(true);
                climberPiston2.set(true);
            } else {
                climberPiston1.set(false);
                climberPiston2.set(false);
            }


            if (getLeftBumper()){
                climberTalon.set(ControlMode.PercentOutput, climberSpeed);
            } else {
                climberTalon.set(ControlMode.PercentOutput, 0);
            }

        }

    }

    public void setElevatorSpeed(double elevatorSpeed) { // method for setting elevator speed
        talon1.set(elevatorSpeed);
        talon2.set(elevatorSpeed);
    }

    public void setBallIntakeSpeed(double speed) { // method for setting ballroller arm speed
        intakeMotor1.set(ControlMode.PercentOutput, speed);
    }

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

    public boolean getLeftBumper() {
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