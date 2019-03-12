package frc.robot.Elevator;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

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
    private TalonSRX talon1;
    private TalonSRX talon2;

    private Solenoid elevatorShifter;

    private double elevatorAxis; // joystick axis 1 for moving elevator
    private final double elevatorHoldSpeed = -0.075; // << ADJUST, constant speed that will keep the elevator held in spot
    // ---------------------------------------------------------

    // ------------- Ball Elevator ------------------------------------
    // Ball Intake/Shooter motors
    private TalonSRX intakeMotor1;

    private VictorSPX ballIntakeArm1;
    private VictorSPX ballIntakeArm2;

    // piston to angle ball intake for 2nd rocket level
    private boolean upPistonActive = false;

    private final double intakeSpeed = 1.0; // << ADJUST, speed to intake the ball
    private final double normalOutakeSpeed = -0.5; // << ADJUST, speed to outtake the ball, cargo Bay
    private final double hardOutakeSpeed = -1.0;// << ADJUST, speed to outtake the ball, rocket 2nd level, when tilted
                                                // up
    // ---------------------------------------------------------

    /*
     * // ------------- Hatch Elevator ------------------------------------ //
     * Piston for extending/retracting Hatch Mechanism 
     * private Solenoid hatchPiston;//this will just be one button control //
     * ---------------------------------------------------------
     * 
     */
    private Solenoid hatchPiston;
    private double rollerArmAxis; // joystick axis 5 for moving the arm

    private final double rollerArmHoldSpeed = 0.1; // << ADJUST, constant hold Speed for the roller Arm
    private final double ballRollerSpeed = 0.45; // << ADJUST, intake speed for the ball Roller
    // ---------------------------------------------------------

    // ------------- Climb ------------------------------------
    // Climb controls
    private Solenoid climberPiston1;// this will just be one button control
    private Solenoid climberPiston2;
    private boolean climbPistonsActive;


    private VictorSPX climberTalon;
    private final double climberSpeed = 0.2;
    // ---------------------------------------------------------

    public manualOverride(TalonSRX talon1, TalonSRX talon2, Joystick player2, boolean talon2Inverted,
            VictorSPX ballIntakeArm1, VictorSPX ballIntakeArm2, 
            Solenoid elevatorShifter, Solenoid climberPiston1, Solenoid climberPiston2, VictorSPX climberTalon, Solenoid hatchPiston) {
        this.talon1 = talon1;
        this.talon2 = talon2;
        this.player2 = player2;
        this.talon2.follow(this.talon1);
        this.talon2.setInverted(talon2Inverted);

        this.ballIntakeArm1 = ballIntakeArm1;
        this.ballIntakeArm2 = ballIntakeArm2;

        this.elevatorShifter = elevatorShifter;

        this.climberPiston1 = climberPiston1;
        this.climberPiston2 = climberPiston2;
        this.climberTalon = climberTalon;
        this.hatchPiston = hatchPiston;
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


            // if y is pressed, extend the piston
            if (getLeftBumper()) {
                climberPiston1.set(true);
                climberPiston2.set(true);
            } else {
                climberPiston1.set(false);
                climberPiston2.set(false);
            }

            // manual for driving climber wheels
            if (getButtonB()){
                climberTalon.set(ControlMode.PercentOutput, climberSpeed);
            } else {
                climberTalon.set(ControlMode.PercentOutput, 0);
            }


            // move ball intake arm 

            if (Math.abs(rollerArmAxis) > 0.15) { 
                setBallIntakeArmSpeed(rollerArmAxis * rollerArmAxis);
            
            } else {
                setBallIntakeArmSpeed(0);

            }
            
            // activate the hatch piston
            if(player2.getRawButton(3)) {
                hatchPiston.set(true);
            } else {
                hatchPiston.set(false);
            }

        }

    }

    public void setElevatorSpeed(double elevatorSpeed) { // method for setting elevator speed
        talon1.set(ControlMode.PercentOutput,elevatorSpeed);
        talon2.set(ControlMode.PercentOutput, elevatorSpeed);
    }

    public void setBallIntakeArmSpeed(double speed) { // method for setting ballroller arm speed
        ballIntakeArm1.set(ControlMode.PercentOutput, -speed);
        ballIntakeArm2.set(ControlMode.PercentOutput, speed);

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