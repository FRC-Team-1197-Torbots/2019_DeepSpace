package frc.robot.Elevator;

import static org.junit.Assert.assertEquals;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class manualOverride {
    // it will need joystick player2
    // it will need both talons that go up
    // it will need both talons that shoot
    // it will need the solenoid for the ball intake to go up

// -------------   Joystick    ------------------------------------
    // Hardware
    private Joystick player2; //player 2 controls manual override
// ---------------------------------------------------------


// -------------   Elevator    ------------------------------------
    // Elevator Gearbox motors
    private TalonSRX talon1;
    private TalonSRX talon2;
    
    private double elevatorAxis; // joystick axis 1 for moving elevator
    private final double elevatorHoldSpeed = 0.1; // << ADJUST, constant speed that will keep the elevator held in spot
// ---------------------------------------------------------



// -------------   Ball Elevator    ------------------------------------
    // Ball Intake/Shooter motors
    private TalonSRX intakeMotor1; //I would make it a button to go in, a button to go out
    private TalonSRX intakeMotor2;

    //piston to angle ball intake for 2nd rocket level
    private Solenoid upPiston;

    private final double intakeSpeed = 1.0; // << ADJUST, speed to intake the ball
    private final double normalOutakeSpeed = -0.5; // << ADJUST, speed to outtake the ball, cargo Bay
    private final double hardOutakeSpeed = -1.0;// << ADJUST, speed to outtake the ball, rocket 2nd level, when tilted up
// ---------------------------------------------------------
 


// -------------   Hatch Elevator    ------------------------------------
    // Piston for extending/retracting Hatch Mechanism
    private Solenoid hatchPiston;//this will just be one button control
// ---------------------------------------------------------



// -------------   Ball Roller    ------------------------------------
    // Motors for Ball Roller Arm, Ball Roller 
    private TalonSRX overIntake1; // Ball Roller Arm Motor 1
    private TalonSRX overIntake2; // Ball Roller Arm Motor 2
    private TalonSRX ballRoller; // Motor for spinning Ball Roller Axel

    private double rollerArmAxis; // joystick axis 5 for moving the arm
    
    private final double rollerArmHoldSpeed = 0.1; // << ADJUST, constant hold Speed for the roller Arm 
    private final double ballRollerSpeed = 0.45; // << ADJUST, intake speed for the ball Roller
// ---------------------------------------------------------

    public manualOverride(TalonSRX talon1, TalonSRX talon2, Joystick player2, boolean talon2Inverted,
            TalonSRX intakeMotor1, TalonSRX intakeMotor2, boolean intakeMotor2Inverted, Solenoid upPiston,
            Solenoid hatchPiston, TalonSRX overIntake1, TalonSRX overIntake2, TalonSRX ballRoller) {
        this.talon1 = talon1;
        this.talon2 = talon2;
        this.player2 = player2;
        this.talon2.follow(this.talon1);
        this.talon2.setInverted(talon2Inverted);
        this.intakeMotor1 = intakeMotor1;
        this.intakeMotor2 = intakeMotor2;
        this.intakeMotor2.follow(this.intakeMotor1);
        this.intakeMotor2.setInverted(intakeMotor2Inverted);
        this.upPiston = upPiston;
        this.hatchPiston = hatchPiston;
        this.overIntake1 = overIntake1;
        this.overIntake2 = overIntake2;
        this.ballRoller = ballRoller;
    }

    public void update(boolean running) {
        // Set joystick Axis to variables
        elevatorAxis = player2.getRawAxis(1);
        rollerArmAxis = player2.getRawAxis(5);
        

        if (running) {//we want to double check in here right trigger is being pressed for the manual override

            //Elevator movement
            if (Math.abs(elevatorAxis) > 0.15) { // if you move the elevator axis to move the elevator manually,
                setElevatorSpeed(elevatorAxis + elevatorHoldSpeed);
            } else {
                setElevatorSpeed(elevatorHoldSpeed);
            }


            //if y is pressed, raise the ball intake and extend the piston 
            if (getButtonY()){
                upPiston.set(true);
            } else {
                upPiston.set(false);
            }


            // if x is pressed, extend the hatch mechanism
            if (getButtonX()){
                hatchPiston.set(true);
            } else {
                hatchPiston.set(false);
            }


            // Ball Mechanims
            if (getButtonA()){//if a is pressed spin the ball roller in and the ball intake in
                setBallIntakeSpeed(intakeSpeed);
                ballRoller.set(ControlMode.PercentOutput, ballRollerSpeed);
            } else if (getLeftBumper()){ //if left bumper is pressed, outtake the ball
                if (getButtonY()){ // if the ball intake is pointed up, shoot hard to get to the rocket 2nd level
                    setBallIntakeSpeed(hardOutakeSpeed);
                } else { // if the ball intake if not tilted, shoot normal for the cargo bay
                    setBallIntakeSpeed(normalOutakeSpeed);
                }
            } else {
                setBallIntakeSpeed(0);
                ballRoller.set(ControlMode.PercentOutput, 0);
            }

            
            // Ball Roller Arm
            if (Math.abs(rollerArmAxis) > 0.15) { // if you move the right axis up or down to move the roller arm speed
                setRollerArmSpeed(rollerArmAxis + rollerArmHoldSpeed);
            } else {
                setRollerArmSpeed(rollerArmHoldSpeed);
            }


        }

    }

    public void setElevatorSpeed(double elevatorSpeed) { // method for setting elevator speed
        talon1.set(ControlMode.PercentOutput, elevatorSpeed);
        talon2.set(ControlMode.PercentOutput, elevatorSpeed);
    }

    public void setRollerArmSpeed(double rollerArmSpeed) { // method for setting ballroller arm speed
        overIntake1.set(ControlMode.PercentOutput, rollerArmSpeed);
        overIntake2.set(ControlMode.PercentOutput, rollerArmSpeed);
    }

    public void setBallIntakeSpeed(double speed) { // method for setting ballroller arm speed
        intakeMotor1.set(ControlMode.PercentOutput, speed);
        intakeMotor2.set(ControlMode.PercentOutput, -speed); //flip one of motors
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