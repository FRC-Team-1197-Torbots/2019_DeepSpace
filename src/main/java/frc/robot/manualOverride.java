package frc.robot;

import static org.junit.Assert.assertEquals;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class manualOverride {
    // it will need joystick player2
    // it will need both talons that go up
    // it will need both talons that shoot
    // it will need the solenoid for the ball intake to go up

    // our hardware
    private TalonSRX talon1;
    private TalonSRX talon2;
    private Joystick player2;

   private TalonSRX overIntake1;// since the 971 intake only matters for this ball elevator
    private TalonSRX overIntake2;
    //Brennan - should have 2

    private Solenoid upPiston;//for the ball intake
    //make it a button to activate

 // hatch elevator hardware
    private Solenoid hatchPiston;//this will just be one button control

    // ball elevator hardware
    private TalonSRX intakeMotor1;//I would make it a button to go in, a button to go out
    private TalonSRX intakeMotor2;
    private final double intakeSpeed = 1.0;
    private final double normalOutakeSpeed = -0.5;
    private final double hardOutakeSpeed = -1.0;//make it this when we are pointed up with the ball elevator piston


    

   
    // elevator variables
    private double elevatorAxis; // joystick axis 1 for moving elevator
    private final double elevatorHoldSpeed = 0.1; // constnat speed that will keep the elevator held in spot

    //ball roller arm variables
    private double rollerArmAxis;
    private final double rollerArmHoldSpeed = 0.1;
    private double ballRollerSpeed;//do the same as for the elevator
        //need another talon for the motor on the arm for the ball roller

    public manualOverride(TalonSRX talon1, TalonSRX talon2, Joystick player2, boolean talon2Inverted,
            TalonSRX intakeMotor1, TalonSRX intakeMotor2, boolean intakeMotor2Inverted, Solenoid upPiston,
            Solenoid hatchPiston) {
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
    }

    public void update(boolean running) {
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

            //if a is pressed spin the ball roller in and the ball intake in
            if (getButtonA()){
                intakeMotor1.set(ControlMode.PercentOutput, intakeSpeed );
                intakeMotor2.set(ControlMode.PercentOutput, intakeSpeed);
                // rollerbar set speed in
            } else if (getLeftBumper()){ //if left bumper is pressed, outtake the ball
                intakeMotor1.set(ControlMode.PercentOutput, normalOutakeSpeed); //not sure which speed yet
                intakeMotor2.set(ControlMode.PercentOutput, normalOutakeSpeed);

            } else {
                intakeMotor1.set(ControlMode.PercentOutput, 0);
                intakeMotor2.set(ControlMode.PercentOutput, 0);
            }

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