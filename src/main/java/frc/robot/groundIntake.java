package frc.robot;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class groundIntake {
    //it will need both talons to go up. It will need the ground intake talons. It will need the encoder to make sure the elevator
    //is high enough before the ground intake deploys
    //it will need a potentiometer for the ground intake
    //it will need both joysticks

    private TalonSRX elevatorTalon1;//talons for the up and down
    private TalonSRX elevatorTalon2;
    private TalonSRX groundTalon1;
    private TalonSRX groundTalon2;
    private Joystick player1;
    private Joystick player2;
    private Potentiometer fourtwenty;//ITS THE POT

    //the state machine
    public static enum ground {
        IDLE, DOWN, PULLEDBACK, SHOOTPOS, SHOOTING, DONE;
        private ground () {}
    }
    private ground groundIntake = ground.IDLE;

    public groundIntake(TalonSRX elevatorTalon1, TalonSRX elevatorTalon2, TalonSRX groundTalon1, TalonSRX groundTalon2,
        Joystick player1, Joystick player2, Potentiometer fourtwenty) {
        this.elevatorTalon1 = elevatorTalon1;
        this.elevatorTalon2 = elevatorTalon2;
        this.groundTalon1 = groundTalon1;
        this.groundTalon2 = groundTalon2;
        this.player1 = player1;
        this.player2 = player2;
        this.fourtwenty = fourtwenty;
    }

    public void update(boolean elevatorManualOverriding, boolean running) {
        
    }

    public void stateMachineRun() {

    }






}