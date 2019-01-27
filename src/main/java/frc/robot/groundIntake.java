package frc.robot;
import edu.wpi.first.wpilibj.Solenoid;
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

    public groundIntake() {
        
    }
}