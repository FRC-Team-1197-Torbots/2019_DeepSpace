package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Elevator {
    //this elevator has the hatch elevator and the ball elevator in it
    private hatchElevator hatchElevator;
    private ballElevator ballElevator;

    //hardware
    private TalonSRX talon1;//this talon is the "drive talon for the elevator"
    private TalonSRX talon2;//this is the second one
    private TalonSRX ballIntake1;
    private TalonSRX ballIntake2;
    private Encoder encoder;
    private Solenoid hatchPiston;
    private Solenoid ballUpPiston;
    private DigitalInput limitSwitch;

    //our controller
    private Joystick player2;
    private Joystick player1;

    private final boolean talon2Inverted = false;
    private final boolean intakeMotor2Inverted = false;

    public Elevator(Joystick player1) {
        talon1 = new TalonSRX(7);
        talon2 = new TalonSRX(8);
        ballIntake1 = new TalonSRX(9);
        ballIntake2 = new TalonSRX(10);
        encoder = new Encoder(0, 1);
        hatchPiston = new Solenoid(2);
        ballUpPiston = new Solenoid(3);
        limitSwitch = new DigitalInput(6);
        hatchElevator = new hatchElevator(talon1, talon2, encoder, player2, talon2Inverted, hatchPiston);
        ballElevator = new ballElevator(talon1, talon2, encoder, player2, talon2Inverted, 
                                            ballIntake1, ballIntake2, intakeMotor2Inverted, ballUpPiston);
        this.player1 = player1;
    }

    public void init() {
        ballElevator.init();
        hatchElevator.init();
    }

    public void update() {
        if(getRightBumper()) {//ball
            ballElevator.update(true, limitSwitch.get());
            hatchElevator.update(false, limitSwitch.get());
        } else {//hatch
            ballElevator.update(false, limitSwitch.get());
            hatchElevator.update(true, limitSwitch.get());
        }
    }

     /*
	 *  The following are a bunch of accessor methods to obtain input from the controller.
	 */
	public double getLeftX(){
		return player2.getRawAxis(0);
	}

	public double getLeftY(){
		return player2.getRawAxis(1);
	}

	public double getRightX(){
		return player2.getRawAxis(4);
	}

	public double getRightTrigger(){
		return player2.getRawAxis(3);
	}

	public boolean getShiftButton(){
		return player2.getRawButton(5);
	}

	public boolean getRightBumper(){
		return player2.getRawButton(6);
	}

	public boolean getButtonA(){
		return player2.getRawButton(1);
	}

	public boolean getButtonB(){
		return player2.getRawButton(2);
	}

	public boolean getButtonX(){
		return player2.getRawButton(3);
	}

	public boolean getButtonY(){
		return player2.getRawButton(4);
	}
}