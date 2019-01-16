package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;

public class Elevator {
    /*
    tuneable variables------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    */
    //our variables
    final double kP = 0.0;
    final double kI = 0.0;
    final double kD = 0.0;
    final double pidOnTolerance = 0.3;//this is in meters and should be kind of large as we are using bang bang till PID turns on
    final double encoderTicksPerMeter = 1.0;//this is how many ticks there are per meter the elevator goes up
    final double lowHatch = 1.0;//these three are the heights of what we want to go to
    final double intakeHatch = 2.0;
    final double highHatch = 3.0;
    final double absoluteMaxUpwardVelocity = 1.0;//don't make it higher than 1.0
    final double absoluteMaxDownwardVelocity = 1.0;//don't make it higher than 1.0
    /*
    -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    */

    int initialTicks;

    //our hardware
    TalonSRX talon1;
    TalonSRX talon2;
    Encoder encoder;
    Joystick player2;

    public static enum theElevator {
        IDLE, lowHatchPID, intakeHatchPID, highHatchPID, goTolowHatchPID, goTointakeHatchPID, goToHighHatchPID;
        private theElevator() {}
    }

    public theElevator elevator = theElevator.IDLE;

    public Elevator(int motorPort1, int motorPort2, int encoderPort1, int encoderPort2, Joystick player2, boolean talon2Inverted) {
        this.talon1 = new TalonSRX(motorPort1);
        this.talon2 = new TalonSRX(motorPort2);
        this.encoder = new Encoder(encoderPort1, encoderPort2);
        this.talon2.follow(talon1);
        this.talon2.setInverted(talon2Inverted);
        this.player2 = player2;
    }
    public void init() {
        initialTicks = encoder.get();
    }


    public void setPercentSpeed(double speed) {
        talon1.set(ControlMode.PercentOutput, speed);
    }

    public double height() {
        return ((encoder.get() - initialTicks)/encoderTicksPerMeter);
    }

    public void update() {
        switch(elevator) {
            case IDLE:
                break;
            case lowHatchPID:
                break;
            case intakeHatchPID:
                break;
            case highHatchPID:
                break;
            case goTolowHatchPID:
                break;
            case goTointakeHatchPID:
                break;
            case goToHighHatchPID:
                break;
        }
    }
}
