package frc.robot.Elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class getGroundIntakeOutOfWay {
    //has a solenoid to get the solenoid back and out of the way
    //has the two talons to move back and get out of the way
    private long currentTime = (long)(1000 * Timer.getFPGATimestamp());
    private long lastTime = currentTime;
    private TalonSRX groundIntake1;
    private TalonSRX groundIntake2;
    private Solenoid groundIntakeSolenoid;
    private final double upSpeed = 0.5;
    private final long waitTime = 1500;//how many mili seconds to make sure its up and stop moving


    public static enum outOfWay {
        IDLE, MOVEUP, OUTOFWAY;
        private outOfWay() {}
    }

    private outOfWay out = outOfWay.IDLE;

    public getGroundIntakeOutOfWay(TalonSRX groundIntake1, TalonSRX groundIntake2, Solenoid groundIntakeSolenoid) {
        this.groundIntake1 = groundIntake1;
        this.groundIntake2 = groundIntake2;
        this.groundIntakeSolenoid = groundIntakeSolenoid;
    }

    public void update(boolean running) {
        currentTime = (long)(1000 * Timer.getFPGATimestamp());
        if(running && out == outOfWay.IDLE) {//basically if it has to start
            out = outOfWay.MOVEUP;
        }
        switch(out) {
            case IDLE:
                lastTime = currentTime;
                break;
            case MOVEUP:
                //move both of them up
                groundIntake1.set(ControlMode.PercentOutput, upSpeed);//moving up the intake
                groundIntake2.set(ControlMode.PercentOutput, upSpeed);
                groundIntakeSolenoid.set(false);//retracting the pistons
                if(currentTime - lastTime > waitTime) {
                    out = outOfWay.OUTOFWAY;
                }
                break;
            case OUTOFWAY:
                groundIntake1.set(ControlMode.PercentOutput, 0);
                groundIntake2.set(ControlMode.PercentOutput, 0);
                if(!running) {
                    out = outOfWay.IDLE;
                }
                break;
        }
    }
}