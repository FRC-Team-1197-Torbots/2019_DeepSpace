package frc.robot.Elevator;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.PID_Tools.*;

public class ballPID {
    private VictorSPX armTalon1;
    private VictorSPX armTalon2;
    private VictorSPX shootTalon;
    private AnalogPotentiometer fourtwenty;
    private TorDerivative derivative;
    
    //intake speed variables
    private final double intakePower = -1;
    private final double outakePower = 1;
    //not tunable
    private double intakeCurrentRunningPower = 0;

    //PID values to tune
    private final double flatAngle = 0;//reading on the pot (fourtwenty) when it is flat
    private final double polarity = 1;//1 if up is positive, -1 if up is negative
    private final double kP = 0;
    private final double kI = 0;
    private final double kD = 0;
    private final double tolerance = 2;//in degrees and is when kI stops
    private final double dt = 0.005;//should be the same as everything else
    //thats it on tunable things

    private double currentAngle;
    private double error;
    private double errorSum;
    private double errorDerivative;
    private double output;
    private boolean onTarget = false;


    public ballPID(VictorSPX armTalon1, VictorSPX armTalon2, 
        VictorSPX shootTalon, AnalogPotentiometer fourtwenty) {
            this.armTalon1 = armTalon1;
            this.armTalon2 = armTalon2;
            this.shootTalon = shootTalon;
            this.fourtwenty = fourtwenty;
            derivative = new TorDerivative(dt);
            derivative.reset();
    }

    public void update(double targetAngle, int mode) {
        setMode(mode);
        shootTalon.set(ControlMode.PercentOutput, intakeCurrentRunningPower);

        currentAngle = (fourtwenty.get() - flatAngle) * polarity;

        error = currentAngle - targetAngle;
        errorSum += error;
        if(Math.abs(error) < tolerance) {
            errorSum = 0;
            onTarget = true;
        } else {
            onTarget = false;
        }
        errorDerivative = derivative.estimate(error);
        output = (error * kP) + (errorDerivative * kD) + (errorSum * kI);
        armTalon1.set(ControlMode.PercentOutput, output);
        armTalon2.set(ControlMode.PercentOutput, output);
    }

    public void setMode(int mode) {
        if(mode == 0) {
            intakeCurrentRunningPower = 0;
        } else if(mode == 1) {
            intakeCurrentRunningPower = outakePower;
        } else if(mode == -1) {
            intakeCurrentRunningPower = intakePower;
        }
    }

    public boolean isOnTarget() {
        return onTarget;
    }


}