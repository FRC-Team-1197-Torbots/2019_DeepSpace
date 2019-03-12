package frc.robot.Elevator;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.PID_Tools.*;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class ballArm {
    private VictorSPX armTalon1;
    private VictorSPX armTalon2;
    private VictorSPX shootTalon;
    private AnalogPotentiometer fourtwenty;
    private TorDerivative derivative;
    
    //intake speed variables
    private final double intakePower = -0.7;
    private final double intakePower2 = -0.5;
    private final double outakePower = 1;
    private final double outakePower2 = 0.675;
    //not tunable
    private double intakeCurrentRunningPower = 0;

    //PID values to tune
    private final double flatAngle = 157.09216944023322;//reading on the pot (fourtwenty) when it is flat
    private final double polarity = -1;//1 if up is positive, -1 if up is negative
    private final double kP = 0.006;
    private final double kI = 0.0001;
    private final double kD = -0.001;
    private final double kF = -0.17;
    private final double tolerance = 2;//in degrees and is when kI stops
    private final double dt = 0.005;//should be the same as everything else
    private final double maxSpeedUp = 0.3;
    private final double maxSpeedDown = -0.6;
    //thats it on tunable things

    private double currentAngle;
    private double error;
    private double errorSum;
    private double errorDerivative;
    private double output;
    private boolean onTarget = false;
    private long currentTime = (long)(1000 * Timer.getFPGATimestamp());
    private long lastTime = currentTime;


    public ballArm(VictorSPX armTalon1, VictorSPX armTalon2, 
        VictorSPX shootTalon, AnalogPotentiometer fourtwenty) {
            this.armTalon1 = armTalon1;
            this.armTalon2 = armTalon2;
            this.shootTalon = shootTalon;
            this.fourtwenty = fourtwenty;
            derivative = new TorDerivative(dt);
            derivative.reset();
    }

    public void update(double targetAngle) {
        shootTalon.set(ControlMode.PercentOutput, intakeCurrentRunningPower);

        currentAngle = (fourtwenty.get() - flatAngle) * polarity;
        SmartDashboard.putNumber("current angle", currentAngle);

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
        output += kF * Math.cos((currentAngle * Math.PI) / 180.0);
        if(output > maxSpeedUp) {
            output = maxSpeedUp;
        } else if(output < maxSpeedDown) {
            output = maxSpeedDown;
        }
        armTalon1.set(ControlMode.PercentOutput, -output);
        armTalon2.set(ControlMode.PercentOutput, output);
    }

    public void setMode(int mode) {
        currentTime = (long)(1000 * Timer.getFPGATimestamp());
        if(mode == 0) {
            if(currentTime - lastTime < 400) {
                intakeCurrentRunningPower = -0.25;
            } else {
                intakeCurrentRunningPower = 0;
            }
        } else if(mode == 1) {
            intakeCurrentRunningPower = outakePower;
        } else if(mode == 2) {
            intakeCurrentRunningPower = outakePower2;
        } else if(mode == -1) {
            lastTime = currentTime;
            intakeCurrentRunningPower = intakePower;
        } else if (mode == -2){
            lastTime = currentTime;
            intakeCurrentRunningPower = intakePower2;
        }
    }

    public boolean isOnTarget() {
        return onTarget;
    }


}