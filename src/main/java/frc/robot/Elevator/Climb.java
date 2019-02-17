/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Elevator;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Add your docs here.
 */

public class Climb {

    // Talons

    private TalonSRX talon1;
    private TalonSRX talon2;
    private TalonSRX climberTalon;

    // Sensors
    private Encoder encoder;
    private AnalogGyro climbGyro;
    private DigitalInput climbSwitch1;
    private DigitalInput climbSwitch2;

    public Climb(TalonSRX talon1, TalonSRX talon2, Encoder encoder, AnalogGyro climbGyro, DigitalInput climbSwitch1,
            DigitalInput climbSwitch2, TalonSRX climberTalon, Solenoid climberPiston1, Solenoid climberPiston2) {
        this.talon1 = talon1;
        this.talon2 = talon2;
        this.encoder = encoder;
        this.climbGyro = climbGyro;
        this.climbSwitch1 = climbSwitch1;
        this.climbSwitch2 = climbSwitch2;
        this.climberTalon = climberTalon;
    }

}
