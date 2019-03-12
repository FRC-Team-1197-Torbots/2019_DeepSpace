/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Elevator;

import edu.wpi.first.wpilibj.DigitalOutput;

/**
 * Add your docs here.
 */
public class statusLights {
    private DigitalOutput light1;
    private DigitalOutput light2;
    private DigitalOutput light3;

    private boolean[] pattern1 = {false, false, false};
    private boolean[] pattern2 = {false, false, true};
    private boolean[] pattern3 = {false, true, false};
    private boolean[] pattern4 = {false, true, true};
    private boolean[] pattern5 = {true, false, false};
    private boolean[] pattern6 = {true, false, true};
    private boolean[] pattern7 = {true, true, false};
    private boolean[] pattern8 = {true, true, true};






    public statusLights(DigitalOutput light1, DigitalOutput light2, DigitalOutput light3){
        this.light1 = light1;
        this.light2 = light2;
        this.light3 = light3;

    }

// 1
    public void displayGreenLights(){
        light1.set(pattern1[0]);
        light2.set(pattern1[1]);
        light3.set(pattern1[2]);
    }
//  2
    public void displayRainbowLights(){
        light1.set(pattern1[0]);
        light2.set(pattern1[1]);
        light3.set(pattern1[2]);
    }
// 3
    public void displayRedLights(){
        light1.set(pattern1[0]);
        light2.set(pattern1[1]);
        light3.set(pattern1[2]);
    }
// 4

    public void displayCyanLights(){
        light1.set(pattern1[0]);
        light2.set(pattern1[1]);
        light3.set(pattern1[2]);
    }
// 5

    public void displayMagentaLights(){
        light1.set(pattern1[0]);
        light2.set(pattern1[1]);
        light3.set(pattern1[2]);
    }
// 6

    public void displayYellowLights(){
        light1.set(pattern1[0]);
        light2.set(pattern1[1]);
        light3.set(pattern1[2]);
    }
// 7
    public void displayWhiteLights(){
        light1.set(pattern1[0]);
        light2.set(pattern1[1]);
        light3.set(pattern1[2]);
    }

}
