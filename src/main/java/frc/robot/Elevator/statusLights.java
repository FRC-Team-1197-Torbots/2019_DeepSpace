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

    private boolean[] off = {false, false, false};
    private boolean[] blue = {false, false, true};
    private boolean[] green = {false, true, false};
    private boolean[] rainbow = {false, true, true}; //aqua light blue
    private boolean[] red = {true, false, false};
    private boolean[] magenta = {true, false, true};
    private boolean[] yellow = {true, true, false};
    private boolean[] white = {true, true, true};






    public statusLights(DigitalOutput light1, DigitalOutput light2, DigitalOutput light3){
        this.light1 = light1;
        this.light2 = light2;
        this.light3 = light3;

    }

// 1
    public void displayGreenLights(){
        light1.set(green[0]);
        light2.set(green[1]);
        light3.set(green[2]);
    }
//  2
    public void displayRainbowLights(){
        light1.set(rainbow[0]);
        light2.set(rainbow[1]);
        light3.set(rainbow[2]);
    }
// 3
    public void displayRedLights(){
        light1.set(red[0]);
        light2.set(red[1]);
        light3.set(red[2]);
    }
// 4

    public void displayCyanLights(){
        light1.set(blue[0]);
        light2.set(blue[1]);
        light3.set(blue[2]);
    }
// 5

    public void displayMagentaLights(){
        light1.set(magenta[0]);
        light2.set(magenta[1]);
        light3.set(magenta[2]);
    }
// 6

    public void displayYellowLights(){
        light1.set(yellow[0]);
        light2.set(yellow[1]);
        light3.set(yellow[2]);
    }
// 7
    public void displayWhiteLights(){
        light1.set(white[0]);
        light2.set(white[1]);
        light3.set(white[2]);
    }

}
