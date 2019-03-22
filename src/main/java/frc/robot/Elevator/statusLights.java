/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Elevator;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;

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
    private long lastTime = 0;



public static enum theRainbow{
    iteration1, iteration2,
    iteration3,
    iteration4,
    iteration5,
    iteration6,
    iteration7;
    private theRainbow(){}
    

}
public theRainbow rainbowLights = theRainbow.iteration1;

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
        long currentTime = (long) (Timer.getFPGATimestamp());
        
  

        switch (rainbowLights){
            case iteration1:
                displayGreenLights();
                if (currentTime - lastTime > 1.5){
                    rainbowLights = theRainbow.iteration2;
                    lastTime = currentTime;
                }
            break;
            case iteration2:
                displayCyanLights();
                if (currentTime - lastTime > 1.5){
                    rainbowLights = theRainbow.iteration3;
                    lastTime = currentTime;
                }
            break;
            case iteration3:
                displayRedLights();
                if (currentTime - lastTime > 1.5){
                    rainbowLights = theRainbow.iteration4;
                    lastTime = currentTime;
                }
            break;
            case iteration4:
                displayYellowLights();
                if (currentTime - lastTime > 1.5){
                    rainbowLights = theRainbow.iteration5;
                    lastTime = currentTime;
                }
            break;
            case iteration5:
                displayWhiteLights();
                if (currentTime - lastTime > 1.5){
                    rainbowLights = theRainbow.iteration6;
                    lastTime = currentTime;
                }
                break;
            case iteration6:
                displayMagentaLights();
                if (currentTime - lastTime > 1.5){
                    rainbowLights = theRainbow.iteration7;
                    lastTime = currentTime;
                }
                break;
            case iteration7:
                displayGreenLights();
                if (currentTime - lastTime > 1.5){
                    rainbowLights = theRainbow.iteration1;
                    lastTime = currentTime;
                }
                break;
           



        }

        // displayCyanLights();
        // Timer.delay(3);
        // displayGreenLights();
        // Timer.delay(3);
        // displayRedLights();
        // Timer.delay(3);
        // displayYellowLights();
        // Timer.delay(3); 
        // displayWhiteLights();
        // Timer.delay(3);
        // displayAquaLights();
        // Timer.delay(3);
        // displayMagentaLights();
        // Timer.delay(3);
       
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

    public void displayAquaLights(){
        light1.set(rainbow[0]);
        light2.set(rainbow[1]);
        light3.set(rainbow[2]);
    }

}
