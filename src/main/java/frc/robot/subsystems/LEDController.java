// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDController extends SubsystemBase {

  //initialize variables
  int ledPort = 9;
  int ledLength = 36;

  AddressableLED led = new AddressableLED(ledPort);
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(ledLength);

  String prevState = "none";
  String currentState = "none";

  public LEDController() {
    led.setLength(ledBuffer.getLength());

    // Set the data
    led.setData(ledBuffer);
    led.start();
  }

  public void readyCollect(){
    //green
    setFullStripColor(0, 255, 0, 0.75f);
    currentState = "readyCollect";
  }

  public void hasGamePiece(){
    //blue
    setFullStripColor(0, 0, 255, 0.75f);
    currentState = "hasGamePiece";
  }

  public void readyScore(){
    //pink
    setFullStripColor(230, 50, 200, 0.75f);
    currentState = "readyScore";
  }

  public void readyDrop(){
    //cyan
    setFullStripColor(96, 209, 149, 0.75f);
    currentState = "readyDrop";
  }

  public void believeScored(){
    //red
    setFullStripColor(255, 0, 0, 0.75f);
    currentState = "believeScored";
  }

  public void wantsCone(){
    //yellow
    setFullStripColor(255, 230, 30, 0.75f);
    currentState = "wantsCone";
  }
  
  public void wantsCube(){
    //purple
    setFullStripColor(220, 30, 240, 0.75f);
    currentState = "wantsCube";
  }

  public void fullWhite(){
    //white
    setFullStripColor(255, 255, 255, 0.75f);
    currentState = "fullWhite";
  }

  public void blink(){
    //flashes between blue and red
    if ((System.currentTimeMillis() % 1000) < 500){
      setFullStripColor(0, 0, 255, 0.75f);
    } else {
      setFullStripColor(255, 0, 0, 0.75f);
    }

    currentState = "blink";
  }

  //turn off LEDs
  public void stop(){
    led.stop();
  }

  // future code here to decide what state to use - for now just use shuffleboard
  //public void chooseState(){
  //  blink();
  //}

  @Override
  public void periodic() {

    //chooseState();

    //for states like blink, we need to keep refreshing it. otherwise, we should only set it once
    if (prevState.equals(currentState)){
      if (currentState.equals("blink")){ 
        led.setData(ledBuffer);
      }
    } else {
      led.setData(ledBuffer);
      prevState = currentState;
    }
  }

  /**
   * lets you set the full strip color and brightness
   * 
   * @param r value 0-255
   * @param g value 0-255
   * @param b value 0-255
   * @param brightness decimal value for brightness where 1.0 is full brightness, 0.5 is half, etc.
   */
  private void setFullStripColor (int r, int g, int b, float brightness){
    for(int i=0; i<=ledLength; i++){
      ledBuffer.setRGB(i, (int) (r * brightness), (int) (g * brightness), (int) (b * brightness));
    } 
  }
}