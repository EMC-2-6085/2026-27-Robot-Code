package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Lights {
    public DigitalChannel whiteLED, redLED, greenLED, blueLED;

    public Lights(HardwareMap hardwareMap) {
        whiteLED = hardwareMap.get(DigitalChannel.class, "ledWhite");
        redLED = hardwareMap.get(DigitalChannel.class, "ledRed");
        greenLED = hardwareMap.get(DigitalChannel.class, "ledGreen");
        blueLED = hardwareMap.get(DigitalChannel.class, "ledBlue");

        whiteLED.setMode(DigitalChannel.Mode.OUTPUT);
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        blueLED.setMode(DigitalChannel.Mode.OUTPUT);
    }
    
    public void setWhite(boolean state) {
        whiteLED.setState(!state);
    }
    public void setRed(boolean state) {
        redLED.setState(!state);
    }
    public void setGreen(boolean state) {
        greenLED.setState(!state);
    }
    public void setBlue(boolean state) {
        blueLED.setState(!state);
    }
    public void whiteOn() {
        setWhite(true);
        setRed(false);
        setGreen(false);
        setBlue(false);
    }
    public void redOn() {
        setWhite(false);
        setRed(true);
        setGreen(false);
        setBlue(false);
    }
    public void greenOn() {
        setWhite(false);
        setRed(false);
        setGreen(true);
        setBlue(false);
    }
    public void blueOn() {
        setWhite(false);
        setRed(false);
        setGreen(false);
        setBlue(true);
    }
}