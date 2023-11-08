package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Definition of Indicator class <BR>
 *
 * Lights that change color to indicate the desired pixel color<BR>
 *
 */


public class SSIndicators {
    public RevBlinkinLedDriver light;
    public Telemetry telemetry;

    public enum SSINDICATORS_LIGHT_COLOR {
        PURPLE,
        GREEN,
        YELLOW,
        RED,
        WHITE
    }

    public enum SSINDICATORS_LIGHT_STATE {
        ON,
        OFF
    }

    public SSINDICATORS_LIGHT_STATE SSIndicatorLightState = SSINDICATORS_LIGHT_STATE.OFF;
    public SSINDICATORS_LIGHT_COLOR SSIndicatorLightColor;

    public SSIndicators(HardwareMap hardwareMap){
        this.telemetry = telemetry;
        light = hardwareMap.get(RevBlinkinLedDriver.class, "indicator_light");
        initIndicators();
    }

    public void initIndicators(){

    }

    public void setLightToGreen() {
        turnLightOn();
        light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        SSIndicatorLightColor = SSINDICATORS_LIGHT_COLOR.GREEN;

    }

    public void setLightToRed() {
        turnLightOn();
        light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        SSIndicatorLightColor = SSINDICATORS_LIGHT_COLOR.RED;

    }

    public void setLightToPurple() {
        turnLightOn();
        light.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        SSIndicatorLightColor = SSINDICATORS_LIGHT_COLOR.PURPLE;
    }

    public void setLightToWhite() {
        turnLightOn();
        light.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        SSIndicatorLightColor = SSINDICATORS_LIGHT_COLOR.WHITE;
    }

    public void setLightToYellow() {
        turnLightOn();
        light.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        SSIndicatorLightColor = SSINDICATORS_LIGHT_COLOR.YELLOW;
    }

    public void turnLightOn(){
        if(SSIndicatorLightState != SSINDICATORS_LIGHT_STATE.ON){
            SSIndicatorLightState = SSINDICATORS_LIGHT_STATE.ON;
        }
    }

    public void turnLightOff(){
        if(SSIndicatorLightState != SSINDICATORS_LIGHT_STATE.OFF){
            SSIndicatorLightState = SSINDICATORS_LIGHT_STATE.OFF;
            light.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
    }

    public String getLightColor(){
        return SSIndicatorLightColor.name();
    }

    public void printDebugMessages(){
        telemetry.addData("Current Color: ", getLightColor());
        telemetry.addLine("=================");
    }

}

