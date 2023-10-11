package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class SSIndicators {
    public RevBlinkinLedDriver light;

    public enum SSINDICATORS_LIGHT_COLOR {
        PURPLE,
        GREEN,
        YELLOW,
        WHITE
    }

    public enum SSINDICATORS_LIGHT_STATE {
        ON,
        OFF
    }

    public SSINDICATORS_LIGHT_STATE SSIndicatorLightState = SSINDICATORS_LIGHT_STATE.OFF;
    public SSINDICATORS_LIGHT_COLOR SSIndicatorLightColor;

    public SSIndicators(HardwareMap hardwareMap){
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


}
