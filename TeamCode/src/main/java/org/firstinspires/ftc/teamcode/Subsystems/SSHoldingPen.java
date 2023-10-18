package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GameOpModes.AbbyMode;

public class SSHoldingPen {

    public DcMotorEx holdingPenMotor;
    public DistanceSensor distanceSensor;

    public enum SSHOLDING_PEN_MOTOR_STATE {
        RUNNING,
        STOPPED,
        REVERSING
    }


    public SSHOLDING_PEN_MOTOR_STATE SSHoldingPenMotorState = SSHOLDING_PEN_MOTOR_STATE.STOPPED;

    public double SSHoldingPenMotorPower = 0.95;

    public SSHOLDING_PEN_MOTOR_STATE SSHoldingPenButtonState;

    public SSHoldingPen(HardwareMap hardwareMap, AbbyMode abbyMode){
        holdingPenMotor = hardwareMap.get(DcMotorEx.class, "holding_pen_motor");
        //touchSensor = hardwareMap.get(TouchSensor.class, "holding_pen_sensor");
        initSSHoldingPen();
    }

    public void initSSHoldingPen(){
        stopSSHoldingPenMotor();
    }

    public void startForwardSSHoldingPenMotor() {
        if (SSHoldingPenMotorState != SSHoldingPen.SSHOLDING_PEN_MOTOR_STATE.RUNNING) {
            runSSHoldingPenMotor(DcMotorEx.Direction.FORWARD, SSHoldingPenMotorPower);
            SSHoldingPenMotorState = SSHoldingPen.SSHOLDING_PEN_MOTOR_STATE.RUNNING;
        }
    }

    public void stopSSHoldingPenMotor() {
        if(SSHoldingPenMotorState != SSHoldingPen.SSHOLDING_PEN_MOTOR_STATE.STOPPED) {
            runSSHoldingPenMotor(DcMotor.Direction.FORWARD, 0.0);
            SSHoldingPenMotorState = SSHoldingPen.SSHOLDING_PEN_MOTOR_STATE.STOPPED;
        }
    }

    public void startReverseSSIntakeMotor() {
        if(SSHoldingPenMotorState != SSHoldingPen.SSHOLDING_PEN_MOTOR_STATE.REVERSING) {
            runSSHoldingPenMotor(DcMotor.Direction.REVERSE, SSHoldingPenMotorPower);
            SSHoldingPenMotorState = SSHoldingPen.SSHOLDING_PEN_MOTOR_STATE.REVERSING;

        }
    }

    public void runSSHoldingPenMotor(DcMotorEx.Direction direction, double power){
        holdingPenMotor.setDirection(direction);
        holdingPenMotor.setPower(power);
    }

    public SSHoldingPen.SSHOLDING_PEN_MOTOR_STATE getSSIntakeMotorState() {
        return SSHoldingPenMotorState;
    }

    public boolean isPixelCollected() {
        return distanceSensor.getDistance(DistanceUnit.CM) < 0.3;
    }


}
