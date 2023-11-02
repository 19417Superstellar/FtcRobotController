package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SSHoldingPen {

    public Servo holdingPenServo;
    public DistanceSensor clawSensor;
    public DistanceSensor penSensor;

    public enum SSHOLDING_PEN_SERVO_STATE {
        RUNNING,
        STOPPED,
        REVERSING
    }


    public SSHOLDING_PEN_SERVO_STATE SSHoldingPenServoState = SSHOLDING_PEN_SERVO_STATE.STOPPED;

    public double SSHoldingPenServoPower = 0.95;
    public Telemetry telemetry;
    public SSHoldingPen(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        holdingPenServo = hardwareMap.get(Servo.class, "holding_pen_servo");
        initSSHoldingPen();
    }

    public void initSSHoldingPen(){
        stopSSHoldingPenServo();
    }

    public void startForwardSSHoldingPenServo() {
        if (SSHoldingPenServoState != SSHoldingPen.SSHOLDING_PEN_SERVO_STATE.RUNNING) {
            runSSHoldingPenServo(Servo.Direction.FORWARD, SSHoldingPenServoPower);
            SSHoldingPenServoState = SSHoldingPen.SSHOLDING_PEN_SERVO_STATE.RUNNING;
        }
    }

    public void stopSSHoldingPenServo() {
        if(SSHoldingPenServoState != SSHoldingPen.SSHOLDING_PEN_SERVO_STATE.STOPPED) {
            runSSHoldingPenServo(Servo.Direction.FORWARD, 0.0);
            SSHoldingPenServoState = SSHoldingPen.SSHOLDING_PEN_SERVO_STATE.STOPPED;
        }
    }

    public void startReverseSSHoldingPenServo() {
        if(SSHoldingPenServoState != SSHoldingPen.SSHOLDING_PEN_SERVO_STATE.REVERSING) {
            runSSHoldingPenServo(Servo.Direction.REVERSE, SSHoldingPenServoPower);
            SSHoldingPenServoState = SSHoldingPen.SSHOLDING_PEN_SERVO_STATE.REVERSING;

        }
    }

    public void runSSHoldingPenServo(Servo.Direction direction, double power){
        holdingPenServo.setDirection(direction);
        holdingPenServo.setPosition(0.5);
    }

    public SSHoldingPen.SSHOLDING_PEN_SERVO_STATE getSSIntakeMotorState() {
        return SSHoldingPenServoState;
    }

    public boolean isClawPixelCollected() {
        return clawSensor.getDistance(DistanceUnit.CM) < 0.3;
    }

    public boolean isPenPixelCollected() {
        return penSensor.getDistance(DistanceUnit.CM) < 0.3;
    }

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("=============");
    }

}