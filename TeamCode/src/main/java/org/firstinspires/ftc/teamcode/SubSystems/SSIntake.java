package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Definition of SSIntake Class <BR>
 *
 * HzIntake consists of system provided intake controls and adds functionality to the selection made on intake. <BR>
 *
 * The states are as followed: <BR>
 *     <emsp>INTAKE_MOTOR_STATE for telling if intake is running, stopped, or reversing </emsp> <BR>
 *     <emsp>INTAKE_REVERSE_BUTTON_STATE sees if the intake is on or off </emsp> <BR>
 *
 * The functions are as followed: <BR>
 *     <emsp>runIntakeMotor checks if the intake is not running and runs the intake </emsp> <BR>
 *     <emsp>stopIntakeMotor checks if the intake has stopped and if its not, it sets the intake power to 0
 *     and sets intakeMotorState to INTAKE_MOTOR_STATE.STOPPED </emsp> <BR>
 *     <emsp>reverseIntakeMotor checks if the intake is not reversing, and sets the intake motor to FORWARD, then also
 *     sets intake motor state to REVERSING</emsp> <BR>
 */
public class SSIntake {

    public DcMotorEx intakeMotor;
    public Servo intakeLatch;
    public NormalizedColorSensor topSensor;
    public NormalizedColorSensor bottomSensor;

    public enum SSINTAKE_MOTOR_STATE {
        RUNNING,
        STOPPED,
        REVERSING
    }

    public SSINTAKE_MOTOR_STATE ssIntakeMotorState = SSINTAKE_MOTOR_STATE.STOPPED;

    public double ssIntakeMotorPower = 0.95;//0.9;
    public Telemetry telemetry;

    public SSIntake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        topSensor = hardwareMap.get(NormalizedColorSensor.class, "topSensor");
        bottomSensor = hardwareMap.get(NormalizedColorSensor.class, "bottomSensor");
        intakeLatch = hardwareMap.get(Servo.class, "intake_latch");

        initSSIntake();
    }

    public void initSSIntake(){
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeLatch.setPosition(0.5);
    }

    /**
     * runIntakeMotor checks if the intake is not running and runs the intake
     */
    public void startForwardSSIntakeMotor() {
        if(ssIntakeMotorState != SSINTAKE_MOTOR_STATE.RUNNING) {
            runSSIntakeMotor(DcMotor.Direction.FORWARD, ssIntakeMotorPower);
            ssIntakeMotorState = SSINTAKE_MOTOR_STATE.RUNNING;
        }
    }

    /**
     * stopIntakeMotor checks if the intake has stopped and if its not, it sets the intake power to 0
     * and sets intakeMotorState to INTAKE_MOTOR_STATE.STOPPED
     */
    public void stopSSIntakeMotor() {
        if(ssIntakeMotorState != SSINTAKE_MOTOR_STATE.STOPPED) {
            runSSIntakeMotor(DcMotor.Direction.FORWARD, 0.0);
            ssIntakeMotorState = SSINTAKE_MOTOR_STATE.STOPPED;
        }
    }

    /**
     * reverseIntakeMotor checks if the intake is not reversing, and sets the intake motor to FORWARD, then also
     * ets intake motor state to REVERSING
     */
    public void startReverseSSIntakeMotor() {
        if(ssIntakeMotorState != SSINTAKE_MOTOR_STATE.REVERSING) {
            runSSIntakeMotor(DcMotor.Direction.REVERSE, ssIntakeMotorPower);
            ssIntakeMotorState = SSINTAKE_MOTOR_STATE.REVERSING;
        }
    }

    /**
     * Run SS Intake motor
     */
    public void runSSIntakeMotor(DcMotor.Direction direction, double power){
        intakeMotor.setDirection(direction);
        intakeMotor.setPower(power);
    }

    public boolean bottomSensorDetectsIntake() {
        if(((DistanceSensor) bottomSensor).getDistance(DistanceUnit.MM) > 40 && ((DistanceSensor) bottomSensor).getDistance(DistanceUnit.MM) < 60) {
            return true;
        }
        return false;
    }

    public boolean topSensorDetectsIntake() {
        if(((DistanceSensor) topSensor).getDistance(DistanceUnit.MM) > 40 && ((DistanceSensor) topSensor).getDistance(DistanceUnit.MM) < 60) {
            return true;
        }
        return false;
    }

    /**
     * Returns Intake motor state
     */
    public SSINTAKE_MOTOR_STATE getSsIntakeMotorState() {
        return ssIntakeMotorState;
    }

    public void printDebugMessages() {
        //******  debug ******
        telemetry.addData("Intake Motor State:", this.getSsIntakeMotorState());
        telemetry.addLine("Bottom intake distance: " + ((DistanceSensor) bottomSensor).getDistance(DistanceUnit.MM));
        telemetry.addLine("Bottom intake detected: " + bottomSensorDetectsIntake());

        telemetry.addLine("Top intake distance: " + ((DistanceSensor) topSensor).getDistance(DistanceUnit.MM));
        telemetry.addLine("Top intake detected: " + topSensorDetectsIntake());
    }

}