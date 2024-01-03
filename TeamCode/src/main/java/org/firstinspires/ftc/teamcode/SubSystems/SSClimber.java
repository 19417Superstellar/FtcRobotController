package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SSClimber {
    public DcMotorEx climberMotor;

    public enum CLIMBER_MOTOR_STATE {
        RUNNING,

        REVERSING,
        STOPPED;
    }
    public SSClimber.CLIMBER_MOTOR_STATE ssClimberMotorState = SSClimber.CLIMBER_MOTOR_STATE.STOPPED;

    public double ssClimberMotorPower = 0.0;//0.9;
    public Telemetry telemetry;

    public SSClimber(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        climberMotor = hardwareMap.get(DcMotorEx.class, "climber_motor");
        initSSClimber();
    }

    public void initSSClimber(){
        climberMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void startForwardClimberMotor() {
        if(ssClimberMotorState != SSClimber.CLIMBER_MOTOR_STATE.RUNNING) {
            runSSClimberMotor(DcMotor.Direction.RUNNING, ssClimberMotorPower);
            ssClimberMotorState = SSIntake.SSINTAKE_MOTOR_STATE.RUNNING;
        }
    }
}