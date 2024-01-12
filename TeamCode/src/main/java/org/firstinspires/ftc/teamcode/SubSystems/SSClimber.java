package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SSClimber {
    public DcMotorEx climberMotor;

    public final int CLIMBER_UP_POSITION_COUNT = 1000; // <-- temp value

    public final int CLIMBER_DOWN_POSITION_COUNT = 0;

    public enum CLIMBER_MOTOR_STATE {
        CLIMBER_UP_POSITION,
        CLIMBER_DOWN_POSITION
    }

    public CLIMBER_MOTOR_STATE climberMotorState = CLIMBER_MOTOR_STATE.CLIMBER_DOWN_POSITION;

    public Telemetry telemetry;

    public SSClimber(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        climberMotor = hardwareMap.get(DcMotorEx.class, "climber_motor");

        initClimber();
    }


    public void initClimber() {
        runClimberMotorDown();
    }

    public void runClimberMotorUp() {
        climberMotorState = CLIMBER_MOTOR_STATE.CLIMBER_UP_POSITION;
        climberMotor.setTargetPosition(CLIMBER_UP_POSITION_COUNT);
    }

    public void runClimberMotorDown() {
        climberMotorState = CLIMBER_MOTOR_STATE.CLIMBER_UP_POSITION;
        climberMotor.setTargetPosition(CLIMBER_DOWN_POSITION_COUNT);
    }


}
