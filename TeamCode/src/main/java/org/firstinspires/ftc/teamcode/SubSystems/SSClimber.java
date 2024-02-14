package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SSClimber {
    public DcMotorEx climberMotor;

    public final int CLIMBER_UP_POSITION_COUNT = 3000;// <-- temp value

    public final int CLIMBER_DOWN_POSITION_COUNT = 300;

    public enum CLIMBER_MOTOR_STATE {
        CLIMBER_UP_POSITION,
        CLIMBER_DOWN_POSITION
    }

    public CLIMBER_MOTOR_STATE climberMotorState = CLIMBER_MOTOR_STATE.CLIMBER_DOWN_POSITION;
    public int climberPositionCount = CLIMBER_DOWN_POSITION_COUNT;



    public Telemetry telemetry;

    public SSClimber(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        climberMotor = hardwareMap.get(DcMotorEx.class, "climber_motor");

        initClimber();
    }

    public void runClimberToLevel(double power, int position) {
        climberMotor.setTargetPosition(position);
        runMotors(power);
    }

    private void stopMotors() {
        climberMotor.setPower(0.0);
    }

    private void runMotors(double power) {
        climberMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        climberMotor.setPower(power);
    }

    public void moveClimberSlightlyDown() {
        climberPositionCount = climberPositionCount - 300;
        if ( climberPositionCount < 0 ) {
            climberPositionCount = 0;
        }
        climberMotor.setTargetPosition(climberPositionCount);
        /*if (runElevatorToLevelState) {
            runElevatorToLevel(motorPowerToRun);
        } */

        //resetElevator();
        //elevatorPositionCount = ELEVATOR_LEVEL_LOW_POSITION_COUNT;
    }

    public void moveClimberSlightlyUp() {
        climberPositionCount = climberPositionCount + 300;
        if ( climberPositionCount < 0 ) {
            climberPositionCount = 0;
        }
        climberMotor.setTargetPosition(climberPositionCount);
        /*if (runElevatorToLevelState) {
            runElevatorToLevel(motorPowerToRun);
        } */

        //resetElevator();
        //elevatorPositionCount = ELEVATOR_LEVEL_LOW_POSITION_COUNT;
    }

    public void initClimber() {
        climberMotor.setDirection(DcMotorEx.Direction.FORWARD);
        climberMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

       // runClimberMotorDown();
    }

    public void runClimberMotorUp() {
        climberMotorState = CLIMBER_MOTOR_STATE.CLIMBER_UP_POSITION;
        climberPositionCount = CLIMBER_UP_POSITION_COUNT;
        runClimberToLevel(0.95, CLIMBER_UP_POSITION_COUNT);
    }


    public void runClimberMotorDown() {
        climberMotorState = CLIMBER_MOTOR_STATE.CLIMBER_DOWN_POSITION;
        climberPositionCount = CLIMBER_DOWN_POSITION_COUNT;
        runClimberToLevel(0.8, CLIMBER_DOWN_POSITION_COUNT);

    }

    public void printDebugMessages(){
        //******  debug ******
        telemetry.addData("climber position", climberMotorState.toString());
    }
}
