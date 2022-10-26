package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

        public class SSElevator {

            public DcMotorEx elevatorMotorLeft;
            public DcMotorEx elevatorMotorRight;
            //Gobilda (enter motor details later
            //Encoder count : enter details to be done


            public enum ELEVATOR_POSITION {
                LEVEL_LOW,
                LEVEL_HIGH,
            }


            // Encoder values for 5203 Gobilda 312rpm motor yyyy encoder counts / revolution
            //5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX Shaft, 312 RPM, 3.3 - 5V Encoder)
            //Encoder Resolution: 537.7 PPR at the Output Shaft
            //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
            public static double ENCODER_VALUE = 537.7;
            public static int baselineEncoderCount = 0;
            public static int ELEVATOR_LEVEL_LOW_POSITION_COUNT = 0;//  TODO : Determine by experimentation
            public static int ELEVATOR_LEVEL_HIGH_POSITION_COUNT = 500;//  TODO : Determine by experimentation
            //MAX 2200
            public static int ELEVATOR_DELTA_SLIGHTLY_UP_DELTA_COUNT = 40;
            public static int ELEVATOR_DELTA_SLIGHTLY_DOWN_DELTA_COUNT = 40;
            public static int ELEVATOR_LEVELMAX_POSITION_COUNT = 2200;

            public static double POWER_NO_CARGO = 0.5;
            public static double POWER_WITH_CARGO = 0.5;

            public ELEVATOR_POSITION elevatorPosition = ELEVATOR_POSITION.LEVEL_LOW;
            public ELEVATOR_POSITION previousPosition = ELEVATOR_POSITION.LEVEL_LOW;

            public int elevatorPositionCount = ELEVATOR_LEVEL_LOW_POSITION_COUNT;



            public SSElevator(HardwareMap hardwareMap) {
                elevatorMotorLeft = hardwareMap.get(DcMotorEx.class,"elevator_motor_left");
                elevatorMotorRight = hardwareMap.get(DcMotorEx.class,"elevator_motor_right");
                initElevator();
            }

            /**
             * Initialization for the Elevator
             */
            public void initElevator(){
                elevatorMotorLeft.setPositionPIDFCoefficients(5.0);
                elevatorMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevatorMotorRight.setPositionPIDFCoefficients(5.0);
                elevatorMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
                elevatorMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                resetElevator();
                moveElevatorLevelGround();
                turnElevatorBrakeModeOn();
            }

            /**
             * Reset Elevator Encoder
             */
            public void resetElevator(){
                elevatorMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevatorMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            /**
             * Method to set Elevator brake mode to ON when Zero (0.0) power is applied. <BR>
             * To be used when arm is above groundlevel
             * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
             */
            public void turnElevatorBrakeModeOn(){
                elevatorMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                elevatorMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            /**
             * Method to set Elevator brake mode to OFF when Zero (0.0) power is applied. <BR>
             * To be used when arm is on groundlevel or blockLevel[0]
             * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
             */
            public void turnElevatorBrakeModeOff(){
                elevatorMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                elevatorMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

            }

            public boolean runElevatorToLevelState = false;
            public double motorPowerToRun = POWER_NO_CARGO;

            /**
             * Method to run motor to set to the set position
             */
            public void runElevatorToLevel(double power){
                elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (runElevatorToLevelState || elevatorMotorLeft.isBusy() ){
                    elevatorMotorLeft.setPower(power);
                    runElevatorToLevelState = false;
                } else {
                    elevatorMotorLeft.setPower(0.0);
                }
                if (runElevatorToLevelState || elevatorMotorRight.isBusy() ){
                    elevatorMotorRight.setPower(power);
                    runElevatorToLevelState = false;
                } else {
                    elevatorMotorRight.setPower(0.0);
                }
            }


            /**
             * Move Elevator to levelGround Position
             */
            public void moveElevatorLevelGround() {
                turnElevatorBrakeModeOff();
                elevatorPositionCount = ELEVATOR_LEVEL_LOW_POSITION_COUNT + baselineEncoderCount;
                elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
                elevatorMotorRight.setTargetPosition(elevatorPositionCount);
                motorPowerToRun = POWER_NO_CARGO;
                runElevatorToLevelState = true;
                elevatorPosition = ELEVATOR_POSITION.LEVEL_LOW;
            }

            /**
             * Move Elevator to levelLow Position
             */
            public void moveElevatorLevelLow() {
                turnElevatorBrakeModeOn();

                elevatorPositionCount = ELEVATOR_LEVEL_LOW_POSITION_COUNT + baselineEncoderCount;
                elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
                elevatorMotorRight.setTargetPosition(elevatorPositionCount);
                motorPowerToRun = POWER_WITH_CARGO;
                runElevatorToLevelState = true;
                elevatorPosition = ELEVATOR_POSITION.LEVEL_LOW;
            }

            /**
             * Move Elevator to levelHigh Position
             */
            public void moveElevatorLevelHigh() {
                turnElevatorBrakeModeOn();

                elevatorPositionCount = ELEVATOR_LEVEL_HIGH_POSITION_COUNT + baselineEncoderCount;
                elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
                elevatorMotorRight.setTargetPosition(elevatorPositionCount);
                motorPowerToRun = POWER_WITH_CARGO;
                runElevatorToLevelState = true;
                elevatorPosition = ELEVATOR_POSITION.LEVEL_HIGH;
            }

            /**
             * Move Elevator Slightly Down
             */
            public void moveSSElevatorSlightlyDown() {
                if ((elevatorPositionCount > ELEVATOR_LEVEL_LOW_POSITION_COUNT + ELEVATOR_DELTA_SLIGHTLY_DOWN_DELTA_COUNT) &&
                        elevatorPositionCount <= ELEVATOR_LEVELMAX_POSITION_COUNT ) {
                    turnElevatorBrakeModeOn();
                    elevatorPositionCount = elevatorPositionCount - ELEVATOR_DELTA_SLIGHTLY_DOWN_DELTA_COUNT;
                    elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
                    elevatorMotorRight.setTargetPosition(elevatorPositionCount);
                    motorPowerToRun = POWER_NO_CARGO;
                    runElevatorToLevelState = true;
                }
            }

            /**
             * Move Elevator Slightly Up
             */
            public void moveSSElevatorSlightlyUp(){
                if ((elevatorPositionCount > ELEVATOR_LEVEL_LOW_POSITION_COUNT) &&
                        elevatorPositionCount <= ELEVATOR_LEVELMAX_POSITION_COUNT - ELEVATOR_DELTA_SLIGHTLY_UP_DELTA_COUNT){
                    turnElevatorBrakeModeOn();
                    elevatorPositionCount = elevatorPositionCount + ELEVATOR_DELTA_SLIGHTLY_UP_DELTA_COUNT;
                    elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
                    elevatorMotorRight.setTargetPosition(elevatorPositionCount);
                    motorPowerToRun = POWER_WITH_CARGO;
                    runElevatorToLevelState = true;
                }
            }

            /**
             * Return elevator state
             */
            public ELEVATOR_POSITION getElevatorPosition() {

                return elevatorPosition;
            }

            /**
             * Return elevator current position
             */
            public int currentLeftEncoderValue(){
                return elevatorMotorLeft.getCurrentPosition();
            }
            public int currentRightEncoderValue(){
                return elevatorMotorRight.getCurrentPosition();
            }

        }


