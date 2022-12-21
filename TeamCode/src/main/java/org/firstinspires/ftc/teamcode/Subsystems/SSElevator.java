package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class SSElevator {

            public DcMotorEx elevatorMotorLeft;
            public DcMotorEx elevatorMotorRight;
            public TouchSensor touchSensor;
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
            
                // TODO : Amjad The motor used is a 1150 rpm motor
            //    https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-24mm-length-6mm-d-shaft-1150-rpm-36mm-gearbox-3-3-5v-encoder/
                
            public static double ENCODER_VALUE = 145.1; //TODO Amjad : The motor being used is 1150 Rpm motor with step of 145.1
            public static int baselineEncoderCount = 0; //TODO Amjad : Can remove as it is not used
            public static int ELEVATOR_LEVEL_LOW_POSITION_COUNT = 0;//  TODO : Determine by experimentation
            public static int ELEVATOR_LEVEL_HIGH_POSITION_COUNT = 850;//  TODO : Determine by experimentation
            //MAX 2200
            public static int ELEVATOR_DELTA_SLIGHTLY_UP_DELTA_COUNT = 50; //TODO Amjad : THere is no need for delta up here. you only have 2 position
            public static int ELEVATOR_DELTA_SLIGHTLY_DOWN_DELTA_COUNT = 50; //TODO Amjad : This is needed only to lower elevator to reset if needed.
            public static int ELEVATOR_LEVELMAX_POSITION_COUNT = 2200;

            public static double POWER_NO_CARGO = 0.75;
            public static double POWER_WITH_CARGO = 0.75;

            public ELEVATOR_POSITION elevatorPosition = ELEVATOR_POSITION.LEVEL_LOW;
            public ELEVATOR_POSITION previousPosition = ELEVATOR_POSITION.LEVEL_LOW;

            public int elevatorPositionCount = ELEVATOR_LEVEL_LOW_POSITION_COUNT;


            public SSElevator(HardwareMap hardwareMap) {
                elevatorMotorLeft = hardwareMap.get(DcMotorEx.class,"elevator_motor_left");
                elevatorMotorRight = hardwareMap.get(DcMotorEx.class,"elevator_motor_right");
                touchSensor = hardwareMap.get(TouchSensor.class, "elevator_sensor");
                initElevator();
            }

            /**
             * Initialization for the Elevator
             */
            public void initElevator(){
                elevatorMotorLeft.setPositionPIDFCoefficients(5.0);
                elevatorMotorLeft.setDirection(DcMotorEx.Direction.FORWARD);
                elevatorMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                elevatorMotorRight.setPositionPIDFCoefficients(5.0);
                elevatorMotorRight.setDirection(DcMotorEx.Direction.FORWARD);
                elevatorMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                resetElevator();
                moveElevatorLevelGround();
                if (getElevatorPosition() == elevatorPosition.LEVEL_LOW) {
                    turnElevatorBrakeModeOff();
                } else if (getElevatorPosition() == elevatorPosition.LEVEL_HIGH) {
                    turnElevatorBrakeModeOn();
                    //TODO Amjad : Brake should be off when the elevator is on low. Turn it on only when it is high.
                }
            }


            /**
             * Reset Elevator Encoder
             */
            public void resetElevator(){
                elevatorMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                elevatorMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            }

            /**
             * Method to set Elevator brake mode to ON when Zero (0.0) power is applied. <BR>
             * To be used when arm is above groundlevel
             * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
             */
            public void turnElevatorBrakeModeOn(){
                elevatorMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                elevatorMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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
            public double motorPowerToRun = POWER_NO_CARGO; //TODO Amjad : There is no way to determine CARGO here. Keep it only as POWER 

            /**
             * Method to run motor to set to the set position
             */
            public void runElevatorToLevel(double power){
                if (runElevatorToLevelState){
                    elevatorMotorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION); //TODO Amjad : Move inside if condition
                    elevatorMotorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION); //TODO Amjad : Move inside if condition
                    elevatorMotorLeft.setPower(power);
                    elevatorMotorRight.setPower(power);
                    runElevatorToLevelState = false;
                }
                else {
                    elevatorMotorRight.setPower(0.0);
                    elevatorMotorLeft.setPower(0.0);
                }

            }


            /**
             * Move Elevator to levelGround Position
             */
            public void moveElevatorLevelGround() {
                turnElevatorBrakeModeOff();
                elevatorPositionCount = ELEVATOR_LEVEL_LOW_POSITION_COUNT ;
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

                elevatorPositionCount = ELEVATOR_LEVEL_LOW_POSITION_COUNT; ////TODO Amjad : Remove baseline count
                elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
                elevatorMotorRight.setTargetPosition(elevatorPositionCount);
                motorPowerToRun = POWER_WITH_CARGO;
                runElevatorToLevelState = true;


                //check if elevator sensor is on or off
                //otherwise create while loop and move elevator slightly down until sensor tells it is all the way down
                /*if(!touchSensor.isPressed()){
                    while (!touchSensor.isPressed() ){
                        elevatorPositionCount = elevatorPositionCount - ELEVATOR_DELTA_SLIGHTLY_DOWN_DELTA_COUNT;
                        elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
                        elevatorMotorRight.setTargetPosition(elevatorPositionCount);
                        motorPowerToRun = POWER_WITH_CARGO;
                        runElevatorToLevelState = true;
                    }
                }*/

                elevatorPosition = ELEVATOR_POSITION.LEVEL_LOW;
            }

            /**
             * Move Elevator to levelHigh Position
             */
            public void moveElevatorLevelHigh() {
                turnElevatorBrakeModeOn();

                elevatorPositionCount = ELEVATOR_LEVEL_HIGH_POSITION_COUNT; //TODO Amjad : Remove baseline Encoder count
                //elevatorPositionCount_left = 
                elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
                elevatorMotorRight.setTargetPosition(elevatorPositionCount);
                motorPowerToRun = POWER_WITH_CARGO;
                runElevatorToLevelState = true;
                elevatorPosition = ELEVATOR_POSITION.LEVEL_HIGH;
            }

            /**
             * Move Elevator Slightly Down
             */
           public void moveSSElevatorSlightlyDown() { // TODO Amjad : This function is to be used only in the reset code, which stops when a touch sensor is pressed.
               turnElevatorBrakeModeOn();
               elevatorPositionCount = elevatorPositionCount - ELEVATOR_DELTA_SLIGHTLY_DOWN_DELTA_COUNT;
               elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
               elevatorMotorRight.setTargetPosition(elevatorPositionCount);
               motorPowerToRun = POWER_NO_CARGO;
               runElevatorToLevelState = true;
               // elevatorPositionCount = ELEVATOR_LEVEL_LOW_POSITION_COUNT ; //TODO Amjad : Why is this position being set?
               runElevatorToLevel(motorPowerToRun);
               resetElevator();

               //TODO : Add the code for runElevatorToLevel here, and once done call resetElevator().


           }


            /**
             * Move Elevator Slightly Up
             */
            public void moveSSElevatorSlightlyUp(){ // TODO Amjad : No need for this function.
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


