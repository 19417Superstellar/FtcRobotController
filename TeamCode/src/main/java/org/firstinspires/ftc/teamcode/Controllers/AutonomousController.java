package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.Subsystems.SSElevator;
import org.firstinspires.ftc.teamcode.Subsystems.SSArm;
import org.firstinspires.ftc.teamcode.Subsystems.SSClaw;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;

public class AutonomousController {



    /**
     * Defenition of the AutoControl Class <BR>
     *
     * HzAutoControl consists of methods to control the robot subsystems in autonomous mode <BR>
     * This is set up as a state machine. And replicates all commands as in the gamepad <BR>
     *
     */

        //Create gamepad object reference to connect to gamepad1
        public DriveTrain driveTrain;
        public SSElevator ssElevator;
        public SSArm ssArm;
        public SSClaw ssClaw;

        public Pose2d startPose = GameField.ORIGINPOSE;

        /**
         * Constructor for HzGamepad1 class that extends gamepad.
         * Assign the gamepad1 given in OpMode to the gamepad used here.
         *
         */

        public AutonomousController(DriveTrain driveTrain,
                                    SSElevator ssElevator,
                                    SSClaw ssClaw,
                                    SSArm ssArm) {
            this.driveTrain = driveTrain;
            this.ssElevator = ssElevator;
            this.ssClaw = ssClaw;
            this.ssArm = ssArm;
        }


        /**
         * Main control function that calls the runs of each of the subsystems in sequence
         */
        public void runAutoControl(){
            int counter = 0;
            while (counter < 5) {
                runAutoSSElevator();
                runAutoSSClaw();
                runAutoSSArm();
                counter++;
            }
        }



/**
         * elevator class functions
         */
        enum AUTO_SSELEVATOR_STATE{
            LEVEL_LOW,
            LEVEL_HIGH,
        }
        AUTO_SSELEVATOR_STATE autoSSElevatorState = AUTO_SSELEVATOR_STATE.LEVEL_LOW;

        public void autoSSElevatorSetToLevelLow(){
            autoSSElevatorState = AUTO_SSELEVATOR_STATE.LEVEL_LOW;
            runAutoControl();
        }

        public void autoSSElevatorSetToLevelHigh(){
            autoSSElevatorState = AUTO_SSELEVATOR_STATE.LEVEL_HIGH;
            runAutoControl();
        }

        public void runAutoSSElevator() {
            switch (autoSSElevatorState) {
                case LEVEL_LOW:
                    ssElevator.moveElevatorLevelLow();
                    break;
                case LEVEL_HIGH:
                    ssElevator.moveElevatorLevelHigh();
                    break;
            }


            if (ssElevator.runElevatorToLevelState){
                ssElevator.runElevatorToLevel(ssElevator.motorPowerToRun);
            }
        }

        public AUTO_SSELEVATOR_STATE autoGetSSElevatorState() {
            return autoSSElevatorState;
        }

        /**** End of Elevator functions ***/


        /**
         * runAutoArm() function
         */

        enum AUTO_CLAW_POSITION {
        OPEN,
        CLOSED,
        }

        AUTO_CLAW_POSITION autoSSClawState = AUTO_CLAW_POSITION.CLOSED;

        public void autoSSClawSetToOpen(){
            autoSSClawState = AUTO_CLAW_POSITION.OPEN;
            runAutoControl();
        }

        public void autoSSClawSetToClose(){
            autoSSClawState = AUTO_CLAW_POSITION.CLOSED;
            runAutoControl();
        }


        enum AUTO_ARM_POSITION {
            ARM_POSITION_INTAKE_FORWARD,
            ARM_POSITION_INTAKE_REAR,
            ARM_POSITION_MID,
            ARM_POSITION_LOW,
            ARM_POSITION_HIGH,
            ARM_POSITION_GROUND,
        }

        AUTO_ARM_POSITION autoSSArmPosition = AUTO_ARM_POSITION.ARM_POSITION_INTAKE_FORWARD;

        public void autoSSArmSetToIntakeForward(){
            autoSSArmPosition = AUTO_ARM_POSITION.ARM_POSITION_INTAKE_FORWARD;
            runAutoControl();
        }

        public void autoSSArmSetToGround(){
            autoSSArmPosition = AUTO_ARM_POSITION.ARM_POSITION_GROUND;
            runAutoControl();
        }

        public void autoSSArmSetToIntakeRear(){
            autoSSArmPosition = AUTO_ARM_POSITION.ARM_POSITION_INTAKE_REAR;
            runAutoControl();
        }

        public void autoSSArmSetToMid(){
            autoSSArmPosition = AUTO_ARM_POSITION.ARM_POSITION_MID;
            runAutoControl();
        }

        public void autoSSArmSetToLow(){
            autoSSArmPosition = AUTO_ARM_POSITION.ARM_POSITION_LOW;
            runAutoControl();
        }

        public void autoSSArmSetToHigh(){
            autoSSArmPosition = AUTO_ARM_POSITION.ARM_POSITION_HIGH;
            runAutoControl();
        }

        public void runAutoSSClaw() {
            if(autoSSClawState == AUTO_CLAW_POSITION.CLOSED){
                ssClaw.setGripOpen();
            } else {
                ssClaw.setGripClose();
            }
        }

        public void runAutoSSArm() {
            if(autoSSArmPosition == AUTO_ARM_POSITION.ARM_POSITION_INTAKE_FORWARD){
                ssArm.moveArmIntakeForward();
            } else if (autoSSArmPosition == AUTO_ARM_POSITION.ARM_POSITION_INTAKE_REAR){
                ssArm.moveArmIntakeRear();
            } else if (autoSSArmPosition == AUTO_ARM_POSITION.ARM_POSITION_LOW){
                ssArm.moveArmLow();
            } else if (autoSSArmPosition == AUTO_ARM_POSITION.ARM_POSITION_MID){
                ssArm.moveArmMid();
            } else {
                ssArm.moveArmHigh();
            }

            if (ssArm.runArmToLevelState) {
                ssArm.runArmToLevel(ssArm.POWER_ARM_UP);
            }

        }

    }