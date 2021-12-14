package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.SSArm;
import org.firstinspires.ftc.teamcode.SubSystems.SSBucket;
import org.firstinspires.ftc.teamcode.SubSystems.SSElevator;
import org.firstinspires.ftc.teamcode.SubSystems.SSIntake;
import org.firstinspires.ftc.teamcode.SubSystems.SSSpinner;

/**
 * Defenition of the HzGamepad Class <BR>
 *
 * HzGamepad consists of system provided gamepad(s) and adds functionality to the selection 
 * made on gamepads <BR>
 * 
 * For Hazmat Skystone, only one Gamepad is used (gamepad1) <BR>
 *
 * The controls are as follows: (replace with gamepad2 for 2nd gamepad) <BR>
 *      <emsp>Left Stick for pan motion (gamepad1.left_stick_x and gamepad1.left_stick_y) <BR>
 *      <emsp>Right Stick for turn motion (only uses the x direction : gamepad1.right_stick_y) <BR>
 *      <emsp>Right Bumper for Launching Ring (gamepad1.right_bumper) <BR>
 *      <emsp>Left Bumper for Grip Arm Servos (gamepad1.left_bumper) <BR>
 *      <emsp>Right Trigger for Accelerating robot (gamepad1.right_trigger) <BR>
 *      <emsp>Button A to Powershot selection (gamepad1.a) <BR>
 *      <emsp>Button Y to High Goal selection (gamepad1.y) <BR>
 *      <emsp>Button X to Turn delta left (gamepad1.x) <BR>
 *      <emsp>Button B to Turn delta right (gamepad1.b) <BR>
 *      <emsp>Button Dpad_up to Reverse Intake on (gamepad1.dpad_up) <BR>
 *      <emsp>Button Dpad_down to Intake On (gamepad1.dpad_down) <BR>
 *
 * To access the gamepad functions, use the gp1Get* or gp2Get* functions at the end of this class <BR>
 *     <emsp>gp1GetLeftStickX(), gp2GetLeftStickX()</emsp>
 *     <emsp>gp1GetLeftStickY(), gp2GetLeftStickY()</emsp>
 *     <emsp>gp1GetRightStickX(), gp2GetRightStickX()</emsp>
 *     <emsp>gp1GetRightStickY(), gp2GetRightStickY()</emsp>
 *     <emsp>gp1GetLeftTrigger(), gp2GetRightTrigger()</emsp>
 *     <emsp>gp1GetLeftTriggerPress(), gp2GetRightTriggerPress for toggle value()</emsp>
 *     <emsp>gp1GetLeftBumper(), gp2GetRightBumper()</emsp>
 *     <emsp>gp1GetLeftBumperPress(), gp2GetRightBumperPress for toggle value()</emsp>
 *     <emsp>gp1GetX(), gp2GetY(), gp1GetA(), gp2GetB()</emsp>
 *     <emsp>gp1GetXPress(), gp2GetYPress(), gp1GetAPress(), gp2GetBPress() for toggle value()</emsp>
 *     <emsp>gp1GetDpad_up(), gp2GetDpad_down(). gp1GetDpad_left(), gp2GetDpad_right() </emsp>
 *     <emsp>gp1GetDpad_upPress(), gp2GetDpad_downPress(). gp1GetDpad_leftPress(), gp2GetDpad_rightPress()  for toggle value()</emsp>
 *     <emsp>gp1GetStart(), gp2GetStart()</emsp>
 *
 */

public class GamepadController {

    //Create object reference to objects to systems passed from TeleOp
    public Gamepad  ssGamepad1,  ssGamepad2;
    public DriveTrain driveTrain;
    public SSIntake ssIntake;
    public SSElevator ssElevator;
    public SSBucket ssBucket;
    public SSSpinner ssSpinner;
    public SSArm ssArm;

    /**
     * Constructor for  ssGamepad1 class that extends gamepad.
     * Assign the gamepad1 given in OpMode to the gamepad used here.
     */
    public GamepadController(Gamepad  ssGamepad1,
                             Gamepad  ssGamepad2,
                             DriveTrain driveTrain,
                             SSIntake ssIntake,
                             SSElevator ssElevator,
                             SSBucket ssBucket,
                             SSSpinner ssSpinner,
                             SSArm ssArm    ) {
        this. ssGamepad1 =  ssGamepad1;
        this. ssGamepad2 =  ssGamepad2;
        this.driveTrain = driveTrain;
        this.ssIntake = ssIntake;
        this.ssElevator = ssElevator;
        this.ssBucket = ssBucket;
        this.ssSpinner = ssSpinner;
        this.ssArm = ssArm;
    }

    /**
     *runByGamepad is the main controller function that runs each subsystem controller based on states
     */
    public void runByGamepadControl(){
        runIntake();
        runElevator();
        runBucket();
        runSpinner();
        runArm();
        runDriveControl_byRRDriveModes();
    }

    /**
     * runByGamepadRRDriveModes sets modes for Road Runner such as ROBOT and FIELD Centric Modes. <BR>
     *     RR Drive Train
     */
    public void runDriveControl_byRRDriveModes() {

        driveTrain.poseEstimate = driveTrain.getPoseEstimate();

        driveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;

        if (driveTrain.driveType == DriveTrain.DriveType.ROBOT_CENTRIC){
            if (ssElevator.getElevatorPosition() != SSElevator.ELEVATOR_POSITION.LEVEL_0) {
                driveTrain.gamepadInput = new Vector2d(
                        -gp1TurboMode(gp1GetLeftStickY()),
                        -gp1TurboMode(gp1GetLeftStickX()));
            } else {
                // Avoid using Turbo when elevator is at Level 0
                driveTrain.gamepadInput = new Vector2d(
                        -limitStick(gp1GetLeftStickY()),
                        -limitStick(gp1GetLeftStickX()));
            }
        }

        if (driveTrain.driveType == DriveTrain.DriveType.FIELD_CENTRIC){

            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE) { // Red Alliance
                driveTrain.gamepadInput = new Vector2d(
                        gp1TurboMode(gp1GetLeftStickX()),
                        -gp1TurboMode(gp1GetLeftStickY())
                ).rotated(-driveTrain.poseEstimate.getHeading());
            }

            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) { // Blue Alliance
                driveTrain.gamepadInput = new Vector2d(
                        -gp1TurboMode(gp1GetLeftStickX()),
                        gp1TurboMode(gp1GetLeftStickY())
                ).rotated(-driveTrain.poseEstimate.getHeading());
            }
        }
        driveTrain.gamepadInputTurn = -gp1TurboMode(gp1GetRightStickX());

        driveTrain.driveTrainPointFieldModes();
    }



    /**
     * function for intake function on Dpad
     * Dpad_UP is spin out the cargo
     * Dpad_Down is spin in the cargo
     */
    public void runIntake() {
        if (gp2GetDpad_upPress()) {

            //if intake motor is not running and if elevator leve is zero
            //if bucket is not in collect set bucket to collect
            //and start intake motor forward
            if((ssIntake.getSSIntakeMotorState() != SSIntake.SSINTAKE_MOTOR_STATE.RUNNING) &&
                    (ssElevator.getElevatorPosition()==SSElevator.ELEVATOR_POSITION.LEVEL_0)) {
                if(ssBucket.getBucketServoState() != SSBucket.BUCKET_SERVO_STATE.COLLECT_POSITION){
                    ssBucket.setToCollect();
                }
                ssIntake.startForwardSSIntakeMotor();
            } else if(ssIntake.getSSIntakeMotorState() != SSIntake.SSINTAKE_MOTOR_STATE.STOPPED) {
                ssIntake.stopSSIntakeMotor();
            }
        }

        //Reverse Intake motors and run - in case of stuck state)
        if (gp2GetDpad_downPress()) {
            if(ssIntake.getSSIntakeMotorState() != SSIntake.SSINTAKE_MOTOR_STATE.REVERSING) {
                ssIntake.startReverseSSIntakeMotor();
            } else if (ssIntake.getSSIntakeMotorState() != SSIntake.SSINTAKE_MOTOR_STATE.STOPPED) {
                ssIntake.stopSSIntakeMotor();
            }
        }
    } //runIntake() ends here

    /**
     * Elevator function mapped to the gamepad buttons
     */
    public void runElevator() {
        if (gp2GetButtonAPress()) {
            if (ssElevator.getElevatorPosition() != SSElevator.ELEVATOR_POSITION.LEVEL_0) {
                ssElevator.moveElevatorLevel0();
            }
            if (ssElevator.runElevatorToLevelState) {
                ssElevator.runElevatorToLevel(ssElevator.motorPowerToRun);
            }
            if (ssBucket.getBucketServoState() != SSBucket.BUCKET_SERVO_STATE.COLLECT_POSITION) {
                ssBucket.setToCollect();
            }
        }

        if (gp2GetButtonXPress()) {
            if (ssIntake.getSSIntakeMotorState() != SSIntake.SSINTAKE_MOTOR_STATE.STOPPED) {
                ssIntake.stopSSIntakeMotor();
            }
            if (ssBucket.getBucketServoState() != SSBucket.BUCKET_SERVO_STATE.TRANSPORT_POSITION) {
                ssBucket.setToTransport();
            }
            if (ssElevator.getElevatorPosition() != SSElevator.ELEVATOR_POSITION.LEVEL_1) {
                ssElevator.moveElevatorLevel1();

            }
        }

        if (gp2GetButtonBPress()) {
            if (ssIntake.getSSIntakeMotorState() != SSIntake.SSINTAKE_MOTOR_STATE.STOPPED) {
                ssIntake.stopSSIntakeMotor();
            }
            if (ssBucket.getBucketServoState() != SSBucket.BUCKET_SERVO_STATE.TRANSPORT_POSITION) {
                ssBucket.setToTransport();
            }
            if (ssElevator.getElevatorPosition() != SSElevator.ELEVATOR_POSITION.LEVEL_2) {
                ssElevator.moveElevatorLevel2();
            }
        }

        if (gp2GetButtonYPress()) {
            if (ssIntake.getSSIntakeMotorState() != SSIntake.SSINTAKE_MOTOR_STATE.STOPPED) {
                ssIntake.stopSSIntakeMotor();
            }
            if (ssBucket.getBucketServoState() != SSBucket.BUCKET_SERVO_STATE.TRANSPORT_POSITION) {
                ssBucket.setToTransport();
            }
            if (ssElevator.getElevatorPosition() != SSElevator.ELEVATOR_POSITION.LEVEL_3) {
                ssElevator.moveElevatorLevel3();
            }
        }
        /*start+left trigger in gamepad1 will move slightly down
          left trigger in gamepad1 will move slightly up
         */

        if (!gp1GetStart()) {
            if (gp1GetLeftTriggerPress()) {
                ssElevator.moveSSElevatorSlightlyDown();
            }
        } else {
            if (gp1GetLeftTriggerPress()) {
                ssElevator.moveSSElevatorSlightlyUp();
            }
        }

        if (ssElevator.runElevatorToLevelState) {
            ssElevator.runElevatorToLevel(ssElevator.motorPowerToRun);
        }
    } // End of runElevator function

    public enum AUTO_BUCKET {
        ON,
        OFF
    }
    public AUTO_BUCKET autoBucket = AUTO_BUCKET.ON;

    /**
     * run function for bucket Gamepad 2 Right bumper
     */
    public void runBucket() {
        if (gp2GetRightBumperPress()) {
            //To toggle automation to off or on if needed
            if (gp2GetStart()) {
                if (autoBucket == AUTO_BUCKET.ON) {
                    autoBucket = AUTO_BUCKET.OFF;
                } else {
                    autoBucket = AUTO_BUCKET.ON;
                }
            }

            /*
             * for level0, if right bumper is pressed,
             * change the bucket from transport to collect if bucket state is transport
             * change from collect to collect to transport if bucket state is collect
             *
             * for level1, level2 and level3,if right bumper is pressed,
             * change the state of bucket from transport to drop if bucket is in transport
             * change the bucket from drop to transport if the bucket is in drop
             *
             */
            if (ssElevator.getElevatorPosition() == SSElevator.ELEVATOR_POSITION.LEVEL_0) {
                if (ssBucket.getBucketServoState() == SSBucket.BUCKET_SERVO_STATE.TRANSPORT_POSITION) {
                    ssBucket.setToCollect();
                } else if (ssBucket.getBucketServoState() == SSBucket.BUCKET_SERVO_STATE.COLLECT_POSITION) {
                    //ssIntake.stopSSIntakeMotor();
                    ssBucket.setToTransport();
                }
            } else if (ssElevator.getElevatorPosition() == SSElevator.ELEVATOR_POSITION.LEVEL_1 ||
                    ssElevator.getElevatorPosition() == SSElevator.ELEVATOR_POSITION.LEVEL_2 ||
                    ssElevator.getElevatorPosition() == SSElevator.ELEVATOR_POSITION.LEVEL_3) {

                if (ssBucket.getBucketServoState() != SSBucket.BUCKET_SERVO_STATE.DROP_POSITION) {
                    ssBucket.setToDrop();

                } else if (ssBucket.getBucketServoState() == SSBucket.BUCKET_SERVO_STATE.DROP_POSITION) {
                    ssBucket.setToTransport();
                }
            }
        }

        if (ssBucket.bucketColorSensor instanceof SwitchableLight) {
            if (ssElevator.getElevatorPosition() == SSElevator.ELEVATOR_POSITION.LEVEL_0 &&
                 ssBucket.getBucketColorSensorState() == SSBucket.BUCKET_COLOR_SENSOR_STATE.EMPTY) {
                ((SwitchableLight) ssBucket.bucketColorSensor).enableLight(true);
            } else {
                ((SwitchableLight) ssBucket.bucketColorSensor).enableLight(false);
                }
            }

        if (autoBucket == AUTO_BUCKET.ON) {
            if (ssElevator.getElevatorPosition() == SSElevator.ELEVATOR_POSITION.LEVEL_0) {
                if (ssBucket.getBucketColorSensorState() == SSBucket.BUCKET_COLOR_SENSOR_STATE.LOADED) {
                    if (ssBucket.getBucketServoState() != SSBucket.BUCKET_SERVO_STATE.TRANSPORT_POSITION) {
                        ssBucket.setToTransport();
                        ssElevator.moveElevatorLevel1();
                        ssIntake.stopSSIntakeMotor();
                    }
                }
            }
        }
    } // end of runBucket function

    /**
     * Start of runSpinner
     */
    public void runSpinner() {
        //spinner to spin forward

        if (!gp2GetStart()) {
                if (gp2GetLeftBumperPress()) {
                    if ((ssSpinner.getSSSpinnerMotorState() == SSSpinner.SSSPINNER_MOTOR_STATE.CLOCKWISE) ||
                            (ssSpinner.getSSSpinnerMotorState() == SSSpinner.SSSPINNER_MOTOR_STATE.ANTICLOCKWISE)) {
                        ssSpinner.stopSSSpinnerMotor();
                    } else {
                        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                            if (ssSpinner.getSSSpinnerMotorState() != SSSpinner.SSSPINNER_MOTOR_STATE.CLOCKWISE) {
                                ssSpinner.startClockwiseSSSPinnerMotor();
                            } else {
                                if (ssSpinner.getSSSpinnerMotorState() != SSSpinner.SSSPINNER_MOTOR_STATE.ANTICLOCKWISE) {
                                    ssSpinner.startAntiClockwiseSSSpinnerMotor();
                                }
                            }
                        }
                    }
                }
        }else {
            if (gp2GetLeftBumperPress()) {
                if (ssSpinner.getSSSpinnerMotorState() == SSSpinner.SSSPINNER_MOTOR_STATE.CLOCKWISE ||
                        ssSpinner.getSSSpinnerMotorState() == SSSpinner.SSSPINNER_MOTOR_STATE.ANTICLOCKWISE) {
                    ssSpinner.stopSSSpinnerMotor();
                } else {
                    if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                        if (ssSpinner.getSSSpinnerMotorState() != SSSpinner.SSSPINNER_MOTOR_STATE.ANTICLOCKWISE) {
                            ssSpinner.startAntiClockwiseSSSpinnerMotor();
                        } else {
                            if (ssSpinner.getSSSpinnerMotorState() != SSSpinner.SSSPINNER_MOTOR_STATE.CLOCKWISE) {
                                ssSpinner.startClockwiseSSSPinnerMotor();
                            }
                        }
                    }
                }
            }
        }
    }//end of runspinner




    /**
     * Start of runArm function
     * */
    public void runArm() {
        if(gp1GetButtonXPress()) {
            ssArm.moveArmPickup();
        }
        if(gp1GetButtonYPress()) {
            ssArm.moveArmDrop();
        }
        if(gp1GetButtonAPress()) {
            ssArm.moveArmParked();
        }
        if(gp1GetButtonBPress()) {
            ssArm.dropBelowCapstone();
        }

        if (ssArm.runArmToLevelState) {
            ssArm.runArmToLevel(ssArm.POWER_ARM_UP);
        }

        if(gp1GetRightBumperPress()) {
            if(ssArm.getGripServoState() != SSArm.GRIP_SERVO_STATE.GRIP_CLOSE) {
                ssArm.setGripClose();
            } else if (ssArm.armPosition != SSArm.ARM_POSITION.ARM_PARKED){
                ssArm.setGripOpen();
            }

        }
    } //End of runArm function


    //*********** KEY PAD MODIFIERS BELOW ***********

    //**** Gamepad buttons
    //Records last button press to deal with single button presses doing a certain methods
    boolean gp1ButtonALast = false;
    boolean gp1ButtonBLast = false;
    boolean gp1ButtonXLast = false;
    boolean gp1ButtonYLast = false;
    boolean gp1RightBumperLast = false;
    boolean gp1LeftBumperLast = false;
    boolean gp1Dpad_upLast = false;
    boolean gp1Dpad_downLast = false;
    boolean gp1Dpad_leftLast = false;
    boolean gp1Dpad_rightLast = false;
    boolean gp1LeftTriggerLast = false;
    boolean gp1RightTriggerLast = false;

    boolean gp2ButtonALast = false;
    boolean gp2ButtonBLast = false;
    boolean gp2ButtonXLast = false;
    boolean gp2ButtonYLast = false;
    boolean gp2RightBumperLast = false;
    boolean gp2LeftBumperLast = false;
    boolean gp2Dpad_upLast = false;
    boolean gp2Dpad_downLast = false;
    boolean gp2Dpad_leftLast = false;
    boolean gp2Dpad_rightLast = false;
    boolean gp2LeftTriggerLast = false;
    boolean gp2RightTriggerLast = false;

    /**
     * Method to convert linear map from gamepad1 stick input to a cubic map
     *
     * @param stickInput input value of button stick vector
     * @return Cube of the stick input reduced to 25% speed
     */
    public double limitStick(double stickInput) {
        return (stickInput * stickInput * stickInput * 0.33);
    }

    /**
     * Method to implement turbo speed mode - from reduced speed of 25% of cubic factor to
     * 100% speed, but controlled by acceleration of the force of pressing the Right Tigger.
     *
     * @param stickInput input value of button stick vector
     * @return modified value of button stick vector
     */
    public double gp1TurboMode(double stickInput) {

        double acceleration_factor;
        double rightTriggerValue;

        double turboFactor;

        rightTriggerValue = gp1GetRightTrigger();
        //acceleration_factor = 1.0 + 3.0 * rightTriggerValue;
        acceleration_factor = 1.0 + 2.0 * rightTriggerValue;
        turboFactor = limitStick(stickInput) * acceleration_factor;
        return turboFactor;
    }

    /**
     * Methods to get the value of gamepad Left stick X for Pan motion X direction.
     * This is the method to apply any directional modifiers to match to the X plane of robot.
     * No modifier needed for Hazmat Skystone Robot.
     *
     * @return gpGamepad1.left_stick_x
     */
    public double gp1GetLeftStickX() {
        return  ssGamepad1.left_stick_x;
    }
    public double gp2GetLeftStickX() {
        return  ssGamepad2.left_stick_x;
    }

    /**
     * Methods to get the value of gamepad Left stick Y for Pan motion Y direction.
     * This is the method to apply any directional modifiers to match to the Y plane of robot.
     * For Hazmat Skystone Robot, Y direction needs to be inverted.
     *
     * @return gpGamepad1.left_stick_y
     */
    public double gp1GetLeftStickY() { return  ssGamepad1.left_stick_y; }
    public double gp2GetLeftStickY() { return  ssGamepad2.left_stick_y; }

    /**
     * Methods to get the value of gamepad Right stick X to keep turning.
     * This is the method to apply any directional modifiers to match to the turn direction robot.
     * No modifier needed for Hazmat Skystone Robot.
     *
     * @return gpGamepad1.right_stick_x
     */
    public double gp1GetRightStickX() {
        return  ssGamepad1.right_stick_x;
    }
    public double gp2GetRightStickX() {
        return  ssGamepad2.right_stick_x;
    }
    public double gp1GetRightStickY() {
        return  ssGamepad1.right_stick_y;
    }
    public double gp2GetRightStickY() {
        return  ssGamepad2.right_stick_y;
    }

    /**
     * Methods to get the value of gamepad Right Trigger for turbo mode (max speed).
     * This is the method to apply any modifiers to match to action of turbo mode for each driver preference.
     * For Hazmat Skystone Right Trigger pressed means turbo mode on.
     *
     * @return gpGamepad1.right_trigger
     */
    public double gp1GetRightTrigger() {
        return  ssGamepad1.right_trigger;
    }
    public double gp2GetRightTrigger() {
        return  ssGamepad2.right_trigger;
    }

    public boolean gp1GetRightTriggerPress() {
        boolean isPressedRightTrigger = false;
        if (!gp1RightTriggerLast && (gp1GetRightTrigger()>0.7)) {
            isPressedRightTrigger = true;
        }
        gp1RightTriggerLast = (gp1GetRightTrigger()>0.7);
        return isPressedRightTrigger;
    }

    public boolean gp2GetRightTriggerPress() {
        boolean isPressedRightTrigger = false;
        if (!gp2RightTriggerLast && (gp2GetRightTrigger()>0.7)) {
            isPressedRightTrigger = true;
        }
        gp2RightTriggerLast = (gp2GetRightTrigger()>0.7);
        return isPressedRightTrigger;
    }

    /**
     * Methods to get the value of gamepad Left Trigger
     *
     * @return gpGamepad1.right_trigger
     */
    public double gp1GetLeftTrigger() {
        return  ssGamepad1.left_trigger;
    }
    public double gp2GetLeftTrigger() {
        return  ssGamepad2.left_trigger;
    }

    public boolean gp1GetLeftTriggerPress() {
        boolean isPressedLeftTrigger = false;
        if (!gp1LeftTriggerLast && (gp1GetLeftTrigger()>0.7)) {
            isPressedLeftTrigger = true;
        }
        gp1LeftTriggerLast = (gp1GetLeftTrigger()>0.7);
        return isPressedLeftTrigger;
    }

    public boolean gp2GetLeftTriggerPress() {
        boolean isPressedLeftTrigger = false;
        if (!gp2LeftTriggerLast && (gp2GetLeftTrigger()>0.7)) {
            isPressedLeftTrigger = true;
        }
        gp2LeftTriggerLast = (gp2GetLeftTrigger()>0.7);
        return isPressedLeftTrigger;
    }

    /**
     * Methods to get the value of gamepad Left Bumper
     *
     * @return gpGamepad1.left_bumper
     */
    public boolean gp1GetLeftBumper() {
        return  ssGamepad1.left_bumper;
    }
    public boolean gp2GetLeftBumper() {
        return  ssGamepad2.left_bumper;
    }

    /**
     * Method to track if Left Bumper was pressed
     * To ensure that the continuous holding of the left bumper does not cause a contiual action,
     * the state of the bumper is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing hold or release of button should not trigger action.
     *
     * @return isPressedLeftBumper| = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetLeftBumperPress() {
        boolean isPressedLeftBumper = false;
        if (!gp1LeftBumperLast &&  ssGamepad1.left_bumper) {
            isPressedLeftBumper = true;
        }
        gp1LeftBumperLast =  ssGamepad1.left_bumper;
        return isPressedLeftBumper;
    }

    public boolean gp2GetLeftBumperPress() {
        boolean isPressedLeftBumper = false;
        if (!gp2LeftBumperLast &&  ssGamepad2.left_bumper) {
            isPressedLeftBumper = true;
        }
        gp2LeftBumperLast =  ssGamepad2.left_bumper;
        return isPressedLeftBumper;
    }

    /**
     * Methods to get the value of gamepad Right Bumper
     *
     * @return gpGamepad1.right_bumper
     */
    public boolean gp1GetRightBumper() {
        return  ssGamepad1.right_bumper;
    }
    public boolean gp2GetRightBumper() {
        return  ssGamepad2.right_bumper;
    }
    /**
     * Method to track if Right Bumper was pressed
     * To ensure that the continuous holding of the right bumper does not cause a continual action,
     * the state of the bumper is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedRightBumper = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetRightBumperPress() {
        boolean isPressedRightBumper = false;
        if (!gp1RightBumperLast &&  ssGamepad1.right_bumper) {
            isPressedRightBumper = true;
        }
        gp1RightBumperLast =  ssGamepad1.right_bumper;
        return isPressedRightBumper;
    }

    public boolean gp2GetRightBumperPress() {
        boolean isPressedRightBumper = false;
        if (!gp2RightBumperLast &&  ssGamepad2.right_bumper) {
            isPressedRightBumper = true;
        }
        gp2RightBumperLast =  ssGamepad2.right_bumper;
        return isPressedRightBumper;
    }

    public boolean gp1GetRightBumperPersistant(){
        return  ssGamepad1.right_bumper;
    }
    public boolean gp2GetRightBumperPersistant(){
        return  ssGamepad2.right_bumper;
    }

    /**
     * Method to track if Button A was pressed
     * To ensure that the continuous holding of Button A does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButton A = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetButtonAPress() {
        boolean isPressedButtonA = false;
        if (!gp1ButtonALast &&  ssGamepad1.a) {
            isPressedButtonA = true;
        }
        gp1ButtonALast =  ssGamepad1.a;
        return isPressedButtonA;
    }
    public boolean gp2GetButtonAPress() {
        boolean isPressedButtonA = false;
        if (!gp2ButtonALast &&  ssGamepad2.a) {
            isPressedButtonA = true;
        }
        gp2ButtonALast =  ssGamepad2.a;
        return isPressedButtonA;
    }
    public boolean gp1GetA(){
        return  ssGamepad1.a;
    }
    public boolean gp2GetA(){
        return  ssGamepad2.a;
    }


    /**
     * Method to track if Button Y was pressed
     * To ensure that the continuous holding of Button Y does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButtonY = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetButtonYPress() {
        boolean isPressedButtonY = false;
        if (!gp1ButtonYLast &&  ssGamepad1.y) {
            isPressedButtonY = true;
        }
        gp1ButtonYLast =  ssGamepad1.y;
        return isPressedButtonY;
    }
    public boolean gp2GetButtonYPress() {
        boolean isPressedButtonY = false;
        if (!gp2ButtonYLast &&  ssGamepad2.y) {
            isPressedButtonY = true;
        }
        gp2ButtonYLast =  ssGamepad2.y;
        return isPressedButtonY;
    }
    public boolean gp1GetY(){
        return  ssGamepad1.y;
    }
    public boolean gp2GetY(){
        return  ssGamepad2.y;
    }

    /**
     * Method to track if Button X was pressed
     * To ensure that the continuous holding of Button X does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButtonX = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetButtonXPress() {
        boolean isPressedButtonX = false;
        if (!gp1ButtonXLast &&  ssGamepad1.x) {
            isPressedButtonX = true;
        }
        gp1ButtonXLast =  ssGamepad1.x;
        return isPressedButtonX;
    }

    public boolean gp2GetButtonXPress() {
        boolean isPressedButtonX = false;
        if (!gp2ButtonXLast &&  ssGamepad2.x) {
            isPressedButtonX = true;
        }
        gp2ButtonXLast =  ssGamepad2.x;
        return isPressedButtonX;
    }
    public boolean gp1GetX(){
        return  ssGamepad1.x;
    }
    public boolean gp2GetX(){
        return  ssGamepad2.x;
    }

    /**
     * Method to track if Button B was pressed to move Arm
     * To ensure that the continuous holding of Button Y does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButtonB = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetButtonBPress() {
        boolean isPressedButtonB = false;
        if (!gp1ButtonBLast &&  ssGamepad1.b) {
            isPressedButtonB = true;
        }
        gp1ButtonBLast =  ssGamepad1.b;
        return isPressedButtonB;
    }
    public boolean gp2GetButtonBPress() {
        boolean isPressedButtonB = false;
        if (!gp2ButtonBLast &&  ssGamepad2.b) {
            isPressedButtonB = true;
        }
        gp2ButtonBLast =  ssGamepad2.b;
        return isPressedButtonB;
    }
    public boolean gp1GetB(){
        return  ssGamepad1.b;
    }
    public boolean gp2GetB(){
        return  ssGamepad2.b;
    }

    /**
     * Method to track if Dpad_up was pressed
     * To ensure that the continuous holding of Dpad_up does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedDpad_up = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetDpad_upPress() {
        boolean isPressedDpad_up;
        isPressedDpad_up = false;
        if (!gp1Dpad_upLast &&  ssGamepad1.dpad_up) {
            isPressedDpad_up = true;
        }
        gp1Dpad_upLast =  ssGamepad1.dpad_up;
        return isPressedDpad_up;
    }
    public boolean gp2GetDpad_upPress() {
        boolean isPressedDpad_up;
        isPressedDpad_up = false;
        if (!gp2Dpad_upLast &&  ssGamepad2.dpad_up) {
            isPressedDpad_up = true;
        }
        gp2Dpad_upLast =  ssGamepad2.dpad_up;
        return isPressedDpad_up;
    }

    public boolean gp1GetDpad_up(){
        return  ssGamepad1.dpad_up;
    }
    public boolean gp2GetDpad_up(){
        return  ssGamepad2.dpad_up;
    }

    /**
     * Method to track if Dpad_down was pressed
     * To ensure that the continuous holding of Dpad_up does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedDpad_down = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetDpad_downPress() {
        boolean isPressedDpad_down;
        isPressedDpad_down = false;
        if (!gp1Dpad_downLast &&  ssGamepad1.dpad_down) {
            isPressedDpad_down = true;
        }
        gp1Dpad_downLast =  ssGamepad1.dpad_down;
        return isPressedDpad_down;
    }
    public boolean gp2GetDpad_downPress() {
        boolean isPressedDpad_down;
        isPressedDpad_down = false;
        if (!gp2Dpad_downLast &&  ssGamepad2.dpad_down) {
            isPressedDpad_down = true;
        }
        gp2Dpad_downLast =  ssGamepad2.dpad_down;
        return isPressedDpad_down;
    }

    public boolean gp1GetDpad_down(){
        return  ssGamepad1.dpad_down;
    }
    public boolean gp2GetDpad_down(){
        return  ssGamepad2.dpad_down;
    }

    /**
     * Method to track if Dpad_left was pressed
     * To ensure that the continuous holding of Dpad_up does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedDpad_left = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetDpad_leftPress() {
        boolean isPressedDpad_left;
        isPressedDpad_left = false;
        if (!gp1Dpad_leftLast &&  ssGamepad1.dpad_left) {
            isPressedDpad_left = true;
        }
        gp1Dpad_leftLast =  ssGamepad1.dpad_left;
        return isPressedDpad_left;
    }
    public boolean gp2GetDpad_leftPress() {
        boolean isPressedDpad_left;
        isPressedDpad_left = false;
        if (!gp2Dpad_leftLast &&  ssGamepad2.dpad_left) {
            isPressedDpad_left = true;
        }
        gp2Dpad_leftLast =  ssGamepad2.dpad_left;
        return isPressedDpad_left;
    }

    public boolean gp1GetDpad_left(){
        return  ssGamepad1.dpad_left;
    }
    public boolean gp2GetDpad_left(){
        return  ssGamepad2.dpad_left;
    }

    /**
     * Method to track if Dpad_right was pressed
     * To ensure that the continuous holding of Dpad_up does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedDpad_left = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetDpad_rightPress() {
        boolean isPressedDpad_right;
        isPressedDpad_right = false;
        if (!gp1Dpad_rightLast &&  ssGamepad1.dpad_right) {
            isPressedDpad_right = true;
        }
        gp1Dpad_rightLast =  ssGamepad1.dpad_right;
        return isPressedDpad_right;
    }
    public boolean gp2GetDpad_rightPress() {
        boolean isPressedDpad_right;
        isPressedDpad_right = false;
        if (!gp2Dpad_rightLast &&  ssGamepad2.dpad_right) {
            isPressedDpad_right = true;
        }
        gp2Dpad_rightLast =  ssGamepad2.dpad_right;
        return isPressedDpad_right;
    }
    public boolean gp1GetDpad_right(){
        return  ssGamepad1.dpad_right;
    }
    public boolean gp2GetDpad_right(){
        return  ssGamepad2.dpad_right;
    }

    public boolean gp1GetStart(){
        return  ssGamepad1.start;
    }
    public boolean gp2GetStart(){
        return  ssGamepad2.start;
    }

}