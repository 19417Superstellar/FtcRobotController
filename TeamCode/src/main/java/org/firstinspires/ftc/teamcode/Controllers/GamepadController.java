package org.firstinspires.ftc.teamcode.Controllers;


        //import com.acmerobotics.roadrunner.geometry.Vector2d;
        import android.os.Debug;

        import com.acmerobotics.roadrunner.geometry.Vector2d;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.Gamepad;
        import com.qualcomm.robotcore.hardware.SwitchableLight;

        import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
        import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
        import org.firstinspires.ftc.teamcode.Subsystems.SSArm;
        import org.firstinspires.ftc.teamcode.Subsystems.SSClaw;
        import org.firstinspires.ftc.teamcode.Subsystems.SSElevator;


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
    public SSClaw ssClaw;
    public SSElevator ssElevator;
    public SSArm ssArm;

    /**
     * Constructor for  ssGamepad1 class that extends gamepad.
     * Assign the gamepad1 given in OpMode to the gamepad used here.
     */
    public GamepadController(Gamepad ssGamepad1,
                             Gamepad  ssGamepad2,
                             DriveTrain driveTrain,
                             SSClaw ssClaw,
                             SSElevator ssElevator,
                             SSArm ssArm,
                             LinearOpMode opMode) {
        this.ssGamepad1 =  ssGamepad1;
        this.ssGamepad2 =  ssGamepad2;
        this.driveTrain = driveTrain;
        this.ssClaw = ssClaw;
        this.ssElevator = ssElevator;
        this.ssArm = ssArm;
    }

    /**
     *runByGamepad is the main controller function that runs each subsystem controller based on states
     */
    public void runByGamepadControl(){
       runElevator();
       runArm();
       runClaw();
       runDriveControl_byRRDriveModes();
    }

    /**
     * runByGamepadRRDriveModes sets modes for Road Runner such as ROBOT and FIELD Centric Modes. <BR>
     *     RR Drive Train
     */
    public void runDriveControl_byRRDriveModes() {
        driveTrain.poseEstimate = driveTrain.getPoseEstimate();

        driveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;

       // if (driveTrain.driveType == DriveTrain.DriveType.ROBOT_CENTRIC){
         //   if (ssElevator.getElevatorPosition() != SSElevator.ELEVATOR_POSITION.LEVEL_LOW) {
                driveTrain.gamepadInput = new Vector2d(
                        -gp1TurboMode(gp1GetLeftStickY()),
                        -gp1TurboMode(gp1GetLeftStickX()));
         //   } else {
                // Avoid using Turbo when elevator is at Level 0
        //        driveTrain.gamepadInput = new Vector2d(
          //              -limitStick(gp1GetLeftStickY()),
          //              -limitStick(gp1GetLeftStickX()));
         //   }
      //  }

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
     * Elevator function mapped to the gamepad buttons
     */
    public void runElevator() {

        if (gp2GetStart() && gp2GetDpad_downPress()) {
            // If start + d-pad down, then reset elevator to
            // all the way down
        //    if (ssElevator.getElevatorPosition() != SSElevator.ELEVATOR_POSITION.LEVEL_LOW) {
        //        ssElevator.moveElevatorLevelLow();
        //    }
            ssElevator.bringElevatorAllTheWayDown();
        }
        else if ( gp2GetDpad_downPress()) {
            ssElevator.moveSSElevatorSlightlyDown();
        }

        if (gp2GetDpad_upPress()) {
            ssElevator.moveSSElevatorSlightlyUp();
//            if (ssElevator.getElevatorPosition() != SSElevator.ELEVATOR_POSITION.LEVEL_HIGH) {
//                ssElevator.moveElevatorLevelHigh();
//            }
        }


        /*start+left trigger in gamepad2 will move slightly down
          left trigger in gamepad2 will move slightly up
         */

//        if (gp2GetStart()) {
//            if (gp2GetLeftTriggerPress()) {
//                ssElevator.moveSSElevatorSlightlyUp();
//            }
//            if (gp2GetRightTriggerPress()) {
//                ssElevator.moveSSElevatorSlightlyDown();
//            }
//        }

        if (ssElevator.runElevatorToLevelState) {
            ssElevator.runElevatorToLevel(ssElevator.motorPowerToRun);
        }

    } // End of runElevator function

    /**
     * Start of runArm function
     * */
    public void runArm() {
        if (!gp2GetStart() && gp2GetButtonAPress()) {
            if (ssArm.getArmPosition() != SSArm.ARM_POSITION.ARM_POSITION_INTAKE_FORWARD){
                ssElevator.moveElevatorLevelLow();
                ssArm.moveArmIntakeForward();
            }
        }

        /*
        Disabled REAR INTAKE till we can strenghen the arm and reinforce it

        if (gp2GetRightTriggerPress()) {
            if (ssArm.getArmPosition() != SSArm.ARM_POSITION.ARM_POSITION_INTAKE_REAR) {
                ssArm.moveArmIntakeRear();

            }
        }
        */

        if (!gp2GetStart() && gp2GetButtonXPress()) {
            if (ssArm.getArmPosition() != SSArm.ARM_POSITION.ARM_POSITION_LOW) {
                // Make sure claw is closed
                ssClaw.setGripClose();
                // Move the elevator to low position
                ssElevator.moveElevatorLevelLow();
                ssArm.moveArmLow();

            }
        }

        if (!gp2GetStart() && gp2GetButtonBPress()) {
            if (ssArm.getArmPosition() != SSArm.ARM_POSITION.ARM_POSITION_MID) {
                // Make sure claw is closed
                ssClaw.setGripClose();
                // Move the elevator to low position
                ssElevator.moveElevatorLevelLow();
                ssArm.moveArmMid();
            }
        }

        if (!gp2GetStart() && gp2GetButtonYPress()) {
            if (ssArm.getArmPosition() != SSArm.ARM_POSITION.ARM_POSITION_HIGH) {
                // Make sure claw is closed
                ssClaw.setGripClose();
                // Move the elevator to high position
                ssElevator.moveElevatorLevelHigh();
                ssArm.moveArmHigh();
            }
        }

        /*start+left trigger in gamepad1 will move slightly down
          left trigger in gamepad1 will move slightly up
         */

        if (gp2GetStart()) {
            if (gp2GetButtonYPress()) {
                ssArm.moveSSArmSlightlyUp();
            }
            if (gp2GetButtonXPress()) {
                ssArm.moveSSArmSlightlyDown();
            }
        }

        if (ssElevator.runElevatorToLevelState) {
            ssElevator.runElevatorToLevel(ssElevator.motorPowerToRun);
        }

        if (ssArm.runArmToLevelState) {
            ssArm.runArmToLevel(ssArm.motorPowerToRun);
        }

        if (gp2GetStart() && gp2GetDpad_down()) {
            ssArm.resetArm();
        }
    } //End of runArm function

    public void runClaw() {
        if (gp2GetLeftBumperPress()) {
            if (ssClaw.getGripServoState() == SSClaw.GRIP_SERVO_STATE.GRIP_OPEN) {
                ssClaw.setGripClose();
            } else {
                ssClaw.setGripOpen();
            }
        }
    }

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
        boolean isPressedRightTrigger = !gp1RightTriggerLast && (gp1GetRightTrigger() > 0.7);
        gp1RightTriggerLast = (gp1GetRightTrigger()>0.7);
        return isPressedRightTrigger;
    }

    public boolean gp2GetRightTriggerPress() {
        boolean isPressedRightTrigger = !gp2RightTriggerLast && (gp2GetRightTrigger() > 0.7);
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
        boolean isPressedLeftTrigger = !gp1LeftTriggerLast && (gp1GetLeftTrigger() > 0.7);
        gp1LeftTriggerLast = (gp1GetLeftTrigger()>0.7);
        return isPressedLeftTrigger;
    }

    public boolean gp2GetLeftTriggerPress() {
        boolean isPressedLeftTrigger = !gp2LeftTriggerLast && (gp2GetLeftTrigger() > 0.7);
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
        boolean isPressedLeftBumper = !gp1LeftBumperLast && ssGamepad1.left_bumper;
        gp1LeftBumperLast =  ssGamepad1.left_bumper;
        return isPressedLeftBumper;
    }

    public boolean gp2GetLeftBumperPress() {
        boolean isPressedLeftBumper = !gp2LeftBumperLast && ssGamepad2.left_bumper;
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
        boolean isPressedRightBumper = !gp1RightBumperLast && ssGamepad1.right_bumper;
        gp1RightBumperLast =  ssGamepad1.right_bumper;
        return isPressedRightBumper;
    }

    public boolean gp2GetRightBumperPress() {
        boolean isPressedRightBumper = !gp2RightBumperLast && ssGamepad2.right_bumper;
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
        boolean isPressedButtonA = !gp1ButtonALast && ssGamepad1.a;
        gp1ButtonALast =  ssGamepad1.a;
        return isPressedButtonA;
    }
    public boolean gp2GetButtonAPress() {
        boolean isPressedButtonA = !gp2ButtonALast && ssGamepad2.a;
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
        boolean isPressedButtonY = !gp1ButtonYLast && ssGamepad1.y;
        gp1ButtonYLast =  ssGamepad1.y;
        return isPressedButtonY;
    }
    public boolean gp2GetButtonYPress() {
        boolean isPressedButtonY = !gp2ButtonYLast && ssGamepad2.y;
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
        boolean isPressedButtonX = !gp1ButtonXLast && ssGamepad1.x;
        gp1ButtonXLast =  ssGamepad1.x;
        return isPressedButtonX;
    }

    public boolean gp2GetButtonXPress() {
        boolean isPressedButtonX = !gp2ButtonXLast && ssGamepad2.x;
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
        boolean isPressedButtonB = !gp1ButtonBLast && ssGamepad1.b;
        gp1ButtonBLast =  ssGamepad1.b;
        return isPressedButtonB;
    }
    public boolean gp2GetButtonBPress() {
        boolean isPressedButtonB = !gp2ButtonBLast && ssGamepad2.b;
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
        isPressedDpad_up = !gp1Dpad_upLast && ssGamepad1.dpad_up;
        gp1Dpad_upLast =  ssGamepad1.dpad_up;
        return isPressedDpad_up;
    }
    public boolean gp2GetDpad_upPress() {
        boolean isPressedDpad_up;
        isPressedDpad_up = !gp2Dpad_upLast && ssGamepad2.dpad_up;
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
        isPressedDpad_down = !gp1Dpad_downLast && ssGamepad1.dpad_down;
        gp1Dpad_downLast =  ssGamepad1.dpad_down;
        return isPressedDpad_down;
    }
    public boolean gp2GetDpad_downPress() {
        boolean isPressedDpad_down;
        isPressedDpad_down = !gp2Dpad_downLast && ssGamepad2.dpad_down;
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
        isPressedDpad_left = !gp1Dpad_leftLast && ssGamepad1.dpad_left;
        gp1Dpad_leftLast =  ssGamepad1.dpad_left;
        return isPressedDpad_left;
    }
    public boolean gp2GetDpad_leftPress() {
        boolean isPressedDpad_left;
        isPressedDpad_left = !gp2Dpad_leftLast && ssGamepad2.dpad_left;
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
     * To ensure that the continuous holding of Dpad_up does   send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedDpad_left = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetDpad_rightPress() {
        boolean isPressedDpad_right;
        isPressedDpad_right = !gp1Dpad_rightLast && ssGamepad1.dpad_right;
        gp1Dpad_rightLast =  ssGamepad1.dpad_right;
        return isPressedDpad_right;
    }
    public boolean gp2GetDpad_rightPress() {
        boolean isPressedDpad_right;
        isPressedDpad_right = !gp2Dpad_rightLast && ssGamepad2.dpad_right;
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
