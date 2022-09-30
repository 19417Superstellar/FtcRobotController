package org.firstinspires.ftc.teamcode.GameOpModes;


       // import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

       /* import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
        import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
        import org.firstinspires.ftc.teamcode.SubSystems.SSArm;
        import org.firstinspires.ftc.teamcode.SubSystems.SSBucket;
        import org.firstinspires.ftc.teamcode.SubSystems.SSElevator;
        import org.firstinspires.ftc.teamcode.SubSystems.SSIntake;
        import org.firstinspires.ftc.teamcode.SubSystems.SSSpinner; */

/**
 * Freight Frenzy TeleOp mode <BR>
 *
 *  *  This code defines the TeleOp mode is done by SuperStellar Robot for Freight Frenzy.<BR>
 *
 */

@TeleOp(name = "SSTeleOp", group = "00-Teleop")
public class SSTeleOp extends LinearOpMode {

    public boolean DEBUG_FLAG = true;

    public GamepadController gamepadController;
    public DriveTrain driveTrain;

    //TODO_SS

    //public Vuforia Vuforia1;
    public Pose2d startPose = GameField.ORIGINPOSE;

    @Override
    public void runOpMode() throws InterruptedException {

        /* Create Subsystem Objects*/


        //TODO_SS


        /* Create Controllers */


        /* Wait for Start or Stop Button to be pressed */
        waitForStart();

        /* If Stop is pressed, exit OpMode */
        if (isStopRequested()) return;

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            if(DEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive()) {
                gamepadController.runByGamepadControl();

                if(DEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }

            }

        }
        GameField.poseSetInAutonomous = false;
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("DEBUG_FLAG is : ", DEBUG_FLAG);



        telemetry.update();
    }
}
