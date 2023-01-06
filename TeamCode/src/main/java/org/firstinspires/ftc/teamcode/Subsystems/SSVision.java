package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;

import java.util.List;

    /**
     *
     * Vuforia implementation for Super Stellar
     *  - Integrates Tensor flow and Navigation to single code
     *  - Updated to manage 1 cameras
     *
     * This 20222-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
     * determine the position of the PowerPlay game elements.
     *
     * This 2022-2023 OpMode illustrates the basics of using the Vuforia localizer to determine
     * positioning and orientation of robot on the ULTIMATE GOAL FTC field.
     * The code is structured as a LinearOpMode
     *
     * When images are located, Vuforia is able to determine the position and orientation of the
     * image relative to the camera.
     *
     * Red Alliance station is on the right and the
     * Blue Alliance Station is on the left.
     * There are a total of three image targets for the ULTIMATE GOAL game.
     * Three of the targets are placed in the center of the Red Alliance, Audience (Front),
     * and Blue Alliance perimeter walls.
     *
     * A final calculation then uses the location of the camera on the robot to determine the
     * robot's location and orientation on the field.
     *
     * @see VuforiaLocalizer
     * @see VuforiaTrackableDefaultListener
     * see  ultimategoal/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
     */

    public class SSVision {

        public enum VISION_STATE {
            TFOD_INIT,
            TFOD_ACTIVE,
            TFOD_RUNNING,
            INACTIVE
        }
        public VISION_STATE visionState = VISION_STATE.INACTIVE;

        public enum ACTIVE_WEBCAM{
            WEBCAM1,
        }

        /*
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         * Vuforia license keys are always 380 characters long.
         */


        private static final String VUFORIA_KEY =
                "AZME4Mr/////AAABmY+MAyxxT0IileR7JBqaAPsxN2XNYlaGBtEjYaHOlVVTqPQf7NH9eIrosYKKQHPGEXLtJUsdMwZ9e3EXBfy6arulcLPvdpW9bqAB2F2MJJXo35lLA096l/t/LQTi+etVso0Xc5RYkTVSIP3YABp1TeOaF8lCSpjVhPIVW3l/c/XlrnEMPhJk9IgqMEp4P/ifqAqMMMUAIKPEqIrXIv79TvAfdIJig46gfQGaQl5tFHr3nmvMbh/LhFrh5AWAy3B/93cCkOszmYkdHxZStbNB5lMdkTnf3sCnYbQY4jviorfhYrAkqHWH6vNOB9lUt8dOSeHsDtlk33e/6xQgOCNYFN80anYMp82JNDBFX3oyGliV";


        // Class Members
        private OpenGLMatrix lastLocation = null;
        private VuforiaLocalizer vuforia = null;
        private VuforiaTrackables targets   = null ;

        /**
         * This is the webcam we are to use. As with other hardware devices such as motors and
         * servos, this device is identified using the robot configuration tool in the FTC application.
         */
        WebcamName webcamName = null;

        public  boolean targetVisible = false;
        private float phoneXRotate    = 0;
        private float phoneYRotate    = 0;
        private float phoneZRotate    = 0;

        public VuforiaLocalizer.Parameters parameters;

        public List<VuforiaTrackable> allTrackables;

        // Superstellar custom trained model file
        //private static final String TFOD_MODEL_ASSET = "SS_PowerPlay.tflite";

        // Custom Superstellar labels for the team sleeve
//        public static final String[] LABELS = {
//                "Yellow Star",
//                "Black Moon",
//                "Purple Sparkles"
//        };

        // Default FTC model file
        private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

        // Default FTC labels for the sleeve
        private static final String[] LABELS = {
                "1 Bolt",
                "2 Bulb",
                "3 Panel"
        };

        public String detectedLabel = "None";
        public float detectedLabelLeft, detectedLabelRight, detectedLabelTop, detectedLabelBottom;
        public static float[] targetPosition = {
                //TODO : Update values based on marker location identifier
                250,
                600,
                1000
        };

        private TFObjectDetector tfod;
        private List<Recognition> recognitions;
        public GameField.VISION_IDENTIFIED_TARGET targetLevelDetected = GameField.VISION_IDENTIFIED_TARGET.UNKNOWN;

        /**
         * Initialize the Vuforia localization engine.
         */
        public SSVision(HardwareMap hardwareMap, ACTIVE_WEBCAM activeWebcam) {
            activeWebcam = ACTIVE_WEBCAM.WEBCAM1;

            if (activeWebcam == ACTIVE_WEBCAM.WEBCAM1){
                webcamName = hardwareMap.get(WebcamName.class, "Webcam1");
            } /*

            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
             * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
             */
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            parameters.vuforiaLicenseKey = VUFORIA_KEY;

            /*
             */
            parameters.cameraName = webcamName;

            // Make sure extended tracking is disabled for this example.
            parameters.useExtendedTracking = false;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);
            initTfod(hardwareMap);
        }

        /**
         * Initialize the TensorFlow Object Detection engine.
         */
        private void initTfod(HardwareMap hardwareMap) {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.8f;
            tfodParameters.isModelTensorFlow2 = true;
            tfodParameters.inputSize = 320;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

        /*
        }*/
            visionState = VISION_STATE.TFOD_INIT;

        }

        /**
         * Activate Vuforia Tensor Flow to determine target zone
         * This is to be done at Init in Autonomous mode
         */
        public void activateVuforiaTensorFlow(){
            /**
             * Activate TensorFlow Object Detection before we wait for the start command.
             **/
            if (tfod != null) {
                tfod.activate();
                visionState = VISION_STATE.TFOD_ACTIVE;

                // The TensorFlow software will scale the input images from the camera to a lower resolution.
                // This can result in lower detection accuracy at longer distances (> 55cm or 22").
                // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
                // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
                // should be set to the value of the images used to create the TensorFlow Object Detection model
                // (typically 1.78 or 16/9).

                // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
                //tfod.setZoom(1.75, 16.0/9.0);
                tfod.setZoom(1.0, 16.0/9.0);
                recognitions = tfod.getUpdatedRecognitions();
            }
        }

        /**
         * Run Tensor flow algorithm.
         * This is to be run till the play button is pressed.. the last target zone identified is returned.
         * @return
         */
        public GameField.VISION_IDENTIFIED_TARGET runVuforiaTensorFlow() {
            visionState = VISION_STATE.TFOD_RUNNING;

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                recognitions = tfod.getUpdatedRecognitions();
                if (recognitions != null) {
                    //telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (recognitions.size() == 0 ) {
                        // empty list.  no objects recognized.
                        detectedLabel = "None";
                        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE) {
                            targetLevelDetected = GameField.VISION_IDENTIFIED_TARGET.LEVEL3;
                        } else { //GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE
                            targetLevelDetected = GameField.VISION_IDENTIFIED_TARGET.LEVEL1;
                        }
                    } else {
                        // list is not empty.
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                    /* step through the list of recognitions and display boundary info.
                    for (Recognition recognition : recognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                    }*/

                        for (Recognition recognition : recognitions) {
                            // check label to see which target zone to go after.
                        /*detectedLabel = recognition.getLabel();
                        detectedLabelLeft = recognition.getLeft();
                        detectedLabelRight = recognition.getRight();
                        detectedLabelTop = recognition.getTop();
                        detectedLabelBottom = recognition.getBottom();*/
                            {
                                detectedLabel = recognition.getLabel();
                                detectedLabelLeft = recognition.getLeft();
                                detectedLabelRight = recognition.getRight();
                                detectedLabelTop = recognition.getTop();
                                detectedLabelBottom = recognition.getBottom();

                                if (recognition.getLeft() < targetPosition[0]) {
                                    if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE) {
                                        targetLevelDetected = GameField.VISION_IDENTIFIED_TARGET.LEVEL1;
                                    } else { //GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE
                                        targetLevelDetected = GameField.VISION_IDENTIFIED_TARGET.LEVEL2;
                                    }
                                } else if (recognition.getLeft() < targetPosition[1]) {
                                    if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE) {
                                        targetLevelDetected = GameField.VISION_IDENTIFIED_TARGET.LEVEL2;
                                    } else { //GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE
                                        targetLevelDetected = GameField.VISION_IDENTIFIED_TARGET.LEVEL3;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            return targetLevelDetected;
        }

        /**
         * Stop Tensor Flow algorithm
         */
        public void deactivateVuforiaTensorFlow(){
            if (tfod != null) {
                tfod.shutdown();
                visionState = VISION_STATE.INACTIVE;
            }
        }
    }

