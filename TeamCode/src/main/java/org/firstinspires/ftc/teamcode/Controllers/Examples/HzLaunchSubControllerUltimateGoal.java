package org.firstinspires.ftc.teamcode.Controllers.Examples;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubSystems.HzDrive;
import org.firstinspires.ftc.teamcode.GameOpModes.Examples.HzGameFieldUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzIntakeUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzLauncherUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzMagazineUltimateGoal;

/**
 * Launch Controller Class that manages the actions and state of intake, Magazine and Launcher to
 * ensure that launch button is enabled and happens only when everything is ready for launch
 * The controller also protects that launcher is disabled and magazine moved to collect state when
 * intake is turned on.
 */
public class HzLaunchSubControllerUltimateGoal {

    public enum LAUNCH_MODE{
        MANUAL,
        AUTOMATED
    };
    public LAUNCH_MODE launchMode = LAUNCH_MODE.MANUAL;

    public enum LAUNCH_READINESS{
        READY,
        NOT_READY
    };
    public LAUNCH_READINESS launchReadiness = LAUNCH_READINESS.NOT_READY;

    public enum LAUNCH_ACTIVATION{
        ACTIVATED,
        NOT_ACTIVATED
    }
    public LAUNCH_ACTIVATION launchActivation = LAUNCH_ACTIVATION.NOT_ACTIVATED;

    public enum ROBOT_ZONE{
        HIGH_GOAL_ZONE,
        MID_GOAL_ZONE,
        LOW_GOAL_ZONE,
        POWER_GOAL_ZONE
    };
    public ROBOT_ZONE robotZone = ROBOT_ZONE.HIGH_GOAL_ZONE;

    public enum LAUNCH_TARGET{
        HIGH_GOAL,
        MID_GOAL,
        LOW_GOAL,
        POWER_SHOT1,
        POWER_SHOT2,
        POWER_SHOT3
    };
    public LAUNCH_TARGET lcTarget = LAUNCH_TARGET.HIGH_GOAL;
    public Vector2d lcTargetVector = HzGameFieldUltimateGoal.ORIGIN;

    public enum LAUNCHER_ALIGNMENT{
        TARGET_ALIGNED,
        TARGET_NOT_ALIGNED
    };
    public LAUNCHER_ALIGNMENT launcherAlignment = LAUNCHER_ALIGNMENT.TARGET_NOT_ALIGNED;

    public double distanceFromTarget, lclaunchMotorPower, angleToTarget;
    public double lclaunchMotorVelocity;
    public boolean batteryCorrectionFlag = false;

    public HzLauncherUltimateGoal lcHzLauncherUltimateGoal;
    public HzIntakeUltimateGoal lcHzIntakeUltimateGoal;
    public HzMagazineUltimateGoal lcHzMagazineUltimateGoal;
    public HzDrive lcHzDrive;
    public HardwareMap lcHzHardwareMap;

    public HzLaunchSubControllerUltimateGoal(HardwareMap lcHzhardwareMapPassed, HzLauncherUltimateGoal lcHzLauncherUltimateGoalPassed, HzIntakeUltimateGoal lcHzIntakeUltimateGoalPassed, HzMagazineUltimateGoal lcHzMagazineUltimateGoalPassed,
                                             HzDrive lcHzDrivePassed){
        lcHzLauncherUltimateGoal = lcHzLauncherUltimateGoalPassed;
        lcHzMagazineUltimateGoal = lcHzMagazineUltimateGoalPassed;
        lcHzIntakeUltimateGoal = lcHzIntakeUltimateGoalPassed;
        lcHzDrive = lcHzDrivePassed;
        lcHzHardwareMap = lcHzhardwareMapPassed;
    }

    /**
     * In case of battery dependency, this is used to set launcher velocity based on battery state
     */
    public double batteryCorrectFlyWheelVelocity(double flywheelVelcityToCorrect){
        double batteryVoltage = lcHzDrive.getBatteryVoltage(lcHzHardwareMap);
        double batteryCorrectedFlyWheelVelocity = 1500;
        if ((batteryCorrectionFlag) && (batteryVoltage >13.0)) {
            batteryCorrectedFlyWheelVelocity = flywheelVelcityToCorrect
                    - (batteryVoltage - 13.0) * lcHzLauncherUltimateGoal.FLYWHEEL_BATTERY_CORRECTION;
        } else {
            batteryCorrectedFlyWheelVelocity = flywheelVelcityToCorrect;
        }
        return batteryCorrectedFlyWheelVelocity;
        /*if (batteryVoltage > 13.0){
            lcHzLauncher.FLYWHEEL_NOMINAL_VELOCITY_HIGH_GOAL = 1500;
            lcHzLauncher.FLYWHEEL_NOMINAL_VELOCITY_POWERSHOT = 1440;
        } else if (batteryVoltage > 13.0){
            lcHzLauncher.FLYWHEEL_NOMINAL_VELOCITY_HIGH_GOAL = 1500;
            lcHzLauncher.FLYWHEEL_NOMINAL_VELOCITY_POWERSHOT = 1440;
        } else if (batteryVoltage > 12.5) {
            lcHzLauncher.FLYWHEEL_NOMINAL_VELOCITY_HIGH_GOAL = 1540;
            lcHzLauncher.FLYWHEEL_NOMINAL_VELOCITY_POWERSHOT = 1460;
        } else if (batteryVoltage > 12.0) {
            lcHzLauncher.FLYWHEEL_NOMINAL_VELOCITY_HIGH_GOAL = 1580;
            lcHzLauncher.FLYWHEEL_NOMINAL_VELOCITY_POWERSHOT = 1500;
        }*/
    }

    public boolean activateLaunchReadinessState;

    /**
     * Launch readiness is verified and activated
     * @return launch readiness state
     */
    public LAUNCH_READINESS activateLaunchReadiness() {
        launchActivation = LAUNCH_ACTIVATION.ACTIVATED;

        //setLauncherFlyWheelNominalVelocityBasedOnBattery();

        if (launchMode == LAUNCH_MODE.MANUAL)  {
            turnRobotToNormalControl();
        }

        if (lcHzIntakeUltimateGoal.intakeMotorState != HzIntakeUltimateGoal.INTAKE_MOTOR_STATE.STOPPED){
            lcHzIntakeUltimateGoal.stopIntakeMotor();
        }

        lcHzMagazineUltimateGoal.moveMagazineTo = HzMagazineUltimateGoal.MOVE_MAGAZINE_TO.LAUNCH;

        if (lcHzMagazineUltimateGoal.magazinePosition == HzMagazineUltimateGoal.MAGAZINE_POSITION.AT_LAUNCH){
            activateLaunchReadinessState = false;
            launchReadiness = LAUNCH_READINESS.READY;
        } else {
            launchReadiness = LAUNCH_READINESS.NOT_READY;
        }

        //gpVuforia.identifyCurrentLocation();

        if (launchMode == LAUNCH_MODE.AUTOMATED && launchReadiness == LAUNCH_READINESS.READY)  {
            determineLaunchTarget();
            turnRobotToTarget();
            runLauncherByDistanceToTarget();
        }

        if (launchMode == LAUNCH_MODE.MANUAL && launchReadiness == LAUNCH_READINESS.READY) {
            if (lcTarget == LAUNCH_TARGET.HIGH_GOAL){
                //lclaunchMotorVelocity = batteryCorrectFlyWheelVelocity(lcHzLauncher.flyWheelVelocityHighGoal);
                lclaunchMotorVelocity = lcHzLauncherUltimateGoal.flyWheelVelocityHighGoal;
                lcHzLauncherUltimateGoal.runFlyWheelToTarget(lclaunchMotorVelocity);
            }
            if (lcTarget == LAUNCH_TARGET.POWER_SHOT1 ||
                    lcTarget ==LAUNCH_TARGET.POWER_SHOT2 ||
                    lcTarget == LAUNCH_TARGET.POWER_SHOT3) {
                //lclaunchMotorVelocity = batteryCorrectFlyWheelVelocity(lcHzLauncher.flyWheelVelocityPowerShot);
                lclaunchMotorVelocity = lcHzLauncherUltimateGoal.flyWheelVelocityPowerShot;
                lcHzLauncherUltimateGoal.runFlyWheelToTarget(lclaunchMotorVelocity);
            }
        }
        return launchReadiness;
    }

    public boolean deactivateLaunchReadinessState = false;

    /**
     * Launch readiness is deactivated
     */
    public void deactivateLaunchReadiness(){
        launchActivation = HzLaunchSubControllerUltimateGoal.LAUNCH_ACTIVATION.NOT_ACTIVATED;
        lcHzLauncherUltimateGoal.stopFlyWheel();
        turnRobotToNormalControl();
        deactivateLaunchReadinessState = false;
    }

    /**
     * Determine distance from target and determine launch velocity
     */
    public void runLauncherByDistanceToTarget(){
        getDistanceFromTarget();
        setLaunchMotorVelocity();
        lcHzLauncherUltimateGoal.runFlyWheelToTarget(lclaunchMotorVelocity);
    }

    /**
     * Set the launch target based on key input or position of launch and sets the launcher
     * to run at correct velocity
     */
    public void determineLaunchTarget(){
        //determineLaunchTarget : Determine the launch target based on current zone of the robot
        //and Y,X,A,B button pressed. Returns launchTargetSelected
        if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            switch (lcTarget) {
                case HIGH_GOAL:
                    lcHzDrive.drivePointToAlign = HzGameFieldUltimateGoal.BLUE_TOWER_GOAL;
                    lcTargetVector = HzGameFieldUltimateGoal.BLUE_TOWER_GOAL;
                    break;
                case MID_GOAL:
                    lcHzDrive.drivePointToAlign = HzGameFieldUltimateGoal.RED_TOWER_GOAL;
                    lcTargetVector = HzGameFieldUltimateGoal.RED_TOWER_GOAL;
                    break;
                case LOW_GOAL:
                    lcHzDrive.drivePointToAlign = HzGameFieldUltimateGoal.BLUE_TOWER_GOAL;
                    lcTargetVector = HzGameFieldUltimateGoal.BLUE_TOWER_GOAL;
                    break;
                case POWER_SHOT1:
                    lcHzDrive.drivePointToAlign = HzGameFieldUltimateGoal.BLUE_POWERSHOT1;
                    lcTargetVector = HzGameFieldUltimateGoal.BLUE_POWERSHOT1;
                    break;
                case POWER_SHOT2:
                    lcHzDrive.drivePointToAlign = HzGameFieldUltimateGoal.BLUE_POWERSHOT2;
                    lcTargetVector = HzGameFieldUltimateGoal.BLUE_POWERSHOT2;
                    break;
                case POWER_SHOT3:
                    lcHzDrive.drivePointToAlign = HzGameFieldUltimateGoal.BLUE_POWERSHOT3;
                    lcTargetVector = HzGameFieldUltimateGoal.BLUE_POWERSHOT3;
                    break;
            }
        }

        if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.RED_ALLIANCE) {
            switch (lcTarget) {
                case HIGH_GOAL:
                    lcHzDrive.drivePointToAlign = HzGameFieldUltimateGoal.RED_TOWER_GOAL;
                    lcTargetVector = HzGameFieldUltimateGoal.RED_TOWER_GOAL;
                    break;
                case MID_GOAL:
                    lcHzDrive.drivePointToAlign = HzGameFieldUltimateGoal.BLUE_TOWER_GOAL;
                    lcTargetVector = HzGameFieldUltimateGoal.BLUE_TOWER_GOAL;
                    break;
                case LOW_GOAL:
                    lcHzDrive.drivePointToAlign = HzGameFieldUltimateGoal.RED_TOWER_GOAL;
                    lcTargetVector = HzGameFieldUltimateGoal.RED_TOWER_GOAL;
                    break;
                case POWER_SHOT1:
                    lcHzDrive.drivePointToAlign = HzGameFieldUltimateGoal.RED_POWERSHOT1;
                    lcTargetVector = HzGameFieldUltimateGoal.RED_POWERSHOT1;
                    break;
                case POWER_SHOT2:
                    lcHzDrive.drivePointToAlign = HzGameFieldUltimateGoal.RED_POWERSHOT2;
                    lcTargetVector = HzGameFieldUltimateGoal.RED_POWERSHOT2;
                    break;
                case POWER_SHOT3:
                    lcHzDrive.drivePointToAlign = HzGameFieldUltimateGoal.RED_POWERSHOT3;
                    lcTargetVector = HzGameFieldUltimateGoal.RED_POWERSHOT3;
                    break;
            }
        }
    }

    /**
     * In automated mode, robot turns to target zone
     */
    public void turnRobotToTarget(){
        //If MODE_AUTOMATED, turnRobotToTarget : turn the robot to face the target based on angle determined.
        // Ensure this function is on time out, or on a parallel thread where drivers can override,
        // so that robot does not get locked in this function.
        // (Driver has to manually turn the robot in MODE_AUTOMATED)
        if (getLaunchMode() == LAUNCH_MODE.AUTOMATED) {
            lcHzDrive.driveMode = HzDrive.DriveMode.ALIGN_TO_POINT;
            lcHzDrive.driveTrainPointFieldModes();
        }
    }

    /**
     * Return robot for target pointed control to Normal control
     */
    public void turnRobotToNormalControl(){
        lcHzDrive.driveMode = HzDrive.DriveMode.NORMAL_CONTROL;
        //lcDrive.driveTrainPointFieldModes();
    }

    public ROBOT_ZONE getRobotZone() {
        return robotZone;
    }

    /**
     * Get distance from target
     */
    public void getDistanceFromTarget() {
        distanceFromTarget = lcTargetVector.distTo(lcHzDrive.poseEstimate.vec());
    }

    /**
     * Set Launch velocity based on distance
     */
    public void setLaunchMotorVelocity() {
        if (distanceFromTarget > 66 && distanceFromTarget < 138) {
            switch (lcTarget) {
                case POWER_SHOT1:
                case POWER_SHOT2:
                case POWER_SHOT3:
                    lclaunchMotorVelocity = lcHzLauncherUltimateGoal.FLYWHEEL_NOMINAL_VELOCITY_POWERSHOT;
                    break;
                case HIGH_GOAL:
                    lclaunchMotorVelocity = lcHzLauncherUltimateGoal.FLYWHEEL_NOMINAL_VELOCITY_HIGH_GOAL;
                    break;
            }
        } else {
            lclaunchMotorVelocity = 0;
        }

    }

    /**
     * get launch mode
     */
    public LAUNCH_MODE getLaunchMode(){
        return launchMode;
    }

    /**
     * Toggle between automated turn to target and manual control modes
     */
    public void toggleModeManualAutomated() {
        if (launchMode == LAUNCH_MODE.AUTOMATED) {
            launchMode = LAUNCH_MODE.MANUAL;
        } else {
            launchMode = LAUNCH_MODE.AUTOMATED;
        }
    }
}


