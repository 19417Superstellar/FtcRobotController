package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "DashBoard Teleop", group = "Misc")
public class DashboardTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

    }

    static {
        FtcDashboard.suppressOpMode();
    }

}
