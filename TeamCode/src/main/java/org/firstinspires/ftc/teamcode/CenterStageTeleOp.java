package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class CenterStageTeleOp extends LinearOpMode {
    hwMap hw;
    @Override
    public void runOpMode() throws InterruptedException {
        hw = new hwMap(this);

        waitForStart();
    }
}
