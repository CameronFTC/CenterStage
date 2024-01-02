package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Park lol", group = "Autnomous")
public class ParkLol extends LinearOpMode {
    hwMap hw;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = new hwMap(this);

        waitForStart();

        while(opModeIsActive() && !isStopRequested())
        {
            hw.goStraightPID(-900,0.01, 0.00138138, 0.05, 4000, -0.7);

            sleep(30000);
        }
    }
}
