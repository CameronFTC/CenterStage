package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class CenterStageTeleOp extends LinearOpMode {
    hwMap hw;
    IMU imu;
    double angle;
    double startAngle;

    @Override

    public void runOpMode() throws InterruptedException {
        hw = new hwMap(this);
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        imu.initialize(parameters);

        angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        startAngle = angle;

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.start)
            {
                imu.resetYaw();
            }
            //
            //telemetry.addData("Angle: ", hw.getAngle());
            telemetry.update();

            //drive();
            fieldCentric();
            liftMove();
            intakeMove();
        }
    }

    private void drive()
    {
        //normal
        hw.fL.setPower(-(-gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x)));
        hw.fR.setPower(-(-gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x)));
        hw.bL.setPower(-(-gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.right_stick_x)));
        hw.bR.setPower(-(-gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x)));
    }

    private void fieldCentric()
    {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        //double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double heading = getAngle();

        telemetry.addData("heading: ", heading);
        telemetry.addData("start: ", startAngle);
        telemetry.addData("angle: ", hw.angle());
        telemetry.update();

        double rotX = x * Math.cos(Math.toRadians(-heading)) - y * Math.sin(Math.toRadians(-heading));
        double rotY = x * Math.sin(Math.toRadians(-heading)) + y * Math.cos(Math.toRadians(-heading));

        //rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double flPwr = (rotY + rotX + rx) / denominator;
        double blPwr = (rotY - rotX + rx) / denominator;
        double frPwr = (rotY - rotX - rx) / denominator;
        double brPwr = (rotY + rotX - rx) / denominator;

        hw.fL.setPower(-flPwr);
        hw.bL.setPower(-blPwr);
        hw.fR.setPower(-frPwr);
        hw.bR.setPower(-brPwr);
    }

    public double getAngle()
    {
        angle = hw.angle() + startAngle;
        if(angle > 180)
        {
            angle -= 360;
        }
        else if(angle < -180)
        {
            angle += 360;
        }

        return angle;
    }

    private void liftMove()
    {
        if(Math.abs(gamepad2.left_stick_y) > 0.1)
        {
            //hw.lift.setPower(gamepad2.left_stick_y);
            hw.lift2.setPower(gamepad2.left_stick_y);
        }
        else
        {
            //hw.lift.setPower(0);
            hw.lift2.setPower(0);
        }
    }

   private void intakeMove()
   {
        hw.intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
   }

}