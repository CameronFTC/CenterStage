package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class CenterStageTeleOp extends LinearOpMode {
    hwMap hw;
    IMU imu;
    double angle;
    double startAngle;
    double slidePos = 0;
    double currSlidePos = 0;
    ColorSensor color;
    private RevBlinkinLedDriver lights;

    public enum liftHeight {
        none,
        retract,
        low,
        medium,
        high,
    }

    public liftHeight currLift = liftHeight.none;

    @Override

    public void runOpMode() throws InterruptedException {
        hw = new hwMap(this);
        imu = hardwareMap.get(IMU.class, "imu");

        color = hardwareMap.get(ColorSensor.class, "Color");
        lights = hardwareMap.get(RevBlinkinLedDriver.class,"Lights");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        imu.initialize(parameters);

        hw.resetAngle();
        angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        startAngle = angle;
        hw.globalAngle = RobotOrientation.getAngle();

        //hw.droneLauncher.setPosition(0.65);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.start) {
                hw.resetAngle();
                hw.globalAngle = 0;
            }
//
//            telemetry.addData("Angle: ", hw.getAngle());
//            telemetry.update();
//
//            drive();
//            robotCentric();
            fieldCentric();
            liftMove();
            intakeMove();
            intakeServos();
            //outakeMove();
            dropper();
            droneLaunch();
            colors();

            //telemetry.addData("Red", color.red());
            //telemetry.addData("Green", color.green());
            //telemetry.addData("Blue", color.blue());

            //telemetry.update();

            if (gamepad1.a) {
                currLift = liftHeight.retract;
                hw.outtake1.setPosition(0);

            }
            if (gamepad1.x) {
                currLift = liftHeight.low;
                //hw.outtake1.setPosition(1);
                //hw.outtake2.setPosition(0.9);
            }
            if (gamepad1.b) {
                currLift = liftHeight.medium;
                currLift = liftHeight.medium;
                //hw.outtake1.setPosition(1);
                //hw.outtake2.setPosition(0.9);
            }



            /*switch(currLift)
            {
                case retract:
                    if(slidePos != 0 && Math.abs(hw.lift.getCurrentPosition()) > 10)
                    {
                        double error = hw.lift.getCurrentPosition();
                        hw.lift.setPower(error * (100 / slidePos));
                        hw.lift2.setPower(-error * (100 / slidePos));
                    }
                    else
                    {
                        hw.lift.setPower(0);
                        hw.lift2.setPower(0);
                        slidePos = 0;
                    }
                    break;

                case low:
                    if(slidePos != 1500 && Math.abs(hw.lift.getCurrentPosition() + 1500) > 75)
                    {
                        double error = Math.abs(hw.lift.getCurrentPosition() + 1500);
                        hw.lift.setPower(error * (10 / (1500 - slidePos)));
                        hw.lift2.setPower(-error * (10 / (1500 - slidePos)));
                    }
                    else
                    {
                        hw.lift.setPower(0.1);
                        hw.lift2.setPower(-0.1);
                        slidePos = 1500;
                    }
                    break;

                case medium:
                    if(slidePos != 2200 && Math.abs(hw.lift.getCurrentPosition() + 2200) > 75)
                    {
                        double error = Math.abs(hw.lift.getCurrentPosition() + 2200);
                        hw.lift.setPower(error * (10 / (2200 - slidePos)));
                        hw.lift2.setPower(-error * (10 / (2200 - slidePos)));
                    }
                    else
                    {
                        hw.lift.setPower(0.1);
                        hw.lift2.setPower(-0.1);
                        slidePos = 2200;
                    }
                    break;*/
        }
    }
    //}
    private void colors(){
        String col = "";
        double RtoG = (double)color.red() / (double)color.green();
        double GtoB = (double)color.green() / (double)color.blue();
        double BtoR = (double)color.blue() / (double)color.red();
        if(color.red() < 100 && color.green() < 100 && color.blue() < 100){
            col = "nothing";
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
        }
        else if(RtoG < .6 ) {
            col = "green";
            //ham
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        else if(GtoB < 1.2){
            col = "purple";
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        }
        else if (BtoR < .6){
            col = "yellow";
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        }
        else {
            col = "white";
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        }


       /* telemetry.addData("Red", color.red());
        telemetry.addData("Green", color.green());
        telemetry.addData("Blue", color.blue());
        telemetry.addData("RtoG", RtoG);
        telemetry.addData("GtoB", GtoB);
        telemetry.addData("BtoR", BtoR);
        telemetry.addData(":", lights.getConnectionInfo());*/
        //telemetry.addData("Color", col);

        // lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);


        //telemetry.update();
    }
    private void drive() {
        //normal
        hw.fL.setPower(-(-gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x)));
        hw.fR.setPower(-(-gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x)));
        // b\hisdhfsipdhf
        hw.bL.setPower(-(-gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.right_stick_x)));
        hw.bR.setPower(-(-gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x)));
    }

    private void fieldCentric() {
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        //double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double heading = -getAngle();

        telemetry.addData("heading: ", heading);
        telemetry.addData("bruh: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("start: ", startAngle);
        telemetry.addData("angle: ", hw.angle());
        telemetry.addData("distance1: ", hw.distance1.getDistance(DistanceUnit.CM));
        telemetry.addData("distance2: ", hw.distance2.getDistance(DistanceUnit.CM));
        //telemetry.addData("encoder", TwoWheelTrackingLocalizer.getEncoderVals());
        telemetry.update();

        double rotX = x * Math.cos(Math.toRadians(-heading)) - y * Math.sin(Math.toRadians(-heading));
        double rotY = x * Math.sin(Math.toRadians(-heading)) + y * Math.cos(Math.toRadians(-heading));

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double flPwr = (rotY + rotX + rx) / denominator;
        double blPwr = (rotY - rotX + rx) / denominator;
        double frPwr = (rotY - rotX - rx) / denominator;
        double brPwr = (rotY + rotX - rx) / denominator;

        hw.fL.setPower(flPwr);
        hw.bL.setPower(blPwr);
        hw.fR.setPower(-frPwr);
        hw.bR.setPower(-brPwr);

        if (hw.distance1.getDistance(DistanceUnit.CM)>30 || hw.distance2.getDistance(DistanceUnit.CM)>30 ) {

                hw.fL.setPower(flPwr);
                hw.bL.setPower(blPwr);
                hw.fR.setPower(-frPwr);
                hw.bR.setPower(-brPwr);
            }else{

                hw.fL.setPower(flPwr*.5);
                hw.bL.setPower(blPwr*.5);
                hw.fR.setPower(-frPwr*.5);
                hw.bR.setPower(-brPwr*.5);
            }
        }

    private void fieldCentricSlow() {


            double y = -gamepad1.left_stick_y;

            double x = -gamepad1.left_stick_x;

            double rx = -gamepad1.right_stick_x;

            //double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            double heading = -getAngle();

            telemetry.addData("heading: ", heading);
            telemetry.addData("bruh: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("start: ", startAngle);
            telemetry.addData("angle: ", hw.angle());
            telemetry.update();

            double rotX = x * Math.cos(Math.toRadians(-heading)) - y * Math.sin(Math.toRadians(-heading));
            double rotY = x * Math.sin(Math.toRadians(-heading)) + y * Math.cos(Math.toRadians(-heading));

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double flPwr = (rotY + rotX + rx) / denominator*.75;

            double blPwr = (rotY - rotX + rx) / denominator*.75;
            double frPwr = (rotY - rotX - rx) / denominator*.75;
            double brPwr = (rotY + rotX - rx) / denominator*.75;

        hw.fL.setPower(flPwr);
        hw.bL.setPower(blPwr);
        hw.fR.setPower(-frPwr);
        hw.bR.setPower(-brPwr);

        if (hw.distance1.getDistance(DistanceUnit.CM)<30 || hw.distance2.getDistance(DistanceUnit.CM)<30 ) {
            if(gamepad1.left_stick_x<0) {
                hw.fL.setPower(flPwr * .5);
                hw.bL.setPower(blPwr * .5);
                hw.fR.setPower(-frPwr * .5);
                hw.bR.setPower(-brPwr * .5);
            }
        }else{
            hw.fL.setPower(flPwr);
            hw.bL.setPower(blPwr);
            hw.fR.setPower(-frPwr);
            hw.bR.setPower(-brPwr);
        }

        }


    private void robotCentric() {
        double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flPwr = (y + x + rx) / denominator;
        double blPwr = (y - x + rx) / denominator;
        double frPwr = (y - x - rx) / denominator;
        double brPwr = (y + x - rx) / denominator;


        if (hw.distance1.getDistance(DistanceUnit.CM)<30 || hw.distance2.getDistance(DistanceUnit.CM)<30 ) {
            if(gamepad1.left_stick_x<0) {
                hw.fL.setPower(flPwr * .5);
                hw.bL.setPower(blPwr * .5);
                hw.fR.setPower(-frPwr * .5);
                hw.bR.setPower(-brPwr * .5);
            }
        }else{
            hw.fL.setPower(flPwr);
            hw.bL.setPower(blPwr);
            hw.fR.setPower(-frPwr);
            hw.bR.setPower(-brPwr);
        }
    }

    public double getAngle() {
        angle = hw.getAngle();

        return angle;
    }

    private void liftMove() {
        if (Math.abs(gamepad2.left_stick_y) > 0.1) {
            hw.lift.setPower(-gamepad2.left_stick_y);
            hw.lift2.setPower(gamepad2.left_stick_y);
        } else {
            hw.lift.setPower(0.01);
            hw.lift2.setPower(-0.01);

        }

    }

    private void intakeMove() {
        if(Math.abs(gamepad2.right_stick_y) > .1){
            hw.intake.setPower(gamepad2.right_stick_y);//gamepad1.right_trigger - gamepad1.left_trigger);
        }
        else{
            hw.intake.setPower(0);
        }
    }

    private void outakeMove() {

        if (gamepad2.left_bumper) {
            //hw.outtake1.setPosition(1);

        } else if (gamepad2.right_bumper) {
            //hw.outtake1.setPosition(0);
        }
    }

    private void dropper() {
       // telemetry.addData("lt: ", gamepad2.left_trigger);
        //telemetry.addData("rt: ", gamepad2.right_trigger);
        //telemetry.update();

        if(Math.abs(gamepad2.left_trigger - gamepad2.right_trigger) > .1){
            hw.dropper.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
        }
        else{
            hw.dropper.setPower(0);
        }
    }

    public void intakeServos() {
        //telemetry.addData("servo1: ", hw.intakeServo1.getPosition());
        //telemetry.addData("servo2: ", hw.intakeServo2.getPosition());
        if(!hw.intakeServo1.equals(0.00)){
            //telemetry.addData("not at 0", hw.intakeServo1.getPosition());
        }
        //telemetry.update();



        if (gamepad2.a) {
            hw.intakeServo1.setPosition(1);
            hw.intakeServo2.setPosition(.2);
            //hw.droneLauncher.setPosition(0);
        } else if (gamepad2.b) {
            //hw.intakeServo1.setPosition(0.5);
            //hw.intakeServo2.setPosition(0.51);
            hw.intakeServo1.setPosition(0.325);
            hw.intakeServo2.setPosition(.675);
        }
    }

    public void droneLaunch() {

        if (gamepad2.y) {
            hw.droneLauncher.setPosition(0.5);
        }
    }
}
