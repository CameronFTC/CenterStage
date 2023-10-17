package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "Red Left", group = "Autonomous")
public class RedLeftAuto extends LinearOpMode {
    private IMU imu;
    hwMap hw;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = new hwMap(this);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        imu.initialize(parameters);

        imu.resetYaw();

        telemetry.addLine("ready");
        telemetry.update();

        //vision

        waitForStart();

        while(opModeIsActive() && !isStopRequested())
        {
            imu.resetYaw();

            splineMovement(0.46, 0.04, 0.6, -90);

            sleep(1000);

            //adjust based on april tag
            strafe(-0.4, -840);

            //deposit
            sleep(500);

            goStraightPID(-1200, 0.005, 0.000138138, 0.0005, 5000, -1);

            sleep(500);

            diagonal(-0.25, -0.3, 530);

            sleep(300);

            strafe(0.4, 300);

            sleep(30000);
        }
    }

    //methods

    private void splineMovement(double yPwr, double xPwr, double rotation, double finalAngle)
    {
        double y = -yPwr;
        double x = xPwr;
        double rx = rotation;

        double kP = 1 / finalAngle;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while(Math.abs(heading - finalAngle) > 2)
        {
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = finalAngle - heading;

            double rotX = x * Math.cos(Math.toRadians(-heading)) - y * Math.sin(Math.toRadians(-heading));
            double rotY = x * Math.sin(Math.toRadians(-heading)) + y * Math.cos(Math.toRadians(-heading));

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double flPwr = (rotY + rotX + rx * (error * kP)) / denominator;
            double blPwr = (rotY - rotX + rx * (error * kP)) / denominator;
            double frPwr = (rotY - rotX - rx * (error * kP)) / denominator;
            double brPwr = (rotY + rotX - rx * (error * kP)) / denominator;

            hw.fL.setPower(-flPwr);
            hw.bL.setPower(-blPwr);
            hw.fR.setPower(-frPwr);
            hw.bR.setPower(-brPwr);
        }

        stopAll();
    }

    public void goStraightPID(double distance, double kP, double kI, double kD, double timeout, double max) {
        ElapsedTime timer = new ElapsedTime();
        resetEncoders();
        timer.startTime();

        double oldGyro = 0;
        double power;
        double start = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = 0;
        double currTime = timer.milliseconds();
        double LhAdjust = 0;
        double RhAdjust = 0;
        double integral = 0;
        double derivative;
        double proportional;
        double oldTime = currTime;
        double startStraight = getAvgEncoder();
        double travel = 0;


        while (Math.abs(distance) > Math.abs(travel) && !isStopRequested()) {
            travel = getAvgEncoder() - startStraight;
            currTime = timer.milliseconds();

            proportional = (distance - travel) * kP;
            integral += (travel - (getAvgEncoder() - startStraight)) * (currTime - oldTime) * kI;
            derivative = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - oldGyro) * kD;
            power = integral + proportional + derivative;

            error = getTrueDiff(start);

            RhAdjust = (error * .02);
            LhAdjust = -(error * .02);

            if(power < 0.15 && distance > 0){
                power = 0.2;
            }
            if(power > -0.15 && distance < 0){
                power = -0.2;
            }

            if(Math.abs(power) > Math.abs(max)){
                power = max;
            }
            oldTime = currTime;
            straight(power, RhAdjust, LhAdjust);

            /*telemetry.addData("Avg Encoder Val", getAvgEncoder());
            telemetry.addData("Gyro Error", error);
            telemetry.addData("Amount left", (distance - getAvgEncoder()));
            telemetry.addData("Forward power", power);
            telemetry.addData("Proportional", proportional);
            telemetry.addData("Integral", integral);
            telemetry.addData("Derivatve", derivative);
            telemetry.addData("Left power: ", LhAdjust);
            telemetry.addData("Right power: ", RhAdjust);
            telemetry.update();*/

            if (currTime > timeout) {
                break;
            }

            oldGyro = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        stopAll();
    }

    public double getAvgEncoder() {

        double div = 4;
        double avgAdd = hw.bL.getCurrentPosition() + hw.bR.getCurrentPosition() + hw.fL.getCurrentPosition() + hw.fR.getCurrentPosition();
        double avg;

        if (hw.bL.getCurrentPosition() == 0) {
            div--;
        }

        if (hw.bR.getCurrentPosition() == 0) {
            div--;
        }
        if (hw.fL.getCurrentPosition() == 0) {
            div--;
        }

        if (hw.fR.getCurrentPosition() == 0) {
            div--;
        }

        if (div == 0) {
            avg = 0;
        } else {
            avg = avgAdd / div;
        }

        return -avg;
    }

    public void resetEncoders() {

        hw.fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        hw.bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        hw.fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        hw.bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        hw.bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();
        hw.bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();
        hw.fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();
        hw.fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();
    }

    public double getTrueDiff(double destTurn){
        double currAng = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if((currAng >= 0 && destTurn >= 0) || (currAng <= 0 && destTurn <= 0))
            return destTurn - currAng;
        else if(Math.abs(destTurn - currAng) <= 180)
            return destTurn - currAng;

        else if(destTurn > currAng)
            return -(360 - (destTurn - currAng));
        else
            return 360 - (currAng - destTurn);

    }

    public void straight(double pwr, double RhAdj, double LhAdj) {
        double max = Math.max(Math.abs(pwr + LhAdj), Math.abs(pwr + RhAdj));
        double leftPwr = pwr + LhAdj;
        double rightPwr = pwr + RhAdj;

        if (max > 1) {
            leftPwr /= max;
            rightPwr /= max;
        }

        hw.fL.setPower(-leftPwr);
        hw.fR.setPower(-rightPwr);
        hw.bL.setPower(-leftPwr);
        hw.bR.setPower(-rightPwr);
    }

    public void stopAll() {
        double pwr = 0;
        hw.fL.setPower(pwr);
        hw.fR.setPower(pwr);
        hw.bL.setPower(pwr);
        hw.bR.setPower(pwr);
    }

    public void strafe(double pwr, double distance)
    {
        double startPos = hw.fL.getCurrentPosition();
        double currentPos = 0;

        while(Math.abs(currentPos - distance) > 10)
        {
            telemetry.addData("currPos: ", currentPos);
            telemetry.update();
            hw.fL.setPower(pwr);
            hw.bL.setPower(-pwr);
            hw.fR.setPower(-pwr);
            hw.bR.setPower(pwr);

            currentPos = hw.fL.getCurrentPosition() - startPos;
        }

        stopAll();
    }

    private void diagonal(double yPwr, double xPwr, double distance)
    {
        double y = -yPwr;
        double x = xPwr;
        double rx = 0;

        double kP = 1 / distance;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double startPos = hw.fR.getCurrentPosition();
        double currentPos = 0;

        while(Math.abs(currentPos - distance) > 5)
        {
            telemetry.addData("fR: ", currentPos);
            telemetry.update();
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = distance - currentPos;

            double rotX = x * Math.cos(Math.toRadians(-heading)) - y * Math.sin(Math.toRadians(-heading));
            double rotY = x * Math.sin(Math.toRadians(-heading)) + y * Math.cos(Math.toRadians(-heading));

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double flPwr = (rotY + rotX + rx * (error * kP)) / denominator;
            double blPwr = (rotY - rotX + rx * (error * kP)) / denominator;
            double frPwr = (rotY - rotX - rx * (error * kP)) / denominator;
            double brPwr = (rotY + rotX - rx * (error * kP)) / denominator;

            hw.fL.setPower(-flPwr);
            hw.bL.setPower(-blPwr);
            hw.fR.setPower(-frPwr);
            hw.bR.setPower(-brPwr);

            currentPos = hw.fR.getCurrentPosition() - startPos;
        }

        stopAll();
    }
}
