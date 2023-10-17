package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class hwMap {

    public DcMotor bR;
    public DcMotor fR;
    public DcMotor fL;
    public DcMotor bL;
    public DcMotor lift;
    public DcMotor lift2;

    public Servo tilt;
    public CRServo dropper;

    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    public Orientation startPos = null;
    public double lastDegrees;
    public double globalAngle;
    public double referenceAngle;
    public double initAngle;
    public double rotations;

    public double liftEncoderGlobal;

    LinearOpMode opmode;
    ElapsedTime runtime = new ElapsedTime();

    public hwMap(LinearOpMode opmode) {
        this.opmode = opmode;
        fL = opmode.hardwareMap.get(DcMotor.class, "fL");
        fR = opmode.hardwareMap.get(DcMotor.class, "fR");
        bL = opmode.hardwareMap.get(DcMotor.class, "bL");
        bR = opmode.hardwareMap.get(DcMotor.class, "bR");

        //lift = opmode.hardwareMap.get(DcMotor.class, "lift");
        //lift2 = opmode.hardwareMap.get(DcMotor.class, "lift2");

        liftEncoderGlobal = 0;

        //tilt = opmode.hardwareMap.get(Servo.class, "tilt");
        //dropper = opmode.hardwareMap.get(CRServo.class, "dropper");

        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //fR.setDirection(DcMotorSimple.Direction.REVERSE);
        //lift.setDirection(DcMotorSimple.Direction.REVERSE);
        //lift2.setDirection(DcMotorSimple.Direction.REVERSE);

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        //imu = opmode.hardwareMap.get(BNO055IMU.class, "imu");

        //imu.initialize(parameters);

        //opmode.telemetry.addData("Mode", "calibrating...");
        //opmode.telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        //while (!opmode.isStopRequested() && !imu.isGyroCalibrated()) {
        //    opmode.sleep(50);
        //    opmode.idle();
        //}

        //opmode.telemetry.addData("Mode", "waiting for start");
        //opmode.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        //opmode.telemetry.update();

    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;

        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        if(globalAngle > 360)
        {
            globalAngle -= 360;
        }
        else if(globalAngle < -360)
        {
            globalAngle += 360;
        }

        return -globalAngle;
    }

    public void lift(double distance, double pwr, double kP){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double error;
        while(Math.abs(distance - lift.getCurrentPosition()) > 30){
            error = distance - lift.getCurrentPosition();
            lift.setPower(error * pwr * kP);
            lift2.setPower(error * pwr * kP);
        }
        lift.setPower(0);
        lift2.setPower(0);
    }

    public double teleOpgetAngle(boolean clockwise) {
        //newAngle = if angle is negative, add 360+angles, if angle positive, angles
        //numRotations = Math.floor(newAngle/359) if coming clockwise direction do this
        //if coming from counterclockwise direction numRotations = -Math.floor(newAngle/359)

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        double currentAngle = angles.firstAngle;
        double returnAngle = 0;
        int numRotations = 0;

        //if(clockwise){
        if(currentAngle < 0){
            returnAngle = currentAngle + 360;
        } else {
            returnAngle = currentAngle;
        }

        if (clockwise)
            numRotations += Math.floor(returnAngle/359.0);
        else
            numRotations -= Math.floor(returnAngle/359.0);

        return returnAngle + (numRotations * 360);

        //double currAng = getCurrGyro();
        //if((currAng >= 0) && destTurn >= );

        /*} else {
            if(currentAngle > 0){
                returnAngle = currentAngle - 360;
            } else {
                returnAngle = currentAngle;
            }*/

    }

    public double getTrueDiff(double destTurn){
        double currAng = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;

        if((currAng >= 0 && destTurn >= 0) || (currAng <= 0 && destTurn <= 0))
            return destTurn - currAng;
        else if(Math.abs(destTurn - currAng) <= 180)
            return destTurn - currAng;

        else if(destTurn > currAng)
            return -(360 - (destTurn - currAng));
        else
            return 360 - (currAng - destTurn);

    }

    public void turnPID(double pwr, double targetAngle, double kP, double kI, double kD)
    {
        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        targetAngle -= startAngle;
        ElapsedTime runtime = new ElapsedTime();
        while(Math.abs(targetAngle) <= 1)
        {
            double time1 = runtime.seconds();
            double error = targetAngle - getAngle();
            double time2 = runtime.seconds();
            double deltaTime = time2 - time1;

            double proportional = error * pwr * kP;
            double integral = error * deltaTime * kI;
            double derivative = error / deltaTime * kD;

            fL.setPower(proportional + integral + derivative);
            bL.setPower(proportional + integral + derivative);
            fR.setPower(-(proportional + integral + derivative));
            bR.setPower(-(proportional + integral + derivative));
        }

        fL.setPower(0);
        bL.setPower(0);
        fR.setPower(0);
        bR.setPower(0);
    }

    public double getAvgEncoder() {

        double div = 4;
        double avgAdd = bL.getCurrentPosition() + bR.getCurrentPosition() + fL.getCurrentPosition() + fR.getCurrentPosition();
        double avg;

        if (bL.getCurrentPosition() == 0) {
            div--;
        }

        if (bR.getCurrentPosition() == 0) {
            div--;
        }
        if (fL.getCurrentPosition() == 0) {
            div--;
        }

        if (fR.getCurrentPosition() == 0) {
            div--;
        }

        if (div == 0) {
            avg = 0;
        } else {
            avg = avgAdd / div;
        }

        return -avg;
    }

    public double getAvgEncoderStrafe() {

        double div = 4;
        double avgAdd = bL.getCurrentPosition() - bR.getCurrentPosition() -  fL.getCurrentPosition() + fR.getCurrentPosition();
        double avg;

        if (bL.getCurrentPosition() == 0) {
            div--;
        }

        if (bR.getCurrentPosition() == 0) {
            div--;
        }
        if (fL.getCurrentPosition() == 0) {
            div--;
        }

        if (fR.getCurrentPosition() == 0) {
            div--;
        }

        if (div == 0) {
            avg = 0;
        } else {
            avg = avgAdd / div;
        }

        return -avg;
    }

    public void straight(double pwr, double RhAdj, double LhAdj) {

        double max = Math.max(Math.abs(pwr + LhAdj), Math.abs(pwr + RhAdj));
        double leftPwr = pwr + LhAdj;
        double rightPwr = pwr + RhAdj;

        if (max > 1) {
            leftPwr /= max;
            rightPwr /= max;
        }

        fL.setPower(-leftPwr);
        fR.setPower(-rightPwr);
        bL.setPower(-leftPwr);
        bR.setPower(-rightPwr);
    }

    public void strafe(double pwr, double RhAdj, double LhAdj) {

        double max = Math.max(Math.abs(pwr + LhAdj), Math.abs(pwr + RhAdj));
        double leftPwr = pwr + LhAdj;
        double rightPwr = pwr + RhAdj;

        if (max > 1) {
            leftPwr /= max;
            rightPwr /= max;
        }

        fL.setPower(-leftPwr);
        fR.setPower(rightPwr);
        bL.setPower(leftPwr);
        bR.setPower(-rightPwr);
    }

    public void stopAll() {
        double pwr = 0;
        fL.setPower(pwr);
        fR.setPower(pwr);
        bL.setPower(pwr);
        bR.setPower(pwr);

    }

    public void goStraightPID(double distance, double kP, double kI, double kD, double timeout, double max) {
        ElapsedTime timer = new ElapsedTime();
        resetEncoders();
        timer.startTime();

        double oldGyro = 0;
        double power;
        double start = getAngle();
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


        while (Math.abs(distance) > Math.abs(travel) && !opmode.isStopRequested()) {
            travel = getAvgEncoder() - startStraight;
            currTime = timer.milliseconds();

            proportional = (distance - travel) * kP;
            integral += (travel - (getAvgEncoder() - startStraight)) * (currTime - oldTime) * kI;
            derivative = (getAngle() - oldGyro) * kD;
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

            opmode.telemetry.addData("Avg Encoder Val", getAvgEncoder());
            opmode.telemetry.addData("Gyro Error", error);
            opmode.telemetry.addData("Amount left", (distance - getAvgEncoder()));
            opmode.telemetry.addData("Forward power", power);
            opmode.telemetry.addData("Proportional", proportional);
            opmode.telemetry.addData("Integral", integral);
            opmode.telemetry.addData("Derivatve", derivative);
            opmode.telemetry.addData("Left power: ", LhAdjust);
            opmode.telemetry.addData("Right power: ", RhAdjust);
            opmode.telemetry.update();

            if (currTime > timeout) {
                break;
            }

            oldGyro = getAngle();
        }


        stopAll();
    }

    public void goStrafePID(double distance, double kP, double kI, double kD, double timeout, double max) {
        ElapsedTime timer = new ElapsedTime();
        resetEncoders();
        timer.startTime();

        double power;
        double start = getAngle();
        double error = 0;
        double currTime = timer.milliseconds();
        double LhAdjust = 0;
        double RhAdjust = 0;
        double integral = 0;
        double derivative;
        double proportional;
        double oldTime = currTime;
        double startPlace = getAvgEncoder();
        double travel = 0;

        while (Math.abs(distance) > Math.abs(travel) && !opmode.isStopRequested()) {
            travel = getAvgEncoderStrafe() - startPlace;
            currTime = timer.milliseconds();

            proportional = (distance - travel) * kP;
            integral += (travel - (getAvgEncoderStrafe() - startPlace)) * (currTime - oldTime) * kI;
            derivative = getTrueDiff(start) * kD;
            power = integral + proportional + derivative;

            error = getTrueDiff(start);

            RhAdjust = (error * .025);
            LhAdjust = -(error * .025);

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
            strafe(power, RhAdjust, LhAdjust);

            opmode.telemetry.addData("Avg Encoder Val", getAvgEncoderStrafe());
            opmode.telemetry.addData("Gyro Error", error);
            opmode.telemetry.addData("Amount left", (distance - getAvgEncoderStrafe()));
            opmode.telemetry.addData("Forward power", power);
            opmode.telemetry.addData("Proportional", proportional);
            opmode.telemetry.addData("Integral", integral);
            opmode.telemetry.addData("Derivatve", derivative);
            opmode.telemetry.addData("Left power: ", LhAdjust);
            opmode.telemetry.addData("Right power: ", RhAdjust);
            opmode.telemetry.update();

            if (currTime > timeout) {
                break;
            }
        }

        stopAll();
    }

    public void resetEncoders() {

        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();

        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opmode.idle();
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opmode.idle();
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opmode.idle();
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opmode.idle();

    }
}
