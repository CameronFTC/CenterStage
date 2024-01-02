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
import org.firstinspires.ftc.teamcode.OpenCV.CVMaster;
import org.firstinspires.ftc.teamcode.OpenCV.StickObserverPipeline;

@Autonomous(name = "Red Left", group = "Autonomous")
public class RedLeftAuto extends LinearOpMode {
    private IMU imu;
    hwMap hw;
    double angle;
    double startAngle;
    double heading;
    double liftStart;
    double liftPos = 0;
    double currentPos = 0;
    double startPos = 0;

    double slidePos = 0;
    boolean goNext = false;
    enum State {
        spike,
        backboard,
        park,
        intermission,
        rightOnly,
    }

    public State currState = State.spike;

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
        angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        startAngle = angle;

        telemetry.addLine("ready");
        telemetry.update();

        imu.resetYaw();
        angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        startAngle = angle;

        liftStart = hw.lift2.getCurrentPosition();

        //vision
        String pos = "Middle";

        CVMaster cv = new CVMaster(this, "Red");
//      call the function to startStreaming
        cv.observeStick();

        while(!isStarted())
        {
            telemetry.addData("Coords: ", StickObserverPipeline.xCoord + " " + StickObserverPipeline.yCoord);
            telemetry.addData("Pos: ", pos);
            telemetry.update();

            double TSE = StickObserverPipeline.xCoord;

            if(TSE > 0 && TSE < 150)
            {
                pos = "Left";
            }
            else if(TSE >= 150 && TSE < 400)
            {
                pos = "Middle";
            }
            else if(TSE >= 400)
            {
                pos = "Right";
            }
        }

        waitForStart();
        imu.resetYaw();

        while(opModeIsActive() && !isStopRequested())
        {
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            switch(currState)
            {
                case spike:
                    if(pos.equals("Right"))
                    {
                        splineMovement(0.46, -0.07, -0.6, 91,1);
                    }
                    else if(pos.equals("Middle"))
                    {
                        goStraightPID(-1400, 0.01, 0.000138138, 0.005, 3000, -0.6);

                        goNext = true;
                    }
                    else
                    {
                        splineMovement(0.46, 0.04, 0.6, -91,1);
                        telemetry.addData("heading: ", heading);

                        slidePos = 500;
                    }

                    if(goNext)
                    {
                        sleep(1000);
                        currState = State.intermission;
                    }
                    break;

                case intermission:
                    goNext = false;

                    if(pos.equals("Right"))
                    {
                        splineMovement(0.29, 0.05, 0.6, -91, 0.5);
                        //setLift(-500, -0.7);

                        slidePos = 500;
                    }
                    else if(pos.equals("Middle"))
                    {
                        splineMovement(0, -0.8, 0.3, -91, 0.5);
                    }
                    else
                    {
                        //hw.autoIntake(-1, 1);
                        strafe(-0.4, -830);
                        sleep(500);
                        goStraightPID(-2100, 0.005, 0.0000138138, 0.00005, 5000, -0.6);
                        sleep(500);

                        goNext = true;
                    }

                    if(goNext)
                    {
                        currState = State.backboard;
                    }
                    break;

                case backboard:
                    goNext = false;

                    if(pos.equals("Right"))
                    {
                        goStraightPID(-2200, 0.005, 0.0000138138, 0.00005, 5000, -0.6);

                        goNext = true;
                    }
                    else if(pos.equals("Middle"))
                    {
                        sleep(30000);
                    }
                    else
                    {
                        //setLift(-500, -0.7);
                        diagonal(-0.4, 0, -590);
                    }

                    if(goNext && !pos.equals("Right"))
                    {
                        sleep(1000);
                        currState = State.park;
                    }
                    else if(goNext && pos.equals("Right"))
                    {
                        sleep(1000);
                        currState = State.rightOnly;
                    }
                    break;

                case rightOnly:
                    goNext = false;
                    diagonal(-0.4, 0, -1150);

                    if(goNext)
                    {
                        currState = State.park;
                    }
                    break;

                case park:
                    goStraightPID(-420, 0.01, 0.000138138, 0.005, 2000, -0.3);
                    sleep(30000);
            }
        }
    }

    //methods

    private void splineMovement(double yPwr, double xPwr, double rotation, double finalAngle, double leniency)
    {
        double y = -yPwr;
        double x = xPwr;
        double rx = rotation;

        double kP = 1 / finalAngle;

        //double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if(Math.abs(heading - finalAngle) > leniency)
        {
            //heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

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
        else
        {
            goNext = true;
            stopAll();
        }
    }

    public void goStraightPID(double distance, double kP, double kI, double kD, double timeout, double max) {
        ElapsedTime timer = new ElapsedTime();
        resetEncoders();
        timer.startTime();

        double oldGyro = 0;
        double power;
        double start = hw.angle();
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

            oldGyro = getAngle();
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
        double currAng = hw.angle();

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
        double leftPwr = pwr;
        double rightPwr = pwr;

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
            hw.bL.setPower(-pwr * 1.08);
            hw.fR.setPower(-pwr);
            hw.bR.setPower(pwr * 1.08);

            currentPos = hw.fL.getCurrentPosition() - startPos;
        }

        stopAll();
    }

    boolean start = false;
    private void diagonal(double yPwr, double xPwr, double distance)
    {
        double y = -yPwr;
        double x = xPwr;
        double rx = 0;

        double kP = 1 / distance;

        if(!start)
        {
            startPos = hw.fR.getCurrentPosition();
        }

        start = true;
        //double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        currentPos = hw.fR.getCurrentPosition() - startPos;

        if(Math.abs(currentPos - distance) > 30)
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
            hw.bL.setPower(-blPwr * 1.05);
            hw.fR.setPower(-frPwr);
            hw.bR.setPower(-brPwr * 1.05);
        }
        else
        {
            goNext = true;
            stopAll();
        }
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

    public void setLift(double target, double pwr)
    {
        double currPos = getLiftPos();

        double kP = 1 / (target - slidePos);

        if(Math.abs(target - currPos) > 75)
        {
            double error = target - currPos;
            hw.lift2.setPower(pwr * kP * (error));
            hw.lift.setPower(pwr * kP * (error));
        }
        else
        {
            hw.lift2.setPower(0);
            hw.lift.setPower(0);
        }

        telemetry.addData("lift: ", currPos);
        telemetry.update();
    }

    public double getLiftPos()
    {
        double deltaLift = (hw.lift2.getCurrentPosition() + hw.lift.getCurrentPosition())  / 2 - liftStart;
        liftPos = deltaLift;

        return liftPos;
    }
}
