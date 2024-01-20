package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.OpenCV.CVMaster;
import org.firstinspires.ftc.teamcode.OpenCV.ConceptAprilTag;
import org.firstinspires.ftc.teamcode.OpenCV.StickObserverPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Disabled
@Autonomous(name = "Red Right", group = "Autonomous")
public class Redfar extends LinearOpMode {
    private IMU imu;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    hwMap hw;
    double angle;
    double startAngle;
    double heading;
    double liftStart;
    double liftPos = 0;

    double slidePos = 0;
    boolean goNext = false;
    boolean aprilFound = true;

    enum State {
        spike,
        backboard,
        park,
        april,
    }

    final int DESIRED_DISTANCE = 3;
    int DESIRED_TAG_ID = 4;
    final double SPEED_GAIN  =  0.02  ;   //  0.02 Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.4  ;   //  0.01 Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    private State currState = State.backboard;
    AprilTag april;
    boolean targetFound;
    double drive;
    double strafe;
    double turn;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = new hwMap(this);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        imu.initialize(parameters);

        telemetry.addLine("ready");
        telemetry.update();

        imu.resetYaw();
        angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        startAngle = angle;

        liftStart = hw.lift2.getCurrentPosition();

        intakeUp();

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

            if(TSE > 0 && TSE < 50)
            {
                pos = "Left";
                currState = State.spike;
                DESIRED_TAG_ID = 4;
            }
            else if(TSE >= 50 && TSE < 400)
            {
                pos = "Middle";
                currState = State.spike;
                DESIRED_TAG_ID = 5;
            }
            else if(TSE >= 400)
            {
                pos = "Right";
                currState = State.spike;
                DESIRED_TAG_ID = 6;
            }
        }


        //cv.stopCamera();

        /*aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));

        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();*/

        targetFound     = false;    // Set to true when an AprilTag target is detected
        drive           = 0;        // Desired forward power/speed (-1 to +1)
        strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        turn            = 0;        // Desired turning power/speed (-1 to +1)

        waitForStart();

        imu.resetYaw();
        cv.stopCamera();

        april = new AprilTag(this);

        pos = "Middle";
        DESIRED_TAG_ID = 5;
        //currState = State.spike;
        while(opModeIsActive() && !isStopRequested())
        {
            //imu.resetYaw();

            april.telemetryAprilTag();
            telemetry.update();

            slidePos = 0;
            heading = hw.getAngle();
            RobotOrientation.angle = heading;

            //telemetry.addData("heading: ", heading);
            //telemetry.update();

            if(pos.equals("Middle"))
            {
                switch(currState)
                {
                    case backboard:
                        goNext = false;
                        //splineMovement(0.48, -0.42, 0.6, -91, 1);
                        goStraightPID(-200, 0.01, 0.000138138, 0.005, 4000, -0.5);

                        goNext = true;

                        if(goNext)
                        {
                            outtakeExtend();
                            currState = State.april;
                        }
                        break;

                    case spike:
                        goNext = false;
                        //splineMovement(-0.05, 0.68, 0.2, 180, 2);
                        //setLift(0, 0.7);
                        splineMovement(0.425, -0.05, 0.63, -115, 3);

                        slidePos = 0;

                        if(goNext)
                        {
                            hw.autoIntake(-1, 1);
                            currState = State.backboard;
                        }
                        break;

                    case april:
                        goNext = false;

                        //setLift(-500, -0.7);

                        aprilTagAdjust();

                        slidePos = 500;

                        if(goNext)
                        {
                            goStraightPID(-300, 0.01, 0.000138138, 0.005, 4000, -0.5);
                            hw.liftTimer(-0.7, 1);

                            currState = State.park;
                        }
                        break;

                    case park:
                        //hw.autoIntake(-1, 5);
                        splineMovement(0, -0.03, -0.1, -89, 1);
                        //hw.autoDrop(1, 1);

                        //outtakeRetract();
                        sleep(30000);
                        break;
                }
            }
            else if(pos.equals("Right"))
            {
                switch(currState)
                {
                    case backboard:
                        splineMovement(0.27, -0.34, 0.6, -91, 1);
                        setLift(-500, -0.7);

                        slidePos = 500;

                        if(goNext)
                        {
                            sleep(2000);
                            currState = State.spike;
                        }
                        break;

                    case spike:
                        goNext = false;
                        splineMovement(0, 0.54, 0.2, 180, 2);
                        setLift(0, 0.7);

                        slidePos = 0;

                        if(goNext)
                        {
                            sleep(1000);
                            currState = State.park;
                        }
                        break;

                    case park:
                        //hw.autoIntake(-1, 5);
                        sleep(30000);
                }
            }
            else if(pos.equals("Left"))
            {
                switch(currState)
                {
                    case spike:
                        splineMovement(0.53, 0.13, 0.6, -91, 1);
                        setLift(-500, -0.7);

                        slidePos = 500;

                        telemetry.addData("heading: ", heading);
                        telemetry.update();

                        if(goNext)
                        {
                            sleep(1000);
                            currState = State.backboard;
                        }
                        break;

                    case backboard:
                        goNext = false;
                        goStraightPID(-1060, 0.01, 0.000138138, 0.005, 4000, -0.5);
                        goNext = true;

                        if(goNext)
                        {
                            sleep(1000);
                            currState = State.park;
                        }
                        break;

                    case park:
                        //hw.autoIntake(-1, 5);
                        sleep(30000);
                }
            }

            //adjust based on april tag

            //goStraightPID(-110, 1 / 110, 0.000138138, 0.0005, 2000, -0.6);

            //deposit

            //sleep(30000);
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

            telemetry.addData("Avg Encoder Val", getAvgEncoder());
            telemetry.addData("Gyro Error", error);
            telemetry.addData("Amount left", (distance - getAvgEncoder()));
            telemetry.addData("Forward power", power);
            telemetry.addData("Proportional", proportional);
            telemetry.addData("Integral", integral);
            telemetry.addData("Derivatve", derivative);
            telemetry.addData("Left power: ", LhAdjust);
            telemetry.addData("Right power: ", RhAdjust);
            telemetry.update();

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
        double currAng = getAngle();

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
        double deltaLift = hw.lift2.getCurrentPosition() - liftStart;
        liftPos = deltaLift;

        return liftPos;
    }

    public void aprilTagAdjust()
    {
        april.telemetryAprilTag();
        telemetry.update();

        targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = april.getAprilTag().getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null)
                    && ((DESIRED_TAG_ID >= 0) || (detection.id == DESIRED_TAG_ID))  ){
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData(">","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData(">","Drive using joystick to find target\n");
        }

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing * -1;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
        else
        {
            drive = 0;
            strafe = 0;
            turn = 0;

            goNext = true;
        }

        moveRobot(drive, strafe, turn);
        telemetry.update();
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        hw.fL.setPower(leftFrontPower);
        hw.fR.setPower(rightFrontPower);
        hw.bL.setPower(leftBackPower);
        hw.bR.setPower(rightBackPower);
    }

    public void intakeUp()
    {
        hw.intakeServo1.setPosition(1);
        hw.intakeServo2.setPosition(0);
    }

    public void intakeDown()
    {
        hw.intakeServo1.setPosition(0.49);
        hw.intakeServo2.setPosition(0.49);
    }

    public void outtakeExtend()
    {
        hw.outtake1.setPosition(1);

    }

    public void outtakeRetract()
    {
        hw.outtake1.setPosition(0);
    }
}
