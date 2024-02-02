package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.OpenCV.CVMaster;
import org.firstinspires.ftc.teamcode.OpenCV.StickObserverPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Bluefar", group = "Autonomous")
public class Bluefar extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//    }

    //@Autonomous(name = "Blue far", group = "Autonomous")
    //public class blueFar extends LinearOpMode {
    //dah[d

    private IMU imu;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    hwMap hw;
    double angle;
    int counter = 0;
    double startAngle;
    double heading;
    double liftStart;
    double liftPos = 0;

    double slidePos = 0;
    boolean goNext = false;


    final int DESIRED_DISTANCE = 3;
    int DESIRED_TAG_ID = 4;
    final double SPEED_GAIN = 0;   //  0.02 Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = -0.1;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0;   //  0.01 Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    AprilTag april;
    boolean targetFound;
    double drive1;
    double strafe;
    double turn;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = (new Pose2d(-35, 62, Math.toRadians(270)));
        ElapsedTime timer = new ElapsedTime();
        Vector2d myVector = new Vector2d(-35, 62);


        mecanumDrive.setPoseEstimate(startPose);


        TrajectorySequence trajl1 = mecanumDrive.trajectorySequenceBuilder(startPose)
//<<<<<<< HEAD
                .lineTo(new Vector2d(-35,32))
                .turn(Math.toRadians(-90))
                .build();
        TrajectorySequence trajl2 = mecanumDrive.trajectorySequenceBuilder(trajl1.end())
                .lineTo(new Vector2d(-40,32))


//                .strafeLeft(1)
                .build();

//=======
//                .splineTo(new Vector2d(-15, 5), Math.toRadians(0))
//                //.turn(Math.toRadians(-90))
//>>>>>>> 1e53bf7b1dc6074257ebf4c4debc58b442740c6e
                //.forward(5)

//ifright
//            TrajectorySequence trajright = mecanumDrive.trajectorySequenceBuilder(traj1.end())
//                    .turn(Math.toRadians(90))
//                    .build();
//
////ifmiddle
//            TrajectorySequence trajmiddle = mecanumDrive.trajectorySequenceBuilder(traj1.end())
//                    .forward(18)
//                    .build();
////ifleft
//            TrajectorySequence trajleft = mecanumDrive.trajectorySequenceBuilder(traj1.end())
//                    .turn(Math.toRadians(-90))
//                    .build();

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

        //vision
        String pos = "Middle";

        CVMaster cv = new CVMaster(this, "Blue");
//      call the function to startStreaming
        cv.observeStick();
        counter = 0;
        while (!isStarted()) {
            telemetry.addData("Coords: ", StickObserverPipeline.xCoord + " " + StickObserverPipeline.yCoord);
            telemetry.addData("Area: ", StickObserverPipeline.maxContour);
            telemetry.addData("Pos: ", pos);
            telemetry.update();

            double TSE = StickObserverPipeline.xCoord;
            //pos = "Right";
            if (StickObserverPipeline.maxContour < 500) {
                pos = "Left";

            } else if (TSE >= 0 && TSE < 300) {
                pos = "Middle";

            } else if (TSE >= 150) {
                pos = "Right" ;

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
        visionPortal = builder.build();

         */

        RobotOrientation.red = false;
        hw.droneLauncher.setPosition(0.65);
        intakeUp();

        waitForStart();

        imu.resetYaw();
        cv.stopCamera();
        //currState = State.spike;

        april = new AprilTag(this);

        //pos = "Middle";
        //currState = State.backboard;
        //while (opModeIsActive() && !isStopRequested()) {
        if (!isStopRequested()) {
            //imu.resetYaw();

            slidePos = 0;
            heading = hw.getAngle();
            RobotOrientation.angle = heading;


            //telemetry.addData("heading: ", heading);
            //telemetry.update();

            if (pos.equals("Left")) {

                mecanumDrive.followTrajectorySequence(trajl1);
                hw.autoIntake(-1, 1);
               mecanumDrive.followTrajectorySequence(trajl2);


//                sleep(1000);
//>>>>>>> 1e53bf7b1dc6074257ebf4c4debc58b442740c6e
//                hw.intake.setPower(0);
                //hi
            }


        }


    }
    //methods

    private void splineMovement(double yPwr, double xPwr, double rotation, double finalAngle, double leniency) {
        double y = -yPwr;
        double x = xPwr;
        double rx = rotation;

        double kP = 1.75 / finalAngle;

        //double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (Math.abs(heading - finalAngle) > leniency) {
            //heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            telemetry.addData("Heading: ", heading);
            telemetry.update();

            double error = finalAngle - heading;

            y *= (error * kP);
            //x *= (error * kP);

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
        } else {
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

            if (power < 0.15 && distance > 0) {
                power = 0.2;
            }
            if (power > -0.15 && distance < 0) {
                power = -0.2;
            }

            if (Math.abs(power) > Math.abs(max)) {
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

    public double getTrueDiff(double destTurn) {
        double currAng = getAngle();

        if ((currAng >= 0 && destTurn >= 0) || (currAng <= 0 && destTurn <= 0))
            return destTurn - currAng;
        else if (Math.abs(destTurn - currAng) <= 180)
            return destTurn - currAng;

        else if (destTurn > currAng)
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

    public double getAngle() {
        angle = hw.angle() + startAngle;
        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }

        return angle;
    }

    public void setLift(double target, double pwr) {
        double currPos = getLiftPos();

        double kP = 1 / (target - slidePos);

        if (Math.abs(target - currPos) > 75) {
            double error = target - currPos;
            hw.lift2.setPower(pwr * kP * (error) * 2);
            hw.lift.setPower(pwr * kP * (error) * 2);
        } else {
            hw.lift2.setPower(0);
            hw.lift.setPower(0);
        }

        //telemetry.addData("lift: ", currPos);
        //telemetry.update();
    }

    public double getLiftPos() {
        double deltaLift = hw.lift2.getCurrentPosition() - liftStart;
        liftPos = deltaLift;

        return liftPos;
    }

    public void intakeUp() {
        hw.intakeServo1.setPosition(1);
        hw.intakeServo2.setPosition(0);
    }

    public void intakeDown() {
        hw.intakeServo1.setPosition(0.49);
        hw.intakeServo2.setPosition(0.49);
    }

    public void outtakeExtend() {
        hw.outtake1.setPosition(1);

    }

    public void outtakeRetract() {
        hw.outtake1.setPosition(0);

    }

    public void aprilTagAdjust() {
        ElapsedTime timer = new ElapsedTime();
        double runtime = timer.seconds();

        while (runtime < 2) {
            april.telemetryAprilTag();
            telemetry.update();

            targetFound = false;
            desiredTag = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = april.getAprilTag().getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null)
                        && ((DESIRED_TAG_ID >= 0) || (detection.id == DESIRED_TAG_ID))) {
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData(">", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData(">", "Drive using joystick to find target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing * -1;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive1 = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive1, strafe, turn);
            } else {
                drive1 = 0;
                strafe = 0;
                turn = 0;
            }

            moveRobot(drive1, strafe, turn);
            telemetry.update();

            runtime = timer.seconds();
        }
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

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

    public void strafe(double pwr, double distance) {
        double startPos = hw.fL.getCurrentPosition();
        double currentPos = 0;

        while (Math.abs(currentPos - distance) > 10) {
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
}

