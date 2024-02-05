package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest{
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-35, 62, Math.toRadians(270)))
                                        .lineTo(new Vector2d(-35,35))
                                        .turn(Math.toRadians(180))
                                        .waitSeconds(1)
                                        .lineTo(new Vector2d(-50,35))
                                        .lineTo(new Vector2d(-50,10))
                                        .turn(Math.toRadians(-90))
                                        .lineTo(new Vector2d(50, 10))
                                        .lineTo(new Vector2d(50,35))

                                        .lineTo(new Vector2d(50,50))
                                        .turn(Math.toRadians(-90))

                                        //intake
        //                                .strafeLeft(25)
        //                                .back(30)
        //                                .lineToLinearHeading(new Pose2d(47, 38, Math.toRadians(-90)))
        //                                .turn(Math.toRadians(90))
        //                                .strafeLeft(5)
                                        //outtake
                                        //.strafeLeft(10)
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}