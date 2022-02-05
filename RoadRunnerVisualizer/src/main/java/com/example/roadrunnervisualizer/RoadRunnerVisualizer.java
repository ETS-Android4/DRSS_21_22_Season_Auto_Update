package com.example.roadrunnervisualizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RoadRunnerVisualizer {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.4829, 52.4829, Math.toRadians(273.36816), Math.toRadians(273.36816), 12)
                .setDimensions(13,17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(8.875, 62.5, Math.toRadians(180.0)))
                        .lineToLinearHeading(new Pose2d(-20, 45, Math.toRadians(110.0)))
                        .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(90.0)), Math.toRadians(90.0))
                        .back(10)
                        .splineToLinearHeading(new Pose2d(-30, 62.5, Math.toRadians(0.0)), Math.toRadians(90.0))
                        .splineTo(new Vector2d(48, 64), Math.toRadians(0))
                        .back(40)
                        .splineTo(new Vector2d(-12, 45), Math.toRadians(270.0))
                        .splineToLinearHeading(new Pose2d(8, 64, Math.toRadians(0.0)), Math.toRadians(90))
                        .forward(40)
                        .back(40)
                        .splineTo(new Vector2d(-12, 45), Math.toRadians(270.0))
                        .splineToLinearHeading(new Pose2d(8, 64, Math.toRadians(0.0)), Math.toRadians(90))
                        .forward(40)
                        .splineToLinearHeading(new Pose2d(42, 45, Math.toRadians(0.0)), Math.toRadians(0.0))
                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}