package com.hydraulichydras.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Right {

    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(800);

        RoadRunnerBotEntity blueRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(229), Math.toRadians(229), 17.67)
                .setDimensions(17.5,17.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12.5, -62, Math.toRadians(90)))


                                .lineTo(new Vector2d(12.5, -33))
                                .back(5)
                                .turn(Math.toRadians(90))

                                .lineTo(new Vector2d(43.7, -38))

                                .waitSeconds(0.5)
                                .splineToConstantHeading(new Vector2d(35.5, -7), Math.toRadians(180))
                                .lineTo(new Vector2d(-31.3, -7))

                                .splineToConstantHeading(new Vector2d(-55, -11.6), Math.toRadians(180))
                                .forward(1.2)

                                .lineToLinearHeading(new Pose2d(35, -11, Math.toRadians(180)))
                                .waitSeconds(0.5)

                                .splineToConstantHeading(new Vector2d(43.7, -35), Math.toRadians(180))

                                .waitSeconds(0.5)
                                .splineToConstantHeading(new Vector2d(35.5, -7), Math.toRadians(180))
                                .lineTo(new Vector2d(-31.3, -7))

                                .splineToConstantHeading(new Vector2d(-55, -11.6), Math.toRadians(180))
                                .forward(1.2)

                                .lineToLinearHeading(new Pose2d(35, -11, Math.toRadians(180)))
                                .waitSeconds(0.5)

                                .splineToConstantHeading(new Vector2d(43.7, -35), Math.toRadians(180))

                                .waitSeconds(5.02)

                                // 2 + 2 uses 15.02 extra seconds
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueRight)
                .start();

    }

}