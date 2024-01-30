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
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 62.5, Math.toRadians(270)))

                                .forward(29)
                                .turn(Math.toRadians(90))
                                .forward(4)
                                .addTemporalMarker(() -> { })
                                .back(4)
                                .addTemporalMarker(() -> { })

                                .setTangent(Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-55, 11.6, Math.toRadians(0)), Math.toRadians(0))


                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueRight)
                .start();

    }

}