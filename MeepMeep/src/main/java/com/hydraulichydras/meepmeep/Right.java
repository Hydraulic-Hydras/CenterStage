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
                .setConstraints(74, 74, Math.toRadians(180), Math.toRadians(180), 16.5)
                .setDimensions(17.5,17.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -62.5, Math.toRadians(90)))

                                /* === RIGHT === */

                                .lineToLinearHeading(new Pose2d(12, -29, Math.toRadians(180)))
                                .waitSeconds(1)

                                .setReversed(true)

                                // .splineToConstantHeading(new Vector2d(48, -35), Math.toRadians(0))
                                .lineTo(new Vector2d(48, -29))
                                .waitSeconds(1)

                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(35, -58.5), Math.toRadians(180))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(-35, -58.5))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(-58, -35.45), Math.toRadians(180))
                                .waitSeconds(1)

                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-35,-58.5), Math.toRadians(0))
                                .lineTo(new Vector2d(35,-58.5))

                                // left, right middle blah blah

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueRight)
                .start();

    }
}