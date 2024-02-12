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
                .setConstraints(100, 50, Math.toRadians(229), Math.toRadians(229), 17.67)
                .setDimensions(17.5,17.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12.5, -62.5, Math.toRadians(90)))

                                // Right Prop
                                /*
                                .splineToConstantHeading(new Vector2d(22.5, -38), Math.toRadians(90))
                                .waitSeconds(0.1)

                                .lineTo(new Vector2d(22.5, -45))
                                .splineTo(new Vector2d(49, -43.5), Math.toRadians(0))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(45, -40))
                                .lineToLinearHeading(new Pose2d(45, -59))

                                .lineTo(new Vector2d(50, -59))

                                 */

                                // Center Prop
                                /*
                                .lineTo(new Vector2d(12.5, -33.5))
                                .waitSeconds(0.1)

                                .lineTo(new Vector2d(12.5, -40))
                                .splineTo(new Vector2d(49, -37.5), Math.toRadians(0))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(45, -37.5))
                                .lineToLinearHeading(new Pose2d(45, -59))

                                .lineTo(new Vector2d(50, -59))

                                 */

                                // Left Prop

                                /*
                                .lineToLinearHeading(new Pose2d(12.5, -29.5, Math.toRadians(180)))
                                .lineTo(new Vector2d(11, -29.5))
                                .waitSeconds(0.1)

                                .lineTo(new Vector2d(49, -29.5))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(45, -29.5))
                                .lineToLinearHeading(new Pose2d(45, -59))

                                .lineTo(new Vector2d(50, -59))

                                 */

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueRight)
                .start();

    }

}