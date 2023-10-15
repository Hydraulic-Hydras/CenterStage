package com.hydraulichydras.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {

    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(800);

        RoadRunnerBotEntity blueRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 65, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(15,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -64, Math.toRadians(90)))

                                // BLUE RIGHT

                                .lineTo(new Vector2d(-36, 24))

                                .lineToLinearHeading(new Pose2d(-59, 24, Math.toRadians(180)))
                                .waitSeconds(1)
                                .setReversed(true)

                                .splineToSplineHeading(new Pose2d(-40, 12, Math.toRadians(180)), Math.toRadians(0))

                                .lineTo(new Vector2d(24,12))

                                // .lineToLinearHeading(new Pose2d(24, 12, Math.toRadians(0)))

                                //  .splineTo(new Vector2d(50, 35), Math.toRadians(0)) makes movement slower

                                .splineToConstantHeading(new Vector2d(50, 35), Math.toRadians(0))
                                .waitSeconds(5)

                                .build()
                );

        RoadRunnerBotEntity blueLeft = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(15,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 62, Math.toRadians(270)))

                                // BLUE LEFT
                                .lineToLinearHeading(new Pose2d(15, 24, Math.toRadians(180)))
                                .setReversed(true)

                                .splineToConstantHeading(new Vector2d(50, 59), Math.toRadians(0))
                                .waitSeconds(1)

                                .lineTo(new Vector2d(-36, 59))

                                .splineTo(new Vector2d(-57, 36), Math.toRadians(180))
                                .waitSeconds(1)

                                // if passover this turn isn't needed

                                .splineToConstantHeading(new Vector2d(-36, 59),Math.toRadians(0))

                                .lineTo(new Vector2d(60,59))

                                .waitSeconds(1)


                                .build()

                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueRight)
                .addEntity(blueLeft)
                .start();

    }
}