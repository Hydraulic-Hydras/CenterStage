package com.hydraulichydras.hydrauliclib.Localization

import com.acmerobotics.roadrunner.drive.Drive
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.util.Angle

abstract class MecanumDrive @JvmOverloads constructor(
    private val kV: Double,
    private val kA: Double,
    private val kStatic: Double,
    private val trackWidth: Double,
    private val wheelBase: Double = trackWidth,
    private val lateralMultiplier: Double = 1.0,

) : Drive() {
class MecanumLocalizer @JvmOverloads constructor(
    private val drive: MecanumDrive,
    private val useExternalHeading: Boolean = true
) : Localizer {

private var _poseEstimate = Pose2d()
override var poseEstimate: Pose2d
    get() = _poseEstimate
    set(value) {
        lastWheelPositions = emptyList()
        lastExtHeading = Double.NaN
        if (useExternalHeading) drive.externalHeading = value.heading
        _poseEstimate = value
    }
override var poseVelocity: Pose2d? = null
    private set
private var lastWheelPositions = emptyList<Double>()
private var lastExtHeading = Double.NaN



override fun update() {
    val wheelPositions = drive.getWheelPositions()
    val extHeading = if (useExternalHeading) drive.externalHeading else Double.NaN
    if (lastWheelPositions.isNotEmpty()) {
        val wheelDeltas = wheelPositions
            .zip(lastWheelPositions)
            .map { it.first - it.second }
        val robotPoseDelta = com.acmerobotics.roadrunner.kinematics.MecanumKinematics.wheelToRobotVelocities(
            wheelDeltas,
            drive.trackWidth,
            drive.wheelBase,
            drive.lateralMultiplier
        )
        val finalHeadingDelta = if (useExternalHeading) {
            Angle.normDelta(extHeading - lastExtHeading)
        } else {
            robotPoseDelta.heading
        }
        _poseEstimate = Kinematics.relativeOdometryUpdate(
            _poseEstimate,
            Pose2d(robotPoseDelta.vec(), finalHeadingDelta)
        )
    }

    val wheelVelocities = drive.getWheelVelocities()
    val extHeadingVel = drive.getExternalHeadingVelocity()
    if (wheelVelocities != null) {
        poseVelocity = com.acmerobotics.roadrunner.kinematics.MecanumKinematics.wheelToRobotVelocities(
            wheelVelocities,
            drive.trackWidth,
            drive.wheelBase,
            drive.lateralMultiplier
        )
        if (useExternalHeading && extHeadingVel != null) {
            poseVelocity = Pose2d(poseVelocity!!.vec(), extHeadingVel)
        }
    }

    lastWheelPositions = wheelPositions
    lastExtHeading = extHeading
}
}

override var localizer: Localizer = MecanumLocalizer(this)
override fun setDriveSignal(driveSignal: DriveSignal) {
    val velocities = com.acmerobotics.roadrunner.kinematics.MecanumKinematics.robotToWheelVelocities(
        driveSignal.vel,
        trackWidth,
        wheelBase,
        lateralMultiplier
    )
    val accelerations = com.acmerobotics.roadrunner.kinematics.MecanumKinematics.robotToWheelAccelerations(
        driveSignal.accel,
        trackWidth,
        wheelBase,
        lateralMultiplier
    )
    val powers = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic)
    setMotorPowers(powers[0], powers[1], powers[2], powers[3])
}

override fun setDrivePower(drivePower: Pose2d) {
    val powers = com.acmerobotics.roadrunner.kinematics.MecanumKinematics.robotToWheelVelocities(
        drivePower,
        1.0,
        1.0,
        lateralMultiplier
    )
    setMotorPowers(powers[0], powers[1], powers[2], powers[3])
}

/**
 * Sets the following motor powers (normalized voltages). All arguments are on the interval `[-1.0, 1.0]`.
 */
abstract fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double)

/**
 * Returns the positions of the wheels in linear distance units. Positions should exactly match the ordering in
 * [setMotorPowers].
 */
abstract fun getWheelPositions(): List<Double>

/**
 * Returns the velocities of the wheels in linear distance units. Positions should exactly match the ordering in
 * [setMotorPowers].
 */
open fun getWheelVelocities(): List<Double>? = null
}

