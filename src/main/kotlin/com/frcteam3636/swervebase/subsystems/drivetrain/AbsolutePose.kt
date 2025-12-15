@file:Suppress("unused")

package com.frcteam3636.swervebase.subsystems.drivetrain

//import org.photonvision.PhotonCamera
//import org.photonvision.PhotonPoseEstimator
import com.frcteam3636.swervebase.Robot
import com.frcteam3636.swervebase.RobotState
import com.frcteam3636.swervebase.utils.LimelightHelpers.convertToLLPoseEstimate
import com.frcteam3636.swervebase.utils.math.degrees
import com.frcteam3636.swervebase.utils.math.inMilliseconds
import com.frcteam3636.swervebase.utils.math.inSeconds
import com.frcteam3636.swervebase.utils.math.meters
import com.frcteam3636.swervebase.utils.math.seconds
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.Struct.kSizeBool
import edu.wpi.first.util.struct.Struct.kSizeInt32
import edu.wpi.first.util.struct.StructSerializable
import org.photonvision.PhotonCamera
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.team9432.annotation.Logged
import java.nio.ByteBuffer
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.thread
import kotlin.math.pow

@Logged
open class AbsolutePoseProviderInputs {
    /**
     * The most recent measurement from the pose estimator.
     */
    var measurements: Array<AbsolutePoseMeasurement> = arrayOf()

    var latestTargetObservation = TargetObservation(Rotation2d.kZero, Rotation2d.kZero)

    /**
     * Whether the provider is connected.
     */
    var connected = false

    var observedTags: IntArray = intArrayOf()
}

interface AbsolutePoseProvider {
    fun updateInputs(inputs: AbsolutePoseProviderInputs)
}

data class LimelightMeasurement(
    var poseMeasurement: AbsolutePoseMeasurement? = null,
    var observedTags: IntArray = intArrayOf(),
    var shouldReject: Boolean = false,
) /* --- BEGIN KOTLIN COMPILER GENERATED CODE ---- */ {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as LimelightMeasurement

        if (poseMeasurement != other.poseMeasurement) return false
        if (!observedTags.contentEquals(other.observedTags)) return false
        if (shouldReject != other.shouldReject) return false

        return true
    }

    override fun hashCode(): Int {
        var result = poseMeasurement?.hashCode() ?: 0
        result = 31 * result + observedTags.contentHashCode()
        return result
    }
} /* --- END KOTLIN COMPILER GENERATED CODE ---- */

class LimelightPoseProvider(
    name: String,
    private val yawGetter: () -> Rotation2d,
    private val velocityGetter: () -> AngularVelocity,
    private val gyroConnectionGetter: () -> Boolean,
    private val isLL4: Boolean
) : AbsolutePoseProvider {
    // References:
    // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation
    // https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib#4-field-localization-with-megatag

    private var observedTags = mutableListOf<Int>()

    private var measurements = mutableListOf<AbsolutePoseMeasurement>()
    private var shouldReject: Boolean = false
    private var lock = ReentrantLock()

    private var lastSeenHb: Double = 0.0
    private var table = NetworkTableInstance.getDefault().getTable(name)
    private var hbSubscriber = table.getDoubleTopic("hb").subscribe(0.0)
    private var txSubscriber = table.getDoubleTopic("tx").subscribe(0.0)
    private var tySubscriber = table.getDoubleTopic("ty").subscribe(0.0)
    private var megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(doubleArrayOf())
    private var megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(doubleArrayOf())
    private val gyroPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish()
    private val throttlePublisher = table.getIntegerTopic("throttle_set").publish()
    private val imuModePublisher = table.getIntegerTopic("imumode_set").publish()
    private val imuAlphaPublisher = table.getDoubleTopic("imuassistalpha_set").publish()
    private var loopsSinceLastSeen: Int = 0
    private var connected = false

    private var isThrottled = false

    private var wasIMUChanged = false

    private var cornerCount = 0

    val gyroVelocity: AngularVelocity
        get() = velocityGetter()

    val gyroAngle: Rotation2d
        get() = yawGetter()

    val gyroConnected: Boolean
        get() = gyroConnectionGetter()

    init {
        thread(isDaemon = true, name = name) { // TODO: do we need to keep this in a thread?
            while (true) {
                val temp = updateCurrentMeasurements()
                try {
                    lock.lock()
                    if (measurements.isEmpty())
                        observedTags.clear()
                    for (measurement in temp) {
                        measurements.add(measurement.poseMeasurement!!)
                        for (tag in measurement.observedTags) {
                            observedTags.add(tag)
                        }
                    }
                    // We assume the camera has disconnected if there are no new updates for several ticks.
                    val hb = hbSubscriber.get()
                    connected = hb > lastSeenHb || loopsSinceLastSeen < CONNECTED_TIMEOUT
                    if (hb == lastSeenHb)
                        loopsSinceLastSeen++
                    else
                        loopsSinceLastSeen = 0
                    lastSeenHb = hb
                } finally {
                    lock.unlock()
                }
                Thread.sleep(Robot.period.seconds.inMilliseconds().toLong())
            }
        }
    }

    private fun updateCurrentMeasurements(): MutableList<LimelightMeasurement> {
        val measurements: MutableList<LimelightMeasurement> = mutableListOf()

        if (!isLL4) {
            gyroPublisher.accept(doubleArrayOf(gyroAngle.degrees, 0.0, 0.0, 0.0, 0.0, 0.0))
            NetworkTableInstance.getDefault().flush()
        } else {
            gyroPublisher.accept(doubleArrayOf(gyroAngle.degrees, 0.0, 0.0, 0.0, 0.0, 0.0))
            NetworkTableInstance.getDefault().flush()
            if (RobotState.beforeFirstEnable) {
                imuModePublisher.accept(1.toLong())
            }
            if (Robot.isDisabled && !isThrottled) {
                throttlePublisher.accept(100.toLong())
                isThrottled = true
            } else if (Robot.isEnabled && isThrottled) {
                isThrottled = false
                throttlePublisher.accept(0.toLong())
            }
        }

        if ((!RobotState.beforeFirstEnable)) {
            if (isLL4 && !wasIMUChanged) {
                imuModePublisher.accept(3.toLong())
                wasIMUChanged = true
            }
        }

        for (rawSample in megatag1Subscriber.readQueue()) {
            if (rawSample.value.size == 0 || !RobotState.beforeFirstEnable) continue
            val measurement = LimelightMeasurement()

            val estimate = convertToLLPoseEstimate(rawSample.value, false)

            measurement.observedTags = estimate.rawFiducials.mapNotNull { it?.id }.toIntArray()

            // Reject zero tag or low-quality one tag readings
            if (estimate.tagCount == 0) {
                measurement.shouldReject = true
            }
            if (estimate.tagCount == 1) {
                val fiducial = estimate.rawFiducials[0]!!
                if (fiducial.ambiguity > AMBIGUITY_THRESHOLD)
                    measurement.shouldReject = true
            }

            measurement.poseMeasurement = AbsolutePoseMeasurement(
                estimate.pose,
                (rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3).seconds,
                APRIL_TAG_STD_DEV(estimate.avgTagDist, estimate.tagCount),
                measurement.shouldReject,
                measurement.observedTags.size
            )

            measurements.add(measurement)
        }

        for (rawSample in megatag2Subscriber.readQueue()) {
            if (rawSample.value.size == 0 || RobotState.beforeFirstEnable || !gyroConnected) continue
            val measurement = LimelightMeasurement()
            val estimate = convertToLLPoseEstimate(rawSample.value, true)
            measurement.observedTags = estimate.rawFiducials.mapNotNull { it?.id }.toIntArray()
            val highSpeed = gyroVelocity.abs(DegreesPerSecond) > 360.0
            if (estimate.tagCount == 0 || highSpeed) measurement.shouldReject = true

            measurement.poseMeasurement = AbsolutePoseMeasurement(
                estimate.pose,
                (rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3).seconds,
                MEGATAG2_STD_DEV(estimate.avgTagDist, estimate.tagCount),
                measurement.shouldReject,
                measurement.observedTags.size
            )

            measurements.add(measurement)
        }

        if (txSubscriber.get() != 0.0 && isLL4) {
            val latestPoseReading = megatag1Subscriber.get()
            if (latestPoseReading.size > 0) {
                imuAlphaPublisher.accept(0.2)
            }
        }

        return measurements
    }

    override fun updateInputs(inputs: AbsolutePoseProviderInputs) {
        try {
            lock.lock()
            inputs.measurements = measurements.toTypedArray()
            inputs.observedTags = observedTags.toIntArray()
            measurements.clear()
            inputs.latestTargetObservation =
                TargetObservation(Rotation2d(txSubscriber.get().degrees), Rotation2d(tySubscriber.get().degrees))
            inputs.connected = connected
        } finally {
            lock.unlock()
        }
    }

    companion object {
        /**
         * The acceptable distance for a single-April-Tag reading.
         *
         * This is a somewhat conservative limit, but it is only applied when using the old MegaTag v1 algorithm.
         * It's possible it could be increased if it's too restrictive.
         */
        private val MAX_SINGLE_TAG_DISTANCE = 3.meters

        /**
         * The acceptable ambiguity for a single-tag reading on MegaTag v1.
         */
        private const val AMBIGUITY_THRESHOLD = 0.3

        /**
         * The amount of time (in robot ticks) an update before considering the camera to be disconnected.
         */
        private const val CONNECTED_TIMEOUT = 250.0
    }
}

class PhotonVisionPoseProvider(name: String, val chassisToCamera: Transform3d) : AbsolutePoseProvider {
    private val camera = PhotonCamera(name)

    override fun updateInputs(inputs: AbsolutePoseProviderInputs) {
        inputs.connected = camera.isConnected
        inputs.measurements = emptyArray()
        inputs.observedTags = intArrayOf()

        for (result in camera.allUnreadResults) {
            if (result.hasTargets()) {
                inputs.latestTargetObservation =
                    TargetObservation(
                        Rotation2d(result.bestTarget.yaw.degrees),
                        Rotation2d(result.bestTarget.pitch.degrees)
                    )
                if (result.multitagResult.isPresent) {
                    val multitagResult = result.multitagResult.get()

                    val fieldToCamera = multitagResult.estimatedPose.best
                    val fieldToRobot = fieldToCamera.plus(chassisToCamera.inverse())
                    val robotPose =
                        Pose2d(fieldToRobot.translation.toTranslation2d(), fieldToRobot.rotation.toRotation2d())
                    var totalTagDistance = 0.0
                    for (target in result.targets) {
                        totalTagDistance += target.bestCameraToTarget.translation.norm
                    }

                    for (tag in multitagResult.fiducialIDsUsed) {
                        inputs.observedTags += tag.toInt()
                    }

                    inputs.measurements += AbsolutePoseMeasurement(
                        robotPose,
                        result.timestampSeconds.seconds,
                        APRIL_TAG_STD_DEV(
                            totalTagDistance / result.targets.size,
                            multitagResult.fiducialIDsUsed.size
                        ),
                        false,
                        multitagResult.fiducialIDsUsed.size
                    )
                } else {
                    val target = result.targets.first()
                    var shouldReject = false
                    val tagPose = FIELD_LAYOUT.getTagPose(target.fiducialId)
                    if (tagPose.isPresent) {
                        val cameraToTarget = target.bestCameraToTarget
                        val fieldToTarget = Transform3d(tagPose.get().translation, tagPose.get().rotation)
                        val fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse())
                        val fieldToRobot = fieldToCamera.plus(chassisToCamera.inverse())
                        val robotPose =
                            Pose2d(fieldToRobot.translation.toTranslation2d(), fieldToRobot.rotation.toRotation2d())

                        if (result.bestTarget.poseAmbiguity > 0.3 || result.bestTarget.bestCameraToTarget.translation.norm > FIELD_LAYOUT.fieldLength / 2) {
                            shouldReject = true
                        }

                        inputs.observedTags += target.fiducialId
                        inputs.measurements += AbsolutePoseMeasurement(
                            robotPose,
                            result.timestampSeconds.seconds,
                            APRIL_TAG_STD_DEV(cameraToTarget.translation.norm, result.targets.size),
                            shouldReject,
                            result.targets.size
                        )
                    }
                }
            } else {
                inputs.latestTargetObservation = TargetObservation()
            }
        }
    }
}

class CameraSimPoseProvider(name: String, val chassisToCamera: Transform3d) : AbsolutePoseProvider {
    private val camera = PhotonCamera(name)
    private val simProperties = SimCameraProperties().apply {
        setCalibration(1280, 960, Rotation2d(LIMELIGHT_FOV))
        fps = 120.0
        avgLatencyMs = 17.0
        latencyStdDevMs = 5.0
    }
    val sim = PhotonCameraSim(camera, simProperties)


    override fun updateInputs(inputs: AbsolutePoseProviderInputs) {
        inputs.connected = camera.isConnected
        inputs.measurements = arrayOf()

        val unreadResults = camera.allUnreadResults
        for (result in unreadResults) {
            if (result.hasTargets()) {
                val target = result.targets[0] // we don't need multitag in sim
                var shouldReject = false
                if (result.bestTarget.poseAmbiguity > 0.3 || result.bestTarget.bestCameraToTarget.translation.norm > FIELD_LAYOUT.fieldLength / 2) {
                    shouldReject = true
                }
                inputs.observedTags = result.targets.map {
                    it.fiducialId
                }.toIntArray()

                val tagPose = FIELD_LAYOUT.getTagPose(target.fiducialId)
                val cameraToTarget = target.bestCameraToTarget
                val fieldToTarget = Transform3d(tagPose.get().translation, tagPose.get().rotation)
                val fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse())
                val fieldToRobot = fieldToCamera.plus(chassisToCamera.inverse())
                val robotPose = Pose2d(fieldToRobot.translation.toTranslation2d(), fieldToRobot.rotation.toRotation2d())
                inputs.measurements += AbsolutePoseMeasurement(
                    robotPose,
                    result.timestampSeconds.seconds,
                    APRIL_TAG_STD_DEV(cameraToTarget.translation.norm, result.targets.size),
                    shouldReject,
                    result.targets.size
                )
            }
        }

    }
}

data class AbsolutePoseMeasurement(
    val pose: Pose2d,
    val timestamp: Time,
    /**
     * Standard deviations of the vision pose measurement (x position in meters, y position in meters, and heading in
     * radians). Increase these numbers to trust the vision pose measurement less.
     */
    val stdDeviation: Matrix<N3, N1>,
    val shouldReject: Boolean,
    // This is a COUNT of observed tags
    val observedTags: Int
) : StructSerializable {
    companion object {
        @JvmField
        @Suppress("unused")
        val struct = AbsolutePoseMeasurementStruct()
    }
}

data class TargetObservation(
    val tx: Rotation2d = Rotation2d.kZero,
    val ty: Rotation2d = Rotation2d.kZero,
) : StructSerializable {
    companion object {
        @JvmField
        @Suppress("unused")
        val struct = TargetObservationStruct()
    }
}

fun SwerveDrivePoseEstimator.addAbsolutePoseMeasurement(measurement: AbsolutePoseMeasurement) {
    addVisionMeasurement(
        measurement.pose,
        measurement.timestamp.inSeconds(),
        measurement.stdDeviation
    )
}

class AbsolutePoseMeasurementStruct : Struct<AbsolutePoseMeasurement> {
    override fun getTypeClass(): Class<AbsolutePoseMeasurement> = AbsolutePoseMeasurement::class.java
    override fun getTypeName(): String = "struct:AbsolutePoseMeasurement"
    override fun getTypeString(): String = "struct:AbsolutePoseMeasurement"
    override fun getSize(): Int =
        Pose2d.struct.size + Struct.kSizeDouble + 3 * Struct.kSizeDouble + kSizeBool + kSizeInt32

    override fun getSchema(): String =
        "Pose2d pose; double timestamp; double stdDeviation[3]; bool shouldReject; int32 observedTags;"

    override fun unpack(bb: ByteBuffer): AbsolutePoseMeasurement =
        AbsolutePoseMeasurement(
            pose = Pose2d.struct.unpack(bb),
            timestamp = bb.double.seconds,
            stdDeviation = VecBuilder.fill(bb.double, bb.double, bb.double),
            shouldReject = bb.get() != 0.toByte(), // read boolean as byte
            observedTags = bb.getInt()
        )

    override fun pack(bb: ByteBuffer, value: AbsolutePoseMeasurement) {
        Pose2d.struct.pack(bb, value.pose)
        bb.putDouble(value.timestamp.inSeconds())
        bb.putDouble(value.stdDeviation[0, 0])
        bb.putDouble(value.stdDeviation[1, 0])
        bb.putDouble(value.stdDeviation[2, 0])
        bb.put(if (value.shouldReject) 1 else 0) // write boolean as byte
        bb.putInt(value.observedTags)
    }
}

class TargetObservationStruct : Struct<TargetObservation> {
    override fun getTypeClass(): Class<TargetObservation> = TargetObservation::class.java
    override fun getTypeName(): String = "struct:TargetObservation"
    override fun getTypeString(): String = "struct:TargetObservation"
    override fun getSize(): Int =
        Rotation2d.struct.size * 2

    override fun getSchema(): String = "Rotation2d tx; Rotation2d ty;"

    override fun unpack(bb: ByteBuffer): TargetObservation =
        TargetObservation(
            Rotation2d.struct.unpack(bb),
            Rotation2d.struct.unpack(bb)
        )


    override fun pack(bb: ByteBuffer, value: TargetObservation) {
        Rotation2d.struct.pack(bb, value.tx)
        Rotation2d.struct.pack(bb, value.ty)
    }
}

internal val APRIL_TAG_STD_DEV = { distance: Double, count: Int ->
    val stdDevFactor = distance.pow(2) / count.toDouble()
    val linearStdDev = 0.02 * stdDevFactor
    val angularStdDev = 0.06 * stdDevFactor
    VecBuilder.fill(
        linearStdDev, linearStdDev, angularStdDev
    )
}

internal val MEGATAG2_STD_DEV = { distance: Double, count: Int ->
    val stdDevFactor = distance.pow(2) / count.toDouble()
    var linearStdDev = (0.02 * stdDevFactor) * 0.5
    VecBuilder.fill(
        linearStdDev, linearStdDev, Double.POSITIVE_INFINITY
    )
}

val LIMELIGHT_FOV = 75.76079874010732.degrees
