package com.frcteam3636.swervebase.subsystems.drivetrain

//import org.photonvision.PhotonCamera
//import org.photonvision.PhotonPoseEstimator
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.frcteam3636.swervebase.Robot
import com.frcteam3636.swervebase.RobotState
import com.frcteam3636.swervebase.utils.LimelightHelpers
import com.frcteam3636.swervebase.utils.math.degrees
import com.frcteam3636.swervebase.utils.math.inSeconds
import com.frcteam3636.swervebase.utils.math.meters
import com.frcteam3636.swervebase.utils.math.seconds
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import edu.wpi.first.wpilibj.RobotController
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import java.nio.ByteBuffer
import java.util.*
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.thread
import kotlin.math.pow


// we can't use @Logged here because measurement is a nullable SerializableStruct :(
class AbsolutePoseProviderInputs : LoggableInputs {
    /**
     * The most recent measurement from the pose estimator.
     */
    var measurement: AbsolutePoseMeasurement? = null

    /**
     * Whether the provider is connected.
     */
    var connected = false

    var observedTags: IntArray = intArrayOf()

    var measurementRejected = false

    var cornerCount = 0

    override fun toLog(table: LogTable) {
        if (measurement != null) {
            table.put("Measurement", measurement)
        }
        table.put("Connected", connected)
        table.put("Observed Tags", observedTags)
        table.put("Measurement Rejected", measurementRejected)
        table.put("Corner Count", cornerCount)
    }

    override fun fromLog(table: LogTable) {
        measurement = table.get("Measurement", measurement)[0]
        connected = table.get("Connected", connected)
        observedTags = table.get("Observed Tags", observedTags)
        measurementRejected = table.get("Measurement Rejected", measurementRejected)
    }
}

interface AbsolutePoseProvider {
    fun updateInputs(inputs: AbsolutePoseProviderInputs)
}

/**
 * A Limelight localization algorithm.
 */
sealed class LimelightAlgorithm {
    /**
     * An older, less accurate localization algorithm.
     */
    object MegaTag : LimelightAlgorithm()

    /**
     * A newer and much more accurate algorithm that requires accurate gyro readings and a right-side-up Limelight.
     */
    class MegaTag2(private val gyroGetter: () -> Rotation2d, private val velocityGetter: () -> AngularVelocity) :
        LimelightAlgorithm() {
        val gyroPosition: Rotation2d
            get() = gyroGetter()
        val gyroVelocity: AngularVelocity
            get() = velocityGetter()
    }
}

data class LimelightMeasurement(
    var poseMeasurement: AbsolutePoseMeasurement? = null,
    var observedTags: IntArray = intArrayOf(),
    var shouldReject: Boolean = false,
    var cornerCount: Int = 0,
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
    private val name: String,
    private val afterEnableAlgo: LimelightAlgorithm.MegaTag2,
    private val isLL4: Boolean
) : AbsolutePoseProvider {
    // References:
    // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation
    // https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib#4-field-localization-with-megatag

    private var observedTags = intArrayOf()

    private var measurement: AbsolutePoseMeasurement? = null
    private var shouldReject: Boolean = false
    private var lock = ReentrantLock()

    private var lastSeenHb: Double = 0.0
    private var hbSub = NetworkTableInstance.getDefault().getTable(name).getDoubleTopic("hb").subscribe(0.0)
    private var cornerSub =
        NetworkTableInstance.getDefault().getTable(name).getDoubleArrayTopic("tcornxy").subscribe(doubleArrayOf())
    private var loopsSinceLastSeen: Int = 0

    private var currentAlgorithm: LimelightAlgorithm = LimelightAlgorithm.MegaTag

    private var isThrottled = false

    private var cornerCount = 0

    init {
        thread(isDaemon = true) {
            while (true) {
                val temp = updateCurrentMeasurement()
                try {
                    lock.lock()
                    measurement = temp.poseMeasurement
                    observedTags = temp.observedTags
                    shouldReject = temp.shouldReject
                } finally {
                    lock.unlock()
                }
                Thread.sleep(Robot.period.toLong())
            }
        }
    }

    private fun updateCurrentMeasurement(): LimelightMeasurement {
        val measurement = LimelightMeasurement()

        if ((!RobotState.beforeFirstEnable) && currentAlgorithm == LimelightAlgorithm.MegaTag) {
            currentAlgorithm = afterEnableAlgo
            if (isLL4)
                LimelightHelpers.SetIMUMode(name, 3)
        }

        cornerCount = cornerSub.get(doubleArrayOf()).size
        measurement.cornerCount = cornerCount
        if (cornerCount < 8) {
            measurement.shouldReject = true
        }

        when (currentAlgorithm) {
            is LimelightAlgorithm.MegaTag ->
                LimelightHelpers.getBotPoseEstimate_wpiBlue(name)?.let { estimate ->
                    if (!isLL4) {
                        LimelightHelpers.SetRobotOrientation(
                            name,
                            afterEnableAlgo.gyroPosition.degrees,
                            // The Limelight sample code leaves these as zero, and the API docs call them "Unnecessary."
                            0.0, 0.0, 0.0, 0.0, 0.0
                        )
                    } else {
                        if (Robot.isDisabled && !isThrottled) {
                            LimelightHelpers.SetThrottle(name, 100)
                            isThrottled = true
                        }
                        if (RobotState.beforeFirstEnable) {
                            LimelightHelpers.SetIMUMode(name, 1)
                            LimelightHelpers.SetRobotOrientation(
                                name,
                                afterEnableAlgo.gyroPosition.degrees,
                                // The Limelight sample code leaves these as zero, and the API docs call them "Unnecessary."
                                0.0, 0.0, 0.0, 0.0, 0.0
                            )
                        }
                    }

                    measurement.observedTags = estimate.rawFiducials.mapNotNull { it?.id }.toIntArray()

                    // Reject zero tag or low-quality one tag readings
                    if (estimate.tagCount == 0) {
                        measurement.shouldReject = true
                    }
                    if (estimate.tagCount == 1) {
                        val fiducial = estimate.rawFiducials[0]!!
                        if (fiducial.ambiguity > AMBIGUITY_THRESHOLD || fiducial.distToCamera > MAX_SINGLE_TAG_DISTANCE)
                            measurement.shouldReject = true
                    }

                    measurement.poseMeasurement = AbsolutePoseMeasurement(
                        estimate.pose,
                        estimate.timestampSeconds.seconds,
                        APRIL_TAG_STD_DEV(estimate.avgTagDist, estimate.tagCount)
                    )
                }

            is LimelightAlgorithm.MegaTag2 -> {
                if (!isLL4) {
                    LimelightHelpers.SetRobotOrientation(
                        name,
                        afterEnableAlgo.gyroPosition.degrees,
                        // The Limelight sample code leaves these as zero, and the API docs call them "Unnecessary."
                        0.0, 0.0, 0.0, 0.0, 0.0
                    )
                } else {
                    if (Robot.isDisabled && !isThrottled) {
                        LimelightHelpers.SetThrottle(name, 100)
                        isThrottled = true
                    } else if (Robot.isEnabled && isThrottled) {
                        LimelightHelpers.SetThrottle(name, 0)
                    }
                }

                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name)?.let { estimate ->
                    measurement.observedTags = estimate.rawFiducials.mapNotNull { it?.id }.toIntArray()
                    val highSpeed = afterEnableAlgo.gyroVelocity.abs(DegreesPerSecond) > 360.0
                    if (estimate.tagCount == 0 || highSpeed) measurement.shouldReject = true

                    measurement.poseMeasurement = AbsolutePoseMeasurement(
                        estimate.pose,
                        estimate.timestampSeconds.seconds,
                        MEGATAG2_STD_DEV(estimate.avgTagDist, estimate.tagCount)
                    )
                }
            }

        }

        return measurement
    }

    override fun updateInputs(inputs: AbsolutePoseProviderInputs) {
        try {
            lock.lock()
            inputs.measurement = measurement
            inputs.observedTags = observedTags
            inputs.measurementRejected = shouldReject
            inputs.cornerCount = cornerCount
        } finally {
            lock.unlock()
        }

        // We assume the camera has disconnected if there are no new updates for several ticks.
        val hb = hbSub.get()
        inputs.connected = hb > lastSeenHb || loopsSinceLastSeen < CONNECTED_TIMEOUT
        if (hb == lastSeenHb)
            loopsSinceLastSeen++
        else
            loopsSinceLastSeen = 0
        lastSeenHb = hb
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
        private const val AMBIGUITY_THRESHOLD = 0.7

        /**
         * The amount of time (in robot ticks) an update before considering the camera to be disconnected.
         */
        private const val CONNECTED_TIMEOUT = 250.0
    }
}

class PhoenixOdometryThread : Thread("PhoenixOdometry") {
    init {
        isDaemon = true
    }

    private val timestampQueues: MutableList<Queue<Double>> = ArrayList()
    private val signalsLock: Lock = ReentrantLock() // Prevents conflicts when registering signals
    private var phoenixSignals: Array<BaseStatusSignal> = emptyArray() // Phoenix API does not accept a mutable list
    private val phoenixQueues = ArrayList<Queue<Double>>()

    override fun start() {
        if (!timestampQueues.isEmpty()) {
            super.start()
        }
    }

    fun registerSignal(signal: StatusSignal<Angle>): Queue<Double> {
        val queue: Queue<Double> = ArrayBlockingQueue(20)

        signalsLock.lock()
        Robot.odometryLock.lock()
        try {
            val newSignals = Array(phoenixSignals.size + 1) { i ->
                if (i < phoenixSignals.size) phoenixSignals[i] else signal
            }
            phoenixSignals = newSignals
            phoenixQueues.add(queue)
        } finally {
            signalsLock.unlock()
            Robot.odometryLock.unlock()
        }

        return queue
    }

    fun makeTimestampQueue(): Queue<Double> {
        val queue = ArrayBlockingQueue<Double>(20)
        Robot.odometryLock.lock()
        try {
            timestampQueues.add(queue)
        } finally {
            Robot.odometryLock.unlock()
        }
        return queue
    }

    override fun run() {
        while (true) {
            signalsLock.lock()
            try {
                if (!phoenixSignals.isEmpty()) {
                    BaseStatusSignal.waitForAll(2.0 / 250.0, *phoenixSignals)
                } else {
                    sleep((1000.0 / 250.0).toLong())
                    if (!phoenixSignals.isEmpty())
                        BaseStatusSignal.refreshAll(*phoenixSignals)
                }
            } catch (e: InterruptedException) {
                e.printStackTrace()
            } finally {
                signalsLock.unlock()
            }

            Robot.odometryLock.lock()
            try {
                var timestamp = RobotController.getFPGATime() / 1e6
                var latency = 0.0
                for (signal in phoenixSignals) {
                    latency += signal.timestamp.latency
                }
                if (!phoenixSignals.isEmpty())
                    timestamp -= latency / phoenixSignals.size

                for (i in 0..<phoenixSignals.size) {
                    phoenixQueues[i].offer(phoenixSignals[i].valueAsDouble)
                }

                for (i in 0..<timestampQueues.size) {
                    timestampQueues[i].offer(timestamp)
                }
            } finally {
                Robot.odometryLock.unlock()
            }
        }
    }

    companion object {
        private val instance = PhoenixOdometryThread()

        fun getInstance(): PhoenixOdometryThread {
            return instance
        }
    }
}

class CameraSimPoseProvider(name: String, val chassisToCamera: Transform3d) : AbsolutePoseProvider {
    private val camera = PhotonCamera(name)
    private val simProperties = SimCameraProperties().apply {
        setCalibration(1280, 800, Rotation2d(LIMELIGHT_FOV))
        fps = 20.0
        avgLatencyMs = 51.0
        latencyStdDevMs = 5.0
    }
    val sim = PhotonCameraSim(camera, simProperties)

    private val estimator =
        PhotonPoseEstimator(
            FIELD_LAYOUT,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            chassisToCamera
        )

    override fun updateInputs(inputs: AbsolutePoseProviderInputs) {
        inputs.connected = true
        inputs.measurement = null
        val unreadResults = camera.allUnreadResults
        val latestResult = unreadResults.lastOrNull()
        if (latestResult != null) {
            estimator.update(latestResult).ifPresent {
                inputs.measurement = AbsolutePoseMeasurement(
                    it.estimatedPose.toPose2d(),
                    it.timestampSeconds.seconds,
                    VecBuilder.fill(0.7, 0.7, 9999999.0)
                )
            }
            inputs.observedTags = latestResult.targets.map {
                it.fiducialId
            }.toIntArray()
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
    val stdDeviation: Matrix<N3, N1>
) : StructSerializable {
    companion object {
        @JvmField
        @Suppress("unused")
        val struct = AbsolutePoseMeasurementStruct()
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
    override fun getTypeName(): String {
        return "struct:AbsolutePoseMeasurement"
    }

    override fun getTypeString(): String = "struct:AbsolutePoseMeasurement"
    override fun getSize(): Int = Pose3d.struct.size + Struct.kSizeDouble + 3 * Struct.kSizeDouble
    override fun getSchema(): String = "Pose2d pose; double timestamp; double stdDeviation[3];"
    override fun unpack(bb: ByteBuffer): AbsolutePoseMeasurement =
        AbsolutePoseMeasurement(
            pose = Pose2d.struct.unpack(bb),
            timestamp = bb.double.seconds,
            stdDeviation = VecBuilder.fill(bb.double, bb.double, bb.double)
        )

    override fun pack(bb: ByteBuffer, value: AbsolutePoseMeasurement) {
        Pose2d.struct.pack(bb, value.pose)
        bb.putDouble(value.timestamp.inSeconds())
        bb.putDouble(value.stdDeviation[0, 0])
        bb.putDouble(value.stdDeviation[1, 0])
        bb.putDouble(value.stdDeviation[2, 0])
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