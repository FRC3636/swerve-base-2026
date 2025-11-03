package com.frcteam3636.swervebase.subsystems.drivetrain

import com.ctre.phoenix6.BaseStatusSignal
import com.frcteam3636.swervebase.CTREDeviceId
import com.frcteam3636.swervebase.Diagnostics
import com.frcteam3636.swervebase.Pigeon2
import com.frcteam3636.swervebase.Robot
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain.Constants.MODULE_POSITIONS
import com.frcteam3636.swervebase.utils.math.*
import com.frcteam3636.swervebase.utils.swerve.DrivetrainCorner
import com.frcteam3636.swervebase.utils.swerve.PerCorner
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.measure.Voltage
import org.photonvision.simulation.VisionSystemSim
import org.team9432.annotation.Logged
import kotlin.math.atan2

@Logged
open class DrivetrainInputs {
    var gyroRotation = Rotation2d()
    var gyroVelocity = 0.degreesPerSecond
    var gyroConnected = true
    var measuredStates = PerCorner.generate { SwerveModuleState() }
    var measuredPositions = PerCorner.generate { SwerveModulePosition() }
    var frontRightTemperatures = doubleArrayOf()
    var frontLeftTemperatures = doubleArrayOf()
    var backLeftTemperatures = doubleArrayOf()
    var backRightTemperatures = doubleArrayOf()
}

abstract class DrivetrainIO {
    protected abstract val gyro: Gyro
    abstract val modules: PerCorner<out SwerveModule>


    open fun updateInputs(inputs: DrivetrainInputs) {
        gyro.periodic()
        modules.forEach(SwerveModule::periodic)

        inputs.gyroRotation = gyro.rotation
        inputs.gyroVelocity = gyro.velocity
        inputs.gyroConnected = gyro.connected
        inputs.measuredStates = modules.map { it.state }
        inputs.measuredPositions = modules.map { it.position }
        inputs.frontRightTemperatures = modules.frontRight.temperatures.map { it.inCelsius() }.toDoubleArray()
        inputs.backRightTemperatures = modules.backRight.temperatures.map { it.inCelsius() }.toDoubleArray()
        inputs.frontLeftTemperatures = modules.frontLeft.temperatures.map { it.inCelsius() }.toDoubleArray()
        inputs.backLeftTemperatures = modules.backLeft.temperatures.map { it.inCelsius() }.toDoubleArray()
    }

    var desiredStates: PerCorner<SwerveModuleState>
        get() = modules.map { it.desiredState }
        set(value) {
            modules.zip(value).forEach { (module, state) -> module.desiredState = state }
        }

    fun runCharacterization(voltage: Voltage, shouldSpin: Boolean = false, shouldStraight: Boolean = false) {
        if (shouldSpin) {
            for (i in 0..<MODULE_POSITIONS.size) {
                val trans = MODULE_POSITIONS[i].position.translation
                var angle = atan2(trans.y, trans.x)
                if (MODULE_POSITIONS[i] == MODULE_POSITIONS.frontRight || MODULE_POSITIONS[i] == MODULE_POSITIONS.backRight) {
//                    angle -= 90.degrees.inRadians()
                    if (MODULE_POSITIONS[i] == MODULE_POSITIONS.backRight) {
                        modules[i].characterize(voltage, Rotation2d(angle.radians).unaryMinus().measure)
                    } else {
                        modules[i].characterize(
                            voltage,
                            Rotation2d(angle.radians).unaryMinus().measure + Rotation2d.k180deg.measure
                        )
                    }
                } else {
                    angle += 90.degrees.inRadians()
                    modules[i].characterize(voltage, Rotation2d(angle.radians).measure)
                }
            }
        } else if (shouldStraight) {
            for (i in 0..<MODULE_POSITIONS.size) {
                modules[i].characterize(voltage, MODULE_POSITIONS[i].position.rotation.unaryMinus().measure)
            }
        } else {
            // keep at same angle
            for (module in modules) {
                module.characterize(voltage, null)
            }
        }

    }

    fun getOdometryPositions(): PerCorner<Array<SwerveModulePosition>> {
        return modules.map { it.odometryPositions }
    }

    fun getOdometryTimestamps(): DoubleArray {
        return modules[DrivetrainCorner.FRONT_LEFT].odometryTimestamps
    }

    @Suppress("unused")
    fun getOdometryYawTimestamps(): DoubleArray {
        return gyro.odometryYawTimestamps
    }

    fun getOdometryYawPositions(): DoubleArray {
        return gyro.odometryYawPositions
    }

    fun getStatusSignals(): MutableList<BaseStatusSignal> {
        val signals = mutableListOf<BaseStatusSignal>()

        modules.forEach { module ->
            signals += module.getSignals()
        }
        signals += gyro.getStatusSignals()
        return signals
    }
}

/** Drivetrain I/O layer that uses real swerve modules along with a NavX gyro. */
class DrivetrainIOReal(override val modules: PerCorner<SwerveModule>) : DrivetrainIO() {
    override val gyro = when (Robot.model) {
        Robot.Model.SIMULATION -> GyroSim(modules)
        Robot.Model.COMPETITION -> GyroPigeon(Pigeon2(CTREDeviceId.PigeonGyro))
    }
}

/** Drivetrain I/O layer that uses simulated swerve modules along with a simulated gyro with an angle based off their movement. */
class DrivetrainIOSim : DrivetrainIO() {
    val vision = VisionSystemSim("main").apply {
        addAprilTags(FIELD_LAYOUT)
    }

    override val modules = PerCorner.generate { SimSwerveModule() }
    override val gyro = GyroSim(modules.map { it })
    override fun updateInputs(inputs: DrivetrainInputs) {
        super.updateInputs(inputs)
        vision.update(Drivetrain.poseEstimator.estimatedPosition)


        Diagnostics.report(gyro)
    }

    fun registerPoseProviders(providers: Iterable<AbsolutePoseProvider>) {
        for (provider in providers) {
            if (provider is CameraSimPoseProvider) {
                vision.addCamera(provider.sim, provider.chassisToCamera)
            }
        }
    }
}

val FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(
    AprilTagFields.k2025ReefscapeWelded.m_resourceFile
)!!