package com.frcteam3636.swervebase

import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain
import com.frcteam3636.swervebase.utils.ElasticWidgets
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import org.littletonrobotics.junction.Logger

object Dashboard {
    val field = Field2d()
    val autoChooser = AutoBuilder.buildAutoChooser()!!

    fun update() {
        field.robotPose = Drivetrain.estimatedPose
    }
}
