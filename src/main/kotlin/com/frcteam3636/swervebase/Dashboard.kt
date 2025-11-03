package com.frcteam3636.swervebase

import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain
import edu.wpi.first.wpilibj.smartdashboard.Field2d

object Dashboard {
    val field = Field2d()
//    val autoChooser = AutoBuilder.buildAutoChooser()!!

    fun update() {
        field.robotPose = Drivetrain.estimatedPose
    }
}
