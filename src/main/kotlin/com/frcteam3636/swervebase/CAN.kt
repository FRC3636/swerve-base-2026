package com.frcteam3636.swervebase

import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.hardware.TalonFX

enum class CTREDeviceId(val num: Int, val bus: String) {
    FrontLeftDrivingMotor(1, "*"),
    BackLeftDrivingMotor(2, "*"),
    BackRightDrivingMotor(3, "*"),
    FrontRightDrivingMotor(4, "*"),
    FrontLeftTurningMotor(5, "*"),
    BackLeftTurningMotor(6, "*"),
    BackRightTurningMotor(7, "*"),
    FrontRightTurningMotor(8, "*"),
    PigeonGyro(20, "*"),
    FrontLeftTurningEncoder(9, "*"),
    BackLeftTurningEncoder(10, "*"),
    BackRightTurningEncoder(11, "*"),
    FrontRightTurningEncoder(12, "*"),
}

fun CANcoder(id: CTREDeviceId) = CANcoder(id.num, id.bus)
fun TalonFX(id: CTREDeviceId) = TalonFX(id.num, id.bus)
fun Pigeon2(id: CTREDeviceId) = Pigeon2(id.num, id.bus)
