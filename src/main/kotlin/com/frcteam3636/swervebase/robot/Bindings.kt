package com.frcteam3636.swervebase.robot

import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine

val controller = CommandXboxController(2)
val joystickLeft = CommandJoystick(0)
val joystickRight = CommandJoystick(1)

@Suppress("unused")
private val joystickDev = CommandJoystick(3)

@Suppress("unused")
private val controllerDev = CommandXboxController(4)

/** Configure which commands each joystick button triggers. */
fun configureBindings() {
    Drivetrain.defaultCommand = Drivetrain.driveWithJoysticks(joystickLeft.hid, joystickRight.hid)
    // (The button with the yellow tape on it)
    joystickLeft.button(8).onTrue(Commands.runOnce({
        println("Zeroing gyro.")
        Drivetrain.zeroGyro()
    }).ignoringDisable(true))


    if (Preferences.getBoolean("DeveloperMode", false)) {
        controllerDev.leftBumper().onTrue(Commands.runOnce(SignalLogger::start))
        controllerDev.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop))

        controllerDev.y().whileTrue(Drivetrain.sysIdQuasistaticSpin(SysIdRoutine.Direction.kForward))
        controllerDev.a().whileTrue(Drivetrain.sysIdQuasistaticSpin(SysIdRoutine.Direction.kReverse))
        controllerDev.b().whileTrue(Drivetrain.sysIdDynamicSpin(SysIdRoutine.Direction.kForward))
        controllerDev.x().whileTrue(Drivetrain.sysIdDynamicSpin(SysIdRoutine.Direction.kReverse))

        controllerDev.povUp().whileTrue(Drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward))
        controllerDev.povDown().whileTrue(Drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        controllerDev.povRight().whileTrue(Drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward))
        controllerDev.povLeft().whileTrue(Drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse))

        joystickDev.button(1).whileTrue(Drivetrain.calculateWheelRadius())
    }
}