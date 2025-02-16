// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

@Logged
public class RobotContainer {
  Shooter shooter = new Shooter();
  DoubleSubscriber speedSub = NetworkTableInstance.getDefault().getDoubleTopic("/Control/ShooterSpeedRPS").subscribe(0);
  public RobotContainer() {
    speedSub.getTopic().publish().accept(60);
    DriverStation.silenceJoystickConnectionWarning(true);
    shooter.setDefaultCommand(shooter.runWithSpeed(() -> RotationsPerSecond.of(speedSub.get())));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.sequence(
      shooter.sysIdQuasistatic(Direction.kForward),
      shooter.sysIdQuasistatic(Direction.kReverse),
      shooter.sysIdDynamic(Direction.kForward),
      shooter.sysIdDynamic(Direction.kReverse)
    );
  }
}
