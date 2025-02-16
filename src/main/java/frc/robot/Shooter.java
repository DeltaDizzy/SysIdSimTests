// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Shooter extends SubsystemBase {
  TalonFX shooter;
  TalonFXSimState simState;
  VelocityVoltage velocity = new VelocityVoltage(0);
  VoltageOut voltage = new VoltageOut(0);
  DCMotorSim motorSim;
  SysIdRoutine routine;

  /** Creates a new Shooter. */
  public Shooter() {
    shooter = new TalonFX(ShooterConstants.ShooterCANId);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // https://www.reca.lc/flywheel?currentLimit=%7B%22s%22%3A70%2C%22u%22%3A%22A%22%7D&efficiency=90&flywheelMomentOfInertia=%7B%22s%22%3A3%2C%22u%22%3A%22in2%2Albs%22%7D&flywheelRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&flywheelRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Reduction%22%7D&flywheelWeight=%7B%22s%22%3A1.5%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Falcon%20500%22%7D&motorRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Step-up%22%7D&projectileRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&projectileWeight=%7B%22s%22%3A1%2C%22u%22%3A%22lbs%22%7D&shooterMomentOfInertia=%7B%22s%22%3A9%2C%22u%22%3A%22in2%2Albs%22%7D&shooterRadius=%7B%22s%22%3A3%2C%22u%22%3A%22in%22%7D&shooterTargetSpeed=%7B%22s%22%3A5500%2C%22u%22%3A%22rpm%22%7D&shooterWeight=%7B%22s%22%3A2%2C%22u%22%3A%22lbs%22%7D&useCustomFlywheelMoi=0&useCustomShooterMoi=0
    //config.Slot0.kS = ShooterConstants.ShooterKs;
    config.Slot0.kV = ShooterConstants.ShooterKv;
    config.Slot0.kA = ShooterConstants.ShooterKa;
    config.Slot0.kP = ShooterConstants.ShooterKp;
    shooter.getConfigurator().apply(config);

    if (Robot.isSimulation()) {
      motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
          ShooterConstants.ShooterKv, 
          ShooterConstants.ShooterKa
        ), 
        DCMotor.getFalcon500(1)
      );
    }

    routine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null, 
        null, 
        null, 
        (state) -> SignalLogger.writeString("state", state.toString())), 
      new SysIdRoutine.Mechanism(
        (volts) -> shooter.setControl(voltage.withOutput(volts)), 
        null, 
        this, 
        "shooter"
      )
    );
  }

  public Command runWithSpeed(Supplier<AngularVelocity> speed) {
    return run(() -> shooter.setControl(velocity.withVelocity(speed.get())));
  }

  public Command runWithSpeed(AngularVelocity speed) {
    return runOnce(() -> shooter.setControl(velocity.withVelocity(speed))).andThen(Commands.idle(this));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction).andThen(Commands.waitUntil(() -> getShooterSpeedRps() < 0.4));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction).andThen(Commands.waitUntil(() -> getShooterSpeedRps() < 0.4));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getShooterSpeedRps() {
    return shooter.getVelocity().getValueAsDouble();
  }

  public double getShooterVelocityError() {
    return shooter.getClosedLoopError().getValueAsDouble();
  }

  public double getShooterVelocityFF() {
    return shooter.getClosedLoopFeedForward().getValueAsDouble();
  }

  public double getShooterVelocityPID() {
    return shooter.getClosedLoopProportionalOutput().getValueAsDouble();
  }

  public double getShooterStatorCurrent() {
    return shooter.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void simulationPeriodic() {
    simState = shooter.getSimState();
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    motorSim.setInput(simState.getMotorVoltage());

    motorSim.update(0.02);

    simState.setRotorVelocity(motorSim.getAngularVelocity());
    simState.setRawRotorPosition(motorSim.getAngularPosition());
  }
}
