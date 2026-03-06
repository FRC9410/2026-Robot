// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team9410.subsystems.VelocitySubsystem;

/**
 * SysId characterization for the shooter flywheel. Run quasistatic and dynamic tests in both
 * directions, then use the generated logs to tune velocity PID/FF in SysId or similar tools.
 *
 * <p>Log generation: Run "Start Shooter SysId Log" before the first test, run all four tests
 * (quasistatic forward/reverse, dynamic forward/reverse), then run "Stop Shooter SysId Log".
 * Logs are written to the path configured in Robot (e.g. /media/sda1/ctre-logs/). The shooter
 * velocity and state are logged via CTRE SignalLogger; WPILib datalog also receives the
 * mechanism data when the routine runs.
 */
public class ShooterSysId {

  private final VelocitySubsystem shooter;
  private final SysIdRoutine routine;
  /** Commanded voltage for logging (updated by the drive callback). */
  private final double[] lastVoltage = {0};

  public ShooterSysId(VelocitySubsystem shooter) {
    this.shooter = shooter;
    this.routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // default ramp rate (1 V/s)
                Volts.of(4), // dynamic step voltage
                null, // default timeout (10 s)
                state ->
                    SignalLogger.writeString("SysIdShooter_State", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> {
                  lastVoltage[0] = voltage.in(Volts);
                  shooter.setVoltage(voltage.in(Volts));
                },
                log -> {
                  var motor = shooter.getVelocityMotor();
                  if (motor == null) return;
                  double posRot = motor.getRotorPosition().getValueAsDouble();
                  double velRps = motor.getRotorVelocity().getValueAsDouble();
                  log
                      .motor("Shooter")
                      .voltage(Volts.of(lastVoltage[0]))
                      .angularPosition(Radians.of(posRot * 2 * Math.PI))
                      .angularVelocity(RadiansPerSecond.of(velRps * 2 * Math.PI));
                },
                shooter,
                "Shooter"));
  }

  /** Quasistatic test in the given direction. */
  public Command quasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  /** Dynamic test in the given direction. */
  public Command dynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  /** Starts CTRE SignalLogger so shooter SysId data is written to the log file. */
  public static Command startLog() {
    return Commands.runOnce(() -> SignalLogger.start());
  }

  /** Stops CTRE SignalLogger and flushes the log file. */
  public static Command stopLog() {
    return Commands.runOnce(() -> SignalLogger.stop());
  }
}
