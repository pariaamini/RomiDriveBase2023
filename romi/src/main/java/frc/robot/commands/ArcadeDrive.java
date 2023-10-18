// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class ArcadeDrive extends CommandBase {
  private final Drivetrain drivetrain;
  private final Supplier<Double> xaxisSpeedSupplier;
  private final Supplier<Double> zaxisRotateSupplier;
  private final Supplier<Double> LowSensitivityControl;

  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to
   * the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain          The drivetrain subsystem on which this command
   *                            will run
   * @param xaxisSpeedSupplier  Lambda supplier of forward/backward speed
   * @param zaxisRotateSupplier Lambda supplier of rotational speed
   */
  public ArcadeDrive(
      Drivetrain drivetrain,
      Supplier<Double> xaxisSpeedSupplier,
      Supplier<Double> zaxisRotateSupplier,
      Supplier<Double> LowSensitivityControl) {
    this.drivetrain = drivetrain;
    this.xaxisSpeedSupplier = xaxisSpeedSupplier;
    this.zaxisRotateSupplier = zaxisRotateSupplier;
    this.LowSensitivityControl = LowSensitivityControl;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(xaxisSpeedSupplier.get() * 1.3

        , zaxisRotateSupplier.get());
    double rounding = LowSensitivityControl.get();
    if (rounding < 0.1 && rounding > -0.1) {
      rounding = 0;

    }
    SmartDashboard.putNumber("speed", xaxisSpeedSupplier.get());
    SmartDashboard.putNumber("turnVal", zaxisRotateSupplier.get());
    if (!(rounding == 0)) {
      drivetrain.curvatureDrive(xaxisSpeedSupplier.get() * 0.4, LowSensitivityControl.get() * 0.2, true);
    } else {
      drivetrain.curvatureDrive(xaxisSpeedSupplier.get() * 0.5, zaxisRotateSupplier.get() * 0.3, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}