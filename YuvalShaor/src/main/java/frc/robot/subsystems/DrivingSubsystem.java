// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.imu.NavXSwerve;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DrivingSubsystem extends SubsystemBase {
  private static DrivingSubsystem instance;
  private NavXSwerve gyroscope = new NavXSwerve(SPI.Port.kMXP);
  private SwerveDrive swerveDrive;
  double maximumSpeed = Units.feetToMeters(17.6);
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

  // DifferentialDrive m_drive = new DifferentialDrive(left, right);
  /** Creates a new DrivingSubsystem. */
  public static DrivingSubsystem getInstance() {
    if (instance == null)
      instance = new DrivingSubsystem();
    return instance;
  }

  private DrivingSubsystem() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    } catch (IOException e) {
      e.printStackTrace();
      throw new RuntimeException("Wrong file or directory of config");
    }
  }

  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
    swerveDrive.drive(new Translation2d(xSpeed, ySpeed), rotationSpeed, fieldRelative, false);

  }

  public void stop() {
    swerveDrive.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
