// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2; 
import com.ctre.phoenix.unmanaged.Unmanaged; 
import com.revrobotics.CANSparkMax; 
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants2023.CAN;
import frc.robot.Constants2023.Swerve;
import frc.robot.Constants2023.Swerve.ModulePosition;
import java.util.HashMap;
import java.util.Map;

import static frc.robot.Constants2023.Swerve.*;


public class SwerveDrive2023 extends SubsystemBase {

  private final HashMap<ModulePosition, SwerveModule2023> m_swerveModules =
          new HashMap<>(
                  Map.of(
                          ModulePosition.FRONT_LEFT,
                          new SwerveModule2023(
                                  0,
                                  new CANSparkMax(CAN.frontLeftTurnMotor, CANSparkMaxLowLevel.MotorType.kBrushless),
                                  new CANSparkMax(CAN.frontLeftDriveMotor, CANSparkMaxLowLevel.MotorType.kBrushless),
                                  new CANCoder(CAN.frontLeftCanCoder),
                                  0),
                          ModulePosition.FRONT_RIGHT,
                          new SwerveModule2023(
                                  1,
                                  new CANSparkMax(CAN.frontRightTurnMotor, CANSparkMaxLowLevel.MotorType.kBrushless),
                                  new CANSparkMax(CAN.frontRightDriveMotor, CANSparkMaxLowLevel.MotorType.kBrushless),
                                  new CANCoder(CAN.frontRightCanCoder),
                                  0),
                          ModulePosition.BACK_LEFT,
                          new SwerveModule2023(
                                  2,
                                  new CANSparkMax(CAN.backLeftTurnMotor, CANSparkMaxLowLevel.MotorType.kBrushless),
                                  new CANSparkMax(CAN.backLeftDriveMotor, CANSparkMaxLowLevel.MotorType.kBrushless),
                                  new CANCoder(CAN.backLeftCanCoder),
                                  0),
                          ModulePosition.BACK_RIGHT,
                          new SwerveModule2023(
                                  3,
                                  new CANSparkMax(CAN.backRightTurnMotor, CANSparkMaxLowLevel.MotorType.kBrushless),
                                  new CANSparkMax(CAN.backRightDriveMotor, CANSparkMaxLowLevel.MotorType.kBrushless),
                                  new CANCoder(CAN.backRightCanCoder),
                                  0)));

private ADIS16470_IMU gyro = new ADIS16470_IMU();


  private SwerveDriveOdometry m_odometry =
          new SwerveDriveOdometry(
                  Swerve.kSwerveKinematics,
                  getHeadingRotation2d(),
                  getModulePositions(),
                  new Pose2d());

  private ProfiledPIDController m_xController =
          new ProfiledPIDController(kP_X, 0, kD_X, kThetaControllerConstraints);
  private ProfiledPIDController m_yController =
          new ProfiledPIDController(kP_Y, 0, kD_Y, kThetaControllerConstraints);
  private ProfiledPIDController m_turnController =
          new ProfiledPIDController(kP_Theta, 0, kD_Theta, kThetaControllerConstraints);

  private double m_simYaw;

  public SwerveDrive2023() {
    gyro.reset();

  }

  public void drive(
          double throttle,
          double strafe,
          double rotation,
          boolean isFieldRelative,
          boolean isOpenLoop) {
    throttle *= kMaxSpeedMetersPerSecond;
    strafe *= kMaxSpeedMetersPerSecond;
    rotation *= kMaxRotationRadiansPerSecond;

    ChassisSpeeds chassisSpeeds =
            isFieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    throttle, strafe, rotation, getHeadingRotation2d())
                    : new ChassisSpeeds(throttle, strafe, rotation);

    SwerveModuleState[] moduleStates = kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kMaxSpeedMetersPerSecond);

    for (SwerveModule2023 module : m_swerveModules.values())
      module.setDesiredState(moduleStates[module.getModuleNumber()], isOpenLoop);
  }

  public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMetersPerSecond);

    for (SwerveModule2023 module : m_swerveModules.values())
      module.setDesiredState(states[module.getModuleNumber()], isOpenLoop);
  }

  public double getHeadingDegrees() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public Pose2d getPoseMeters() {
    return m_odometry.getPoseMeters();
  }

  public SwerveModule2023 getSwerveModule(int moduleNumber) {
    return m_swerveModules.get(ModulePosition.values()[moduleNumber]);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
            m_swerveModules.get(ModulePosition.FRONT_LEFT).getState(),
            m_swerveModules.get(ModulePosition.FRONT_RIGHT).getState(),
            m_swerveModules.get(ModulePosition.BACK_LEFT).getState(),
            m_swerveModules.get(ModulePosition.BACK_RIGHT).getState()
    };
  }
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
            m_swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
            m_swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
            m_swerveModules.get(ModulePosition.BACK_LEFT).getPosition(),
            m_swerveModules.get(ModulePosition.BACK_RIGHT).getPosition()
    };
  }

  public void updateOdometry() {
    m_odometry.update(getHeadingRotation2d(), getModulePositions());

    for (SwerveModule2023 module : m_swerveModules.values()) {
      var modulePositionFromChassis =
              kModuleTranslations[module.getModuleNumber()]
                      .rotateBy(getHeadingRotation2d())
                      .plus(getPoseMeters().getTranslation());
      module.setModulePose(
              new Pose2d(
                      modulePositionFromChassis,
                      module.getHeadingRotation2d().plus(getHeadingRotation2d())));
    }
  }

  private void updateSmartDashboard() {}

  @Override
  public void periodic() {
    updateOdometry();
    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeed = kSwerveKinematics.toChassisSpeeds(getModuleStates());
    m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;

    Unmanaged.feedEnable(20);
    SmartDashboard.putString("need to get a simulated Gyro working for simulated mode to work", "Please fix in SwerveDrive");
    
//    gyro.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));s
//    pigeon.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));
  }
}