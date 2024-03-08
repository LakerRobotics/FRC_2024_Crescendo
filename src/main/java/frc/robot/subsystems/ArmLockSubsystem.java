// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants2023;
import frc.robot.commands.ArmLockDisengage;

public class ArmLockSubsystem extends SubsystemBase {

  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private SparkPIDController m_controller;
  private double m_setpoint;

  private TrapezoidProfile m_profile;
  private Timer m_timer;
  private TrapezoidProfile.State m_startState;
  private TrapezoidProfile.State m_endState;

  private TrapezoidProfile.State m_targetState;
  private double m_feedforward;
  private double m_manualValue;

  /** Creates a new ArmSubsystem. */
  public ArmLockSubsystem() {

    // create a new SPARK MAX and configure it
    m_motor = new CANSparkMax(Constants2023.CAN.armLockMotor, MotorType.kBrushless);
    m_motor.setInverted(false);
    //m_motor.setSmartCurrentLimit(Constants.Arm.kCurrentLimit);
    m_motor.setIdleMode(IdleMode.kCoast);

    m_motor.burnFlash();

    setDefaultCommand(new ArmLockDisengage(this));
  }

  public void runManual(double _power) {
    
    m_motor.set(_power);
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
  }
}