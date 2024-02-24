// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands; 

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos{
  /** Example static factory for an autonomous command. */
  public static Command ShootSpeaker(ArmSubsystem mArmSubsystem, LauncherSubsystem mLauncherSubsystem, IntakeSubsystem mIntakeSubsystem) {
    return Commands.sequence(
    new ArmHomePosition(mArmSubsystem),  
    new LauncherRun(mLauncherSubsystem),
    new IntakeRunCommand(mIntakeSubsystem));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
