// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands; 

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

  public class AutoShootSpeaker extends SequentialCommandGroup{


  public AutoShootSpeaker (ArmSubsystem mArmSubsystem, LauncherSubsystem mLauncherSubsystem, IntakeSubsystem mIntakeSubsystem,DriveSubsystem m_driveTrain) {
    // Make sure the arm is up (should already be there)
    addCommands( new ArmHomePosition(mArmSubsystem).withTimeout(1)); 
    
    // Spin up the Launcher
    addCommands( new LauncherRun(mLauncherSubsystem).withTimeout(2));

    //addCommands(new IntakeSetPower(mIntakeSubsystem, 1).withTimeout(2));

    // Now that the launcher is spinning, index the note into the launcher by running the intake
       // Setup the command
        ParallelCommandGroup runLauncerAndIntake = new ParallelCommandGroup(
           new LauncherRun(mLauncherSubsystem).withTimeout(2),
           new IntakeSetPower(mIntakeSubsystem, 1).withTimeout(2)
         );
//         runLauncerAndIntake.withTimeout(2);
      //Add the command to the sequence
        addCommands(runLauncerAndIntake);

    addCommands( new DriveTrainMove(m_driveTrain).withTimeout(2));
    
  }
  
}
