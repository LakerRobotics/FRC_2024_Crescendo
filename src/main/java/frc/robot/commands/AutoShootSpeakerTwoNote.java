// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands; 

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

  public class AutoShootSpeakerTwoNote extends SequentialCommandGroup{


  public AutoShootSpeakerTwoNote (ArmSubsystem mArmSubsystem, LauncherSubsystem mLauncherSubsystem, IntakeSubsystem mIntakeSubsystem,DriveSubsystem m_driveTrain) {
    // Make sure the arm is up (should already be there)
    addCommands( new ArmHomePosition(mArmSubsystem).withTimeout(0.1)); 
    // addCommands(new RunCommand(() -> mArmSubsystem.runManual(0.4),mArmSubsystem));
    
    // Spin up the Launcher
    addCommands( new LauncherAutoPower(mLauncherSubsystem,0.9,1).withTimeout(2));

    //addCommands(new IntakeSetPower(mIntakeSubsystem, 1).withTimeout(2));

    // Now that the launcher is spinning, index the note into the launcher by running the intake
       // Setup the command
        ParallelCommandGroup runLauncerAndIntake = new ParallelCommandGroup(
           new ArmHomePosition(mArmSubsystem).withTimeout(3),
           new LauncherAutoPower(mLauncherSubsystem,1,1).withTimeout(2),
           new IntakeSetPower(mIntakeSubsystem, 1).withTimeout(2)
         );
      //Add the command to the sequence
        addCommands(runLauncerAndIntake);    

    //Lower the arm
    addCommands( new ArmIntakePosition(mArmSubsystem).withTimeout(2));
    
    //Spin the intake to take in the note
    addCommands( new IntakeRunCommand(mIntakeSubsystem).withTimeout(0.1));

    //Move forward towards the note
    addCommands( new DriveTrainMove(m_driveTrain, 0.2, 0, 0).withTimeout(0.4));

    //Raise the arm
    addCommands( new ArmHomePosition(mArmSubsystem).withTimeout(2));

    //Move backwards to the speaker
    addCommands( new DriveTrainMove(m_driveTrain, -0.2, 0, 0).withTimeout(0.4));

    addCommands( new DriveTrainMove(m_driveTrain, 0, 0, 0).withTimeout(0.4));
    

    //Prepare to shoot
    addCommands( new LauncherAutoPower(mLauncherSubsystem,0.9,1).withTimeout(2));    

    //Shoot second note
    // Now that the launcher is spinning, index the note into the launcher by running the intake
       // Setup the command
        ParallelCommandGroup runLauncerAndIntake2 = new ParallelCommandGroup(
           new ArmHomePosition(mArmSubsystem).withTimeout(3),
           new LauncherAutoPower(mLauncherSubsystem,1,1).withTimeout(2),
           new IntakeSetPower(mIntakeSubsystem, 1).withTimeout(2)
         );
      //Add the command to the sequence
        addCommands(runLauncerAndIntake2);

        /*ParallelCommandGroup runLowerArmRunIntakeAndDrive = new ParallelCommandGroup(
        new ArmIntakePosition(mArmSubsystem).withTimeout(3),
        new DriveTrainMove(m_driveTrain).withTimeout(3),
        new IntakeSetPower(mIntakeSubsystem, 1).withTimeout(3) 
        );

        addCommands(runLowerArmRunIntakeAndDrive);
     */
  }
  
}
