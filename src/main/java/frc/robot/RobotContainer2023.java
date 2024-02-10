// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
  
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
//import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand; 
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SetSwerveDrive2023;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveDrive2023;
import frc.robot.Constants2023.USB; 
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
 
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer2023 {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrive2023 m_robotDrive = new SwerveDrive2023();
  // Initialize Limelight NetworkTable
    private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
      private final ArmSubsystem m_arm = new ArmSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();




  private final FieldSim m_fieldSim = new FieldSim(m_robotDrive);

  static Joystick leftJoystick = new Joystick(USB.leftJoystick);
  static Joystick rightJoystick = new Joystick(USB.rightJoystick);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer2023() {
     // Initialize Logitech camera
    CameraServer.startAutomaticCapture();  // set the arm subsystem to run the "runAutomatic" function continuously when no other command is running
    m_arm.setDefaultCommand(new RunCommand(() -> m_arm.runAutomatic(), m_arm));

    // set the intake to stop (0 power) when no other command is running
    m_intake.setDefaultCommand(new RunCommand(() -> m_intake.setPower(0.0), m_intake));

    // configure the launcher to stop when no other command is running
    m_launcher.setDefaultCommand(new RunCommand(() -> m_launcher.stopLauncher(), m_launcher));

   final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");



 
    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new SetSwerveDrive2023(
                    m_robotDrive,
                    ()-> leftJoystick.getY(),
                    ()-> leftJoystick.getX(),
                    ()-> leftJoystick.getZ(),
                  true));

    m_fieldSim.initSim();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // set up arm preset positions
    new JoystickButton(rightJoystick, PS4Controller.Button.kSquare.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kScoringPosition)));
//temp    new Trigger(
//temp            () ->
//temp                ((PS4Controller) rightJoystick).getLeftTriggerAxis()
//temp                    > Constants.OIConstants.kTriggerButtonThreshold)
//temp        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kIntakePosition)));
//temp    new JoystickButton(rightJoystick, PS4Controller.Button.kL1.value)
//temp        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kHomePosition)));

    // intake controls (run while button is held down, run retract command once when the button is released)
//temp2    new Trigger(
//temp2            () ->
//temp2                rightJoystick.getRawAxis(PS4Controller.Axis.kL2)
//temp2                    > Constants.OIConstants.kTriggerButtonThreshold)
//temp2        .whileTrue(new RunCommand(() -> m_intake.setPower(Constants.Intake.kIntakePower), m_intake))
//temp2        .onFalse(m_intake.retract());


        // Assuming rightJoystick is an instance of PS4Controller and is already initialized.
        
        // Convert the trigger into a Button object using a lambda for the getRawAxis check.
//temp2Alt        Button triggerButton = new Button();
//temp2Alt        Button(
//temp2Alt            () -> rightJoystick.getRawAxis(PS4Controller.Axis.kL2.value) > Constants.OIConstants.kTriggerButtonThreshold
//temp2Alt        );
        
        // Use the button to run commands
//temp2Alt        triggerButton.whileHeld(new RunCommand(() -> m_intake.setPower(Constants.Intake.kIntakePower), m_intake)); // Replace RunIntakeCommand with your actual intake command
//temp2Alt        triggerButton.whenReleased(); // Replace RetractIntakeCommand with your actual retract command
        

    new JoystickButton(rightJoystick, PS4Controller.Button.kTriangle.value)
        .whileTrue(new RunCommand(() -> m_intake.setPower(-1.0)));

    // launcher controls (button to pre-spin the launcher and button to launch)
    new JoystickButton(rightJoystick, PS4Controller.Button.kCircle.value)
        .whileTrue(new RunCommand(() -> m_launcher.runLauncher(), m_launcher));

    new JoystickButton(rightJoystick, PS4Controller.Button.kR1.value)
        .onTrue(m_intake.feedLauncher(m_launcher));
  

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new WaitCommand(0);
  }
  
  public void periodic() {
    m_fieldSim.periodic();
  }
}