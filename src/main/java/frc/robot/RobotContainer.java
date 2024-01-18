package frc.robot;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
  
import edu.wpi.first.wpilibj2.command.Command; 
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
  
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.simulation.FieldSim;
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer;
  // The robot's subsystems
  // The robot's subsystems and commands are defined here...
  private final SwerveDrive2023 m_robotDrive = new SwerveDrive2023();

  private final FieldSim m_fieldSim = new FieldSim(m_robotDrive);

  //2023 static Joystick leftJoystick = new Joystick(USB.leftJoystick);
  //2023 static Joystick rightJoystick = new Joystick(USB.rightJoystick);

  public final SwerveSubsystemOLD m_swerveSubsystem = new SwerveSubsystemOLD();
    //public final Climber m_climber = new Climber();
    public final DriveTrain m_driveTrain = null; //new DriveTrain();

  // Joysticks
    private final PS4Controller driverController = new PS4Controller(0);
    private final PS4Controller operatorController = new PS4Controller(1);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  public RobotContainer() {
      m_swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        m_swerveSubsystem,
                () -> -driverController.getRawAxis(OperatorConstants.kDriverYAxis),
                () -> driverController.getRawAxis(OperatorConstants.kDriverXAxis),
                () -> driverController.getRawAxis(OperatorConstants.kDriverRotAxis),
                () -> !driverController.getRawButton(OperatorConstants.kDriverFieldOrientedButtonIdx)));
                                
    // Smartdashboard Subsystems

    //m_chooser.addOption("CurvedPath", loadPathplannerTrajectoryToRamseteCommand(
    //     "C:\\Users\\Laker-Programming\\FRC2023ChargedUp3\\src\\main\\deploy\\pathplanner\\generatedJSON\\Curved Path.wpilib.json",
    //    "pathplanner/generatedJSON/Curved Path.wpilib.json",
    //    true));
      
    //  m_chooser.addOption("Place Cone", new AutonomousPlaceCone(m_intake, m_arm, m_driveTrain));
    //  m_chooser.addOption("Place Cone and Balance", new AutonomousPlaceConeBalance(m_intake, m_arm, m_driveTrain));     
  
    Shuffleboard.getTab("Autonomous").add(m_chooser);

    //SmartDashboard Buttons
    //SmartDashboard.putData("AutonomousCommand", new AutonomousShootandBackupStraight(m_intake,m_shooter,m_conveyor,m_driveTrain));
    //SmartDashboard.putData("DriveTrainArcade", new DriveTrainArcade( m_driveTrain ));

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    //m_climber.setDefaultCommand(new ClimberMove( m_climber ) );
    //m_driveTrain.setDefaultCommand(new DriveTrainFieldOrientated(m_driveTrain ) );

    // Configure autonomous sendable chooser

    //m_chooser.setDefaultOption("$command.getName()", new ${name.replace(' ', '')}( m_${name.substring(0,1).toLowerCase()}${name.substring(1).replace(' ', '')} ));
    //m_chooser.setDefaultOption("AutnomouseShootAndBackup",         new AutonomousShootandBackup(         m_intake, m_shooter, m_conveyor, m_driveTrain));
    //m_chooser.addOption(       "AutnomouseShootAndBackup",         new AutonomousShootandBackup(         m_intake, m_shooter, m_conveyor, m_driveTrain));

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }
/* 
  public Command loadPathplannerTrajectoryToRamseteCommand(String filename, boolean resetOdometry){
    Trajectory trajectory;
    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }catch(IOException exception){
      DriverStation.reportError("Unable to open trajectory" + filename,exception.getStackTrace());
      System.out.println("Unable to read from file" + filename);
      return new InstantCommand();
    }
    */
/* 
    RamseteCommand ramseteCommand =
     new RamseteCommand(trajectory, m_driveTrain::getPose, 
                        new RamseteController(DriveTrainConstants.kRamsetB, DriveTrainConstants.kRamseteZeta), 
                        new SimpleMotorFeedforward(DriveTrainConstants.ksVolts, DriveTrainConstants.kvVoltSecondsPerMeter,DriveTrainConstants.kaVoltSecondsSquaredPerMeter), 
                        DriveTrainConstants.kDriveKinematics,
                        m_driveTrain::getWheelSpeeds, 
                        new PIDController(DriveTrainConstants.kpDriveVel, 0, 0), 
                        new PIDController(DriveTrainConstants.kpDriveVel, 0, 0), 
                        m_driveTrain::tankDriveVolts, m_driveTrain
                       );

    if(resetOdometry){
      return new SequentialCommandGroup(new InstantCommand(()->m_driveTrain.resetOdometry(trajectory.getInitialPose())),ramseteCommand);
    }else{
      return ramseteCommand;
    }
    
  }
  */

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // set default command
    //m_driveTrain.setDefaultCommand(new DriveTrainArcade( m_driveTrain ) );
    // setup some buttons
    //final JoystickButton driveTrainLockButton = new JoystickButton(driverController, PS4Controller.Button.kCircle.value);        
    //                     driveTrainLockButton.whileTrue(new DriveTrainLock(m_driveTrain));
    //m_intake.setDefaultCommand(new IntakeTeleop( m_intake ) );
    /*@deprecated */
    new JoystickButton(driverController,Constants.OperatorConstants.kDriverFieldOrientedButtonIdx).onTrue(getAutonomousCommand());(() -> m_swerveSubsystem.zeroHeading());
  }

      
    // final JoystickButton armHighButton = new JoystickButton(operatorController, PS4Controller.Button.kTriangle.value);        
    //                 armHighButton.whileTrue(new ArmControlHighPosition( m_arm ));
    //
    //SmartDashboard.putData("IntakeConeIn",new IntakeConeIn( m_intake ) );
                   
  

  public PS4Controller getDriverController() {
      return driverController;
  }

  public PS4Controller getOperatorController() {
      return operatorController;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }

  public void periodic() {
    m_fieldSim.periodic();
  }
}

