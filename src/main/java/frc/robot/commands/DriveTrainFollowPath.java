package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import pathplanner.PathPlanner;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class DriveTrainFollowPath extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final String pathName;
    private SwerveControllerCommand swerveControllerCommand;

    public DriveTrainFollowPath(DriveSubsystem driveSubsystem, String pathName) {
        this.driveSubsystem = driveSubsystem;
        this.pathName = pathName;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        Trajectory trajectory = null;
        //PathPlanner.loadPath(pathName, DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared);

        swerveControllerCommand =
        null;
/*         new SwerveControllerCommand(
            trajectory,
            driveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new PIDController(AutoConstants.kPThetaController, 0, 0),
            driveSubsystem::setModuleStates,
            driveSubsystem
        );
*/
        // Optionally reset odometry to the starting pose of the trajectory
        driveSubsystem.resetOdometry(trajectory.getInitialPose());
    }

    @Override
    public void execute() {
        swerveControllerCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return swerveControllerCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        swerveControllerCommand.end(interrupted);
    }
}
