package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.DriveSubsystem;
import java.nio.file.Path;
import java.io.IOException;

public class DriveTrainFollowPath extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private Trajectory trajectory;
    private final String trajectoryJSON;

    public DriveTrainFollowPath(DriveSubsystem subsystem, String trajectoryPath) {
        this.driveSubsystem = subsystem;
        this.trajectoryJSON = trajectoryPath;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        try {
            Path trajectoryFilePath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryFilePath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        driveSubsystem.resetOdometry(trajectory.getInitialPose());
    }

    @Override
    public void execute() {
        var timeSinceInit = timeSinceInitialized();
        if (timeSinceInit < trajectory.getTotalTimeSeconds()) {
            var desiredPose = trajectory.sample(timeSinceInit);
            driveSubsystem.followTrajectory(desiredPose);
        }
    }

    @Override
    public boolean isFinished() {
        return timeSinceInitialized() >= trajectory.getTotalTimeSeconds();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopModules();
    }
}
