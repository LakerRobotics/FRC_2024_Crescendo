package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;

import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj.interfaces.Gyro; 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;  
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;


public class SwerveSubsystemOLD extends SubsystemBase{
    private final SwerveModule frontLeft = new SwerveModule(
            DriveTrainConstants.kLeftFrontDriveMotorCANID,
            DriveTrainConstants.kLeftFrontStearingMotorCANID,
            DriveTrainConstants.kFrontLeftDriveMotorReversed,
            DriveTrainConstants.kFrontLeftSteeringMotorReversed,
            DriveTrainConstants.kLeftFrontEncoderCANID,
            DriveTrainConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveTrainConstants.kFrontLeftDriveAbsoluteEncoderReversed);    

    private final SwerveModule frontRight = new SwerveModule(    
        DriveTrainConstants.kRightFrontDriveMotorCANID,
        DriveTrainConstants.kRightFrontStearingMotorCANID,
        DriveTrainConstants.kFrontRightDriveMotorReversed,
        DriveTrainConstants.kFrontRightSteeringMotorReversed,
        DriveTrainConstants.kRightFrontEncoderCANID,
        DriveTrainConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveTrainConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveTrainConstants.kLeftRearDriveMotorCANID,
        DriveTrainConstants.kLeftRearStearingMotorCANID,
        DriveTrainConstants.kLeftRearDriveMotorReversed,
        DriveTrainConstants.kLeftRearSteeringMotorReversed,
        DriveTrainConstants.kLeftRearEncoderCANID,
        DriveTrainConstants.kLeftRearDriveAbsoluteEncoderOffsetRad,
        DriveTrainConstants.kLeftRearDriveAbsoluteEncoderReversed);    

    private final SwerveModule backRight = new SwerveModule(
        DriveTrainConstants.kRightRearDriveMotorCANID,
        DriveTrainConstants.kRightRearStearingMotorCANID,
        DriveTrainConstants.kRightRearDriveMotorReversed,
        DriveTrainConstants.kRightRearSteeringMotorReversed,
        DriveTrainConstants.kRightRearEncoderCANID,
        DriveTrainConstants.kRightRearDriveAbsoluteEncoderOffsetRad,
        DriveTrainConstants.kRightRearDriveAbsoluteEncoderReversed);

//OLDProblem        private final Translation2d m_frontLeftLocation = new Translation2d(x:0.381, y:0.381);
//OLDProblem        private final Translation2d m_frontRightLocation = new Translation2d(x:0.381, -0.381);
//OLDProblem        private final Translation2d m_backLeftLocation = new Translation2d(-0.381, y:0.381);
//OLDProblem        private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
     
//OLDProblem        private final SwerveModule m_frontLeft = new SwerveModule(1, 2, 0, 1, 2, 3);
//OLDProblem        private final SwerveModule m_frontRight = new SwerveModule(3, 4, 4, 5, 6, 7);
//OLDProblem        private final SwerveModule m_backLeft = new SwerveModule(5, 6, 8, 9, 10, 11);
//OLDProblem        private final SwerveModule m_backRight = new SwerveModule(7, 8, 12, 13, 14, 15);
      
        //    private AHRS gryo = new AHRS(SPI.port.kMXP);
    private ADIS16470_IMU gyro = new ADIS16470_IMU();
 
//OLDProblem    private final SwerveDriveKinematics m_kinematics =
//OLDProblem    new SwerveDriveKinematics(
//OLDProblem        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);


//OLDProblemprivate final SwerveDriveOdometry odometer = 
//OLDProblem    new SwerveDriveOdometry(
//OLDProblem        m_kinematics,
//OLDProblem        gyro.getRotation2d(),
//OLDProblem        new SwerveModulePosition[] {
//OLDProblem            frontLeft.getPosition(),
//OLDProblem            frontRight.getPosition(),
//OLDProblem            backLeft.getPosition(),
//OLDProblem            backRight.getPosition()
//OLDProblem    }, new Pose2d(5.0, 13.5, new Rotation2d()));
  
//OLDProblemnew SwerveDriveOdometry(DriveTrainConstants.kDriveKinematics, new Rotation2d(0), null);

    public SwerveSubsystemOLD() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
        gyro.calibrate();
    }

    public double getHeading() {
        //return Math.IEEEremainder(gyro.getAngle(), 360);
        return gyro.getAngle(); 
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return null; ////OLDProblem
//OLDProblem        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
//OLDProblem        odometer.resetPosition(pose, getRotation2d());
    }

    double temp= 0;
    @Override
    public void periodic() {
//OLDProblem        odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState(),
        SmartDashboard.putNumber("Robot Heading", getHeading());
        temp = temp+0.1;
        SmartDashboard.putNumber("Test", temp);
        SmartDashboard.putNumber("backleftEncoder", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("backLeftDrivePosition", backLeft.getDrivePosition());
        SmartDashboard.putNumber("backLeftTurningPosition", backLeft.getTurningPosition());
        SmartDashboard.putNumber("backrightEncoder", backRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("backRightDrivePosition", backRight.getDrivePosition());
        SmartDashboard.putNumber("backRightTurningPosition", backRight.getTurningPosition());
        SmartDashboard.putNumber("frontleftEncoder", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("frontLeftDrivePosition", frontLeft.getDrivePosition());
        SmartDashboard.putNumber("frontLeftTurningPosition", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("frontrightEncoder", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("frontRightDrivePosition", frontRight.getDrivePosition());
        SmartDashboard.putNumber("frontRightTurningPosition", frontRight.getTurningPosition());
        SmartDashboard.putNumber("gyro angle", gyro.getAngle());

    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveTrainConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}