package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.CANcoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANEncoder;
//import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.hal.simulation.CTREPCMDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {
  
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final Encoder driveEncoder;
    private final Encoder turningEncoder;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule (int driveMotorId, 
        int turningMotorId, 
        boolean driveMotorReversed, 
        boolean turningMotorReversed,
        int absoluteEncoderId, 
        double absoluteEncoderOffset, 
        boolean absoluteEncoderReversed) {
            driveEncoder = null;
            turningEncoder = null;

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        //absoluteEncoder = new DutyCycleEncoder(absoluteEncoderId);
        absoluteEncoder = new CANcoder(absoluteEncoderId) {
            
        };
        
        //new SparkMaxAnalogSensor(temp,SparkMaxAnalogSensor.Mode.kAbsolute);// ( absoluteEncoderId);
    
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
        
//OLDProblem        driveEncoder = driveMotor.getAbsoluteEncoder(Type.kDutyCycle);
//OLDProblem        turningEncoder = turningMotor.getEncoder();
        
//OLDProblem        driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kDriveEncoderRot2Meter);
//OLDProblem        driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveEncoderRPM2MeterPerSec);
//OLDProblem        turningEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurningEncoderRot2Rad);
//OLDProblem        turningEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningEncoderRPM2RadPerSec);
    
        turningPidController = new PIDController(SwerveModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        
    }



public double getDrivePosition() {
//    return driveEncoder.getPosition();
return 0;
}     

public double getTurningPosition() {
//OLDProblem  return turningEncoder.getPosition();
  return 0;
}

public double getDriveVelocity() {
//OLDProblem  return turningEncoder.getVelocity();
return 0;
}
   
public double getTurningVelocity() {
//OLDProblem    return turningEncoder.getVelocity();
return 0;
}

public double getAbsoluteEncoderRad() {
    //double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    double angle = 0;// TEMP TODO
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);

}

public void resetEncoders() {
//OLDProblem    driveEncoder.setPosition(0);
//OLDProblem    turningEncoder.setPosition(getAbsoluteEncoderRad());
}

public SwerveModuleState getState() {
    return null;
//    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
}

public void setDesiredState(SwerveModuleState state) {
    SmartDashboard.putNumber("setdesiredState", state.speedMetersPerSecond);
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
        stop();
        return;
    }
     
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveTrainConstants.kPhysicalMaxSpeedMetersPerSecond);
    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getAbsolutePosition() + "] state", state.toString());

}

public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
    
}



public SwerveModulePosition getPosition() {
//OLDProblem    return new SwerveModulePosition(
//OLDProblem        driveEncoder.getDistance(), new Rotation2d(turningEncoder.getDistance()));
//OLDProblem}
return null;
}
}

