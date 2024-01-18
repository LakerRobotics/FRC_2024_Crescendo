package frc.robot.commands;
//import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

/**
 *
 */
public class DriveTrainFieldOrientated extends Command {
    private final DriveTrain m_DriveTrain;

    Gyro m_rotationSource;

	protected Gyro m_TurnSource;
    private double m_maxspeed;

//    private SimpleMotorFeedforward m_simpleMotorFeedForward;

    public final double StraightKp = 0.006;// 0.020;
    public final double StraightKi = 0.008;//0.001;
    public final double StraightKd = 0.0;

//    private PIDController anglePIDController;
//    private edu.wpi.first.math.controller.PIDController speedPIDController;
    private SimpleMotorFeedforward speedFeedForwardController;
//    private final double StraightMaxPower = 1;

/** 
    * @param theDriveTrain the drivetrain subsystem
    ------------------------------------------------*/
   public DriveTrainFieldOrientated(DriveTrain theDriveTrain){

        m_DriveTrain = theDriveTrain;
        addRequirements(m_DriveTrain);
        m_rotationSource = theDriveTrain.getGyro();
     
        m_TurnSource = m_rotationSource;
        m_maxspeed = 15; //ft/sec

    }
     


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
	{
//        angleStick = new Joystick(0);
//        powerStick = new Joystick(1);
//        gyro = new ADXRS450_Gyro();
        speedFeedForwardController =  new SimpleMotorFeedforward(Constants.DriveTrainConstants.ksVolts, 
                                                               Constants.DriveTrainConstants.kvVoltSecondsPerMeter,
                                                               Constants.DriveTrainConstants.kaVoltSecondsSquaredPerMeter);


//		double convertedSpeed = m_maxspeed * 12; 	// Converted from Feet/Second to Inches/Second
			
//		}
    }
}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double desiredAngle = getJoystickAngle();
        double maxTurnPower = getJoystickAngleLength();
        double desiredSpeed = RobotContainer.getInstance().getDriverController().getRightX() * m_maxspeed; // convert joystick value to ft/sec

        SmartDashboard.putNumber("FieldDrive Desired Angle", desiredAngle);
        SmartDashboard.putNumber("FieldDrive max Turn Power", maxTurnPower);
        SmartDashboard.putNumber("FieldDrive Desired Speed", desiredSpeed);

        drive(desiredAngle, maxTurnPower, desiredSpeed);
    }

    private void drive(double desiredAngle, double maxTurnPower, double desiredSpeed) {
        double currentAngle = m_DriveTrain.getHeading();
        double currentSpeed = m_DriveTrain.getSpeed();

        double turnError = desiredAngle-currentAngle;
        if(java.lang.Math.abs(turnError) > 180){                    //Test cases desired angle 1, current angle 359, in this case want answer to be +2
                                                //Test Case desired angle 180 current angle 0, then want 180
                                                //Test Case desired angle 1 current angle 182 then want 179
                                                //Test Case desired angle 1 current angle 179 then want -178
                                                //Test Case desired angle 90 current angle 270 then want +180
                                                //Test Case desired angle 270 current angle 1 then want -91
            if(turnError>0){
                turnError =  360-turnError; 
            }else{ // turnAngleIsNegative
                turnError = -360+turnError;
            };
        }
//        double turnPower = anglePIDController.calculate(currentAngle, desiredAngle) + turnError * 1/25; /*so if 25 Degrees off, so about 1/16 or a full turn, then apply full power to turning */
        double turnPower =  maxTurnPower *  turnError * 1/25; /*so if 25 Degrees off, so about 1/16 or a full turn, then apply full power to turning
                                                                  NO negative because one test it just spun in circles so perhapes gyro rate and turn power are already opposit each other
                                                                  (had had negative turnPower but one testit just spun in circles)*/
        double forwardPower = speedFeedForwardController.calculate(currentSpeed, desiredSpeed);
        // Drive the robot based on the desired angle and speed
        m_DriveTrain.arcadeDrive(forwardPower, turnPower);
      }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }

    private double getJoystickAngle() {
        double x = RobotContainer.getInstance().getDriverController().getLeftX();// Angle() getDriverController().getLeftY();
        double y = RobotContainer.getInstance().getDriverController().getLeftY(); /*turnpower*/
    
        //double angle =  RobotContainer.getInstance().getDriverController().getAngle()
        double angle = Math.toDegrees(Math.atan2(y, x));
    
        // Handle transition from 360 to 0 degrees
        if (angle < 0) {
          angle += 360;
        }
    
        return angle;
      }
      private double getJoystickAngleLength() {
        double x = RobotContainer.getInstance().getDriverController().getLeftX();// Angle() getDriverController().getLeftY();
        double y = RobotContainer.getInstance().getDriverController().getLeftY(); /*turnpower*/
        double length = java.lang.Math.sqrt(x*x +y*y);
        return length;
      }
    
    
    
}
