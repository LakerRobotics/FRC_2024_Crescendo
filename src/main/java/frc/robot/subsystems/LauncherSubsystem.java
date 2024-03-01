package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.LauncherAutoPower;

public class LauncherSubsystem extends SubsystemBase {

  private CANSparkMax m_topMotor;
  private CANSparkMax m_bottomMotor;

  private boolean m_launcherRunning;

  double m_power_top = Constants.Launcher.kBottomPower;
  double m_power_bottom = Constants.Launcher.kBottomPower;
        
  /**
   * Creates a new LauncherSubsystem.
   */
  public LauncherSubsystem() {
    // create two new SPARK MAXs and configure them
    m_topMotor =
        new CANSparkMax(Constants.Launcher.kTopCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_topMotor.setInverted(true);
    m_topMotor.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    m_topMotor.setIdleMode(IdleMode.kBrake);

    m_topMotor.burnFlash();

    m_bottomMotor =
        new CANSparkMax(Constants.Launcher.kBottomCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_bottomMotor.setInverted(true);
    m_bottomMotor.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    m_bottomMotor.setIdleMode(IdleMode.kBrake);

    m_bottomMotor.burnFlash();

    m_launcherRunning = false;

    setDefaultCommand(new LauncherAutoPower(this,0,0));
  }

  /**
   * Turns the launcher on.  Can be run once and the launcher will stay running or run continuously in a {@code RunCommand}.
   */
  public void runLauncher() {
    m_launcherRunning = true;
  }

  /**
   * Turns the launcher off.  Can be run once and the launcher will stay running or run continuously in a {@code RunCommand}.
   */
  public void stopLauncher() {
    m_launcherRunning = false;
  }

  public void setPower(double power_top, double power_bottom){
    m_power_top = power_top;
    m_power_bottom = power_bottom;
  }

  @Override
  public void periodic() {  // this method will be called once per scheduler run
    // set the launcher motor powers based on whether the launcher is on or not
    if (m_launcherRunning) {
      
      m_topMotor.set(m_power_top);
      m_bottomMotor.set(m_power_bottom);
    } else {
      m_topMotor.set(0.0);
      m_bottomMotor.set(0.0);
    }
  }
}