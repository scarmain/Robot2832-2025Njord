package frc.robot.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import org.livoniawarriors.motorcontrol.TalonFXMotor;

public class ClimbHw extends Climb {
  private CANcoder climbAngle;
  private TalonFXMotor climbMotor;
  TalonFX rawMotor;
  PositionVoltage positionSetter;
  static final double CLIMB_END_MEASURE = 0.324;

  public ClimbHw() {
    super();
    climbAngle = new CANcoder(54);
    climbMotor = new TalonFXMotor("Climb", 50);
    climbMotor.setBrakeMode(true);

    var fx_cfg = new TalonFXConfiguration();
    fx_cfg.Feedback.FeedbackRemoteSensorID = climbAngle.getDeviceID();
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    fx_cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = CLIMB_END_MEASURE - 0.01;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.01;
    fx_cfg.Slot0.kP = 30;
    fx_cfg.Slot0.kG = 0.5;

    rawMotor = (TalonFX) climbMotor.getBaseMotor();
    rawMotor.getConfigurator().apply(fx_cfg);
    rawMotor.setNeutralMode(NeutralModeValue.Brake);

    positionSetter = new PositionVoltage(0).withUpdateFreqHz(1000);
    // climbMotor.pidConstants.kP = 50;
    // climbMotor.configurePid();

    // sometimes the climb angle starts in a position greater than the allowed
    // absolute range, so we set the position in range
    climbAngle.setPosition(climbAngle.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public void setPower(double power) {
    // climbMotor.setPower(power);
    if (getAngle() < (CLIMB_END_MEASURE - 0.04) || power < 0) {
      climbMotor.setPower(power);
    } else {
      rawMotor.setControl(positionSetter.withPosition(CLIMB_END_MEASURE - 0.02));
    }
  }

  @Override
  public void setAngle(double angle) {}

  @Override
  public double getAngle() {
    var angle = climbAngle.getPosition().getValueAsDouble();
    return MathUtil.inputModulus(angle, -0.5, 0.5);
  }
}
