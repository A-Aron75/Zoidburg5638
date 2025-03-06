// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeSubsystemConstants;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
    public final RelativeEncoder m_Encoder;
    private final SparkMax m_WristMotor;
    private SparkMaxConfig config ;
    private SparkClosedLoopController closedLoopController;

  public WristSubsystem() {
        m_WristMotor = new SparkMax(IntakeSubsystemConstants.kWristCanId, MotorType.kBrushless);
       // updateMotorSettings(m_WristMotor); 
       // m_Encoder = m_WristMotor.getEncoder();

    closedLoopController = m_WristMotor.getClosedLoopController();
    m_Encoder = m_WristMotor.getEncoder();

    config = new SparkMaxConfig();

    config.encoder 
          .positionConversionFactor(1)
          .velocityConversionFactor(1);

    config.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(0.1)
          .i(0)
          .d(0)
          .outputRange(-0.25, 0.25)
          .p(0.0001, ClosedLoopSlot.kSlot1)
          .i(0,ClosedLoopSlot.kSlot1)
          .d(0,ClosedLoopSlot.kSlot1)
          .velocityFF(1.0/5767,ClosedLoopSlot.kSlot1)
          .outputRange(-0.25, 0.25,ClosedLoopSlot.kSlot1);

         m_WristMotor.configure(config, null, PersistMode.kNoPersistParameters);

        //  m_WristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
          
          SmartDashboard.setDefaultNumber("Target Position", 0);
          SmartDashboard.setDefaultNumber("Target Velocity", 0);
          SmartDashboard.setDefaultBoolean("Control Mode", false);
          SmartDashboard.setDefaultBoolean("Reset Encoder", false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   /*  if (SmartDashboard.getBoolean("Control Mode", false)) {
      double targetVelocity = SmartDashboard.getNumber("Target Velocity", 0);
      closedLoopController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    } else {
      double targetPosition = SmartDashboard.getNumber("Target Position", 0);
      closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    } */
    SmartDashboard.putNumber("Actual Position", m_Encoder.getPosition());
    SmartDashboard.putNumber("Actual Velocity", m_Encoder.getVelocity());

    if(SmartDashboard.getBoolean("Reset Encoder", false)){
      SmartDashboard.putBoolean("Reset Encoder", false);
      m_Encoder.setPosition(0);
    }
  }
public void WristRest() {
   double targetPosition = Constants.Levels.kWristRestHeight;
 closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
}

public void WristScore() {
  double targetPosition = Constants.Levels.kWristScoreHeight;
closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
}

public void WristIntake() {
  double targetPosition = Constants.Levels.kWristIntakeHeight;
closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
}

public void Wristl4Height() {
  double targetPosition = Constants.Levels.kWristL4Height;
closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
}

}
