// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  private  SparkMax  m_ClimberMotorL;
  private  SparkMax  m_ClimberMotorR;

  public ClimbSubsystem() {
  m_ClimberMotorL = new SparkMax(ClimbConstants.kClimbMotorLeftCANID, MotorType.kBrushless);
  m_ClimberMotorR = new SparkMax(ClimbConstants.kClimbMotorRightCANID, MotorType.kBrushless);  
  }

  public void ClimberUp() {
    m_ClimberMotorR.set(0.75);
    m_ClimberMotorL.set( -0.75);
  }

  public void ClimberDown(){
    m_ClimberMotorR.set(-0.15);
    m_ClimberMotorL.set(0.15);
  }

  public void ClimberStop(){
    m_ClimberMotorR.set(0.0);
    m_ClimberMotorL.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
