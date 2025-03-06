// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.IntakeSubsystemConstants;

//import com.revrobotics.spark.SparkMax;

/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase {
    SparkMaxConfig config = new SparkMaxConfig();
    private final SparkMax m_CoralMotor;
    private final SparkMax m_leftAlgaeMotor;
    private final SparkMax m_rightAlgaeMotor;
    private final RelativeEncoder m_RelativeEncoder;
    private PIDController m_WristPIDController;
   // private double m_power;



public IntakeSubsystem() {
    m_leftAlgaeMotor = new SparkMax(IntakeSubsystemConstants.kAlgaeLeftCanId, MotorType.kBrushless);
    m_rightAlgaeMotor = new SparkMax(IntakeSubsystemConstants.kAlgaeRightCanId, MotorType.kBrushless);
    m_CoralMotor = new SparkMax(IntakeSubsystemConstants.kCoralCanId, MotorType.kBrushless);

    updateMotorSettings(m_CoralMotor);
    updateMotorSettings(m_leftAlgaeMotor);
    updateMotorSettings(m_rightAlgaeMotor);

    m_WristPIDController = new PIDController(0, 0, 0);
    m_WristPIDController.setTolerance(1.434);
    m_RelativeEncoder = m_CoralMotor.getEncoder();
  }

  public void updateMotorSettings(SparkMax motor) {
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kIntakeMotorsCurrentLimit);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //Algae Controls
    public void IntakeAlgae() {
        m_leftAlgaeMotor.set(0.33);
        m_rightAlgaeMotor.set( -0.33);
      }
      public void ScoreAlgae(){
        m_leftAlgaeMotor.set(-0.33);
        m_rightAlgaeMotor.set(0.33);
      }
      public void AlgaeStop(){
        m_leftAlgaeMotor.set(0.0);
        m_rightAlgaeMotor.set(0.0);
      }

  //Coral controls
      public void IntakeCoral() {
        m_CoralMotor.set(0.18);
      }
      public void ScoreCoral(){
        m_CoralMotor.set(-0.27);
      }
      public void StopCoral(){
        m_CoralMotor.set(0.0);
      }
    
  //Wrist Code


/* 
    public void goToSetpoint(double setpoint) {
      //double feedforward = 0.01;
          //if (m_WristMotor.getAbsoluteEncoder()-setPoint<0.01 && m_WristMotor.getAbsoluteEncoder()-setPoint>-0.01) m_WristMotor.stopWrist();;;
          double speed = m_WristPIDController.calculate(getRelativeEncoderPosition(), setpoint);
          //speed = (speed>0) ? speed + feedforward : speed-feedforward;
          setSpeed(speed);
          System.out.println("PIDElevator output (speed): " + speed + "\nset point: " + m_WristPIDController.getSetpoint() + "\ncurrent position: " + getRelativeEncoderPosition());
    }


     public void setSpeed(double speed) {
      speed = (speed > MotorConstants.kSparkMaxWristMotorsMaxSpeed) ? MotorConstants.kSparkMaxWristMotorsMaxSpeed : speed;
      speed = (speed < -MotorConstants.kSparkMaxWristMotorsMaxSpeed) ? -MotorConstants.kSparkMaxWristMotorsMaxSpeed : speed;
    
      m_WristMotor.set(speed);
    } */
/* 
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist", getRelativeEncoderPosition() );
  } 
    */


  public RelativeEncoder getRelativeEncoder() {
    return m_RelativeEncoder;
  }
  public double getRelativeEncoderPosition() {
    //System.out.println("position is " + m_RelativeEncoder.getPosition());
    return m_RelativeEncoder.getPosition();
  }


/* 
  public void MoveWristUp() {
    m_WristMotor.set(0.2);
  }
  public void MoveWristDown(){
    m_WristMotor.set(-0.2);
  }
  public void StopWrist(){
    m_WristMotor.set(0.0);
  }

public void WristHome(){
  
}

public void WristScorePosition(){

}

public void WristIntakePosition(){}
  //public void setPower(double _power) {
  //  m_power = _power;
 // }




*/
}
