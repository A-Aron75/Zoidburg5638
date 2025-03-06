// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
//import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;

public class Elevator2Subsystem extends SubsystemBase {

    public final SparkMax ElevatorMotor1;
    public final SparkMax ElevatorMotor2;
    public final RelativeEncoder m_Encoder1;
    public final RelativeEncoder m_Encoder2;


    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;
   // private PIDController m_ElevatorPIDController;
    private SparkMaxConfig config ;
    private SparkClosedLoopController closedLoopController;

  /** Creates a new Elevator2Subsystem. */
  public Elevator2Subsystem() {
   ElevatorMotor1 = new SparkMax(MotorConstants.kSparkMaxElevator2Motor1CANID, MotorType.kBrushless);
   ElevatorMotor2 = new SparkMax(MotorConstants.kSparkMaxElevator2Motor2CANID, MotorType.kBrushless);

   ElevatorMotor2.setInverted(true);

    topLimitSwitch = new DigitalInput(1);
    bottomLimitSwitch = new DigitalInput(0);
    
    closedLoopController = ElevatorMotor1.getClosedLoopController();
    closedLoopController = ElevatorMotor2.getClosedLoopController();

   m_Encoder1 = ElevatorMotor1.getEncoder();
   m_Encoder2 = ElevatorMotor2.getEncoder();
    config = new SparkMaxConfig();

    config.encoder 
          .positionConversionFactor(1)
          .velocityConversionFactor(1);

    config.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(0.4)// was 0.1
          .i(0)
          .d(0)
          .outputRange(-1, 1)
          .p(1, ClosedLoopSlot.kSlot1)
          .i(0,ClosedLoopSlot.kSlot1)
          .d(0,ClosedLoopSlot.kSlot1)
          .velocityFF(1.0/5767,ClosedLoopSlot.kSlot1) //was 5767
          .outputRange(-1, 1,ClosedLoopSlot.kSlot1);

       ElevatorMotor1.configure(config, null, PersistMode.kNoPersistParameters);
       ElevatorMotor2.configure(config, null, PersistMode.kNoPersistParameters);


  /*   config.closedLoop.maxMotion
    .maxVelocity(1000)
    .maxAcceleration(1000)
    .allowedClosedLoopError(1)
    .maxAcceleration(500, ClosedLoopSlot.kSlot1)
    .maxVelocity(6000, ClosedLoopSlot.kSlot1)
    .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);
*/
        //  ElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
          
          SmartDashboard.setDefaultNumber("Target Position", 0);
          SmartDashboard.setDefaultNumber("Target Velocity", 0);
          SmartDashboard.setDefaultBoolean("Control Mode", false);
          SmartDashboard.setDefaultBoolean("Reset Encoder", false);
  }

  public void setSpeed(double speed) {
    speed = (speed > MotorConstants.kSparkMaxElevatorMotorsMaxSpeed) ? MotorConstants.kSparkMaxElevatorMotorsMaxSpeed : speed;
    speed = (speed < -MotorConstants.kSparkMaxElevatorMotorsMaxSpeed) ? -MotorConstants.kSparkMaxElevatorMotorsMaxSpeed : speed;
    //System.out.println("speed: " + speed);
    speed = ((!topLimitSwitch.get() && speed > 0) || (bottomLimitSwitch.get() && speed < 0)) ? 0 : speed;
    //System.out.println("speed: " + speed + "\n" + bottomLimitSwitch.get() + " "  + topLimitSwitch.get());

    ElevatorMotor1.set(speed);
    ElevatorMotor2.set(speed);
  }

  public boolean getTopLimitSwitchState() {
    return topLimitSwitch.get();
  }

  public boolean getBottomLimitSwitchState() {
    return bottomLimitSwitch.get();
  }

  public void zeroEncoder() {
    m_Encoder1.setPosition(0);
    m_Encoder2.setPosition(0);

    System.out.println("zero elevator encoder!!");
  }

  public double getRelativeEncoderPosition1() {
    return m_Encoder1.getPosition(); }

  public double getRelativeEncoderPosition2() {
    return m_Encoder2.getPosition();
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
    SmartDashboard.putNumber("Actual Position1", m_Encoder1.getPosition());
    SmartDashboard.putNumber("Actual Velocity1", m_Encoder1.getVelocity());
    SmartDashboard.putNumber("Actual Position2", m_Encoder2.getPosition());
    SmartDashboard.putNumber("Actual Velocity2", m_Encoder2.getVelocity());
    SmartDashboard.putNumber("Elevator1", getRelativeEncoderPosition1());
    SmartDashboard.putNumber("Elevator2", getRelativeEncoderPosition2());

    if (bottomLimitSwitch.get()) {zeroEncoder();}

    if(SmartDashboard.getBoolean("Reset Encoder", false)){
      SmartDashboard.putBoolean("Reset Encoder", false);
    m_Encoder1.setPosition(0);
    m_Encoder2.setPosition(0);
    }
  }

  public void CoralL1Height(){
 double targetPosition = Constants.Levels.kCoralL1Height;
 closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);

}

public void CoralL2Height(){
  double targetPosition = Constants.Levels.kCoralL2Height;
  closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
}
 public void CoralL3Height(){
  double targetPosition = Constants.Levels.kCoralL3Height;
  closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
 }
 public void CoralL4Height(){
  double targetPosition = Constants.Levels.kCoralL4Height;
  closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
 }
 public void IntakeHeight(){
  double targetPosition = Constants.Levels.kIntakeHeight;
  closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
 }
 public void AlgaeL1Height(){
  double targetPosition = Constants.Levels.AlgaeL1Height;
  closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
 }
public void AlgaeL2Height(){
  double targetPosition = Constants.Levels.AlgaeL2Height;
  closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
}
 public void ElevatorHomeHeight(){
  double targetPosition = Constants.Levels.ElevatorHomeHeight;
  closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
 }

 public void ElevatorUp() {
  ElevatorMotor1.set(0.25);
  ElevatorMotor2.set( 0.25);
}
public void ElevatorDown(){
  ElevatorMotor1.set(-0.25);
  ElevatorMotor2.set(-0.25);

}
public void ElevatorStop(){
  ElevatorMotor1.set(0.0);
  ElevatorMotor2.set(0.0);
}


}
