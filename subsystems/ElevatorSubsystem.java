package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
//import com.revrobotics.spark.SparkBase;

//import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.SparkMax;
//import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

//import com.revrobotics.jni.CANSparkJNI;
//import com.revrobotics.spark.config.SmartMotionConfig;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;



public class ElevatorSubsystem extends SubsystemBase {
    public final SparkMax m_ElevatorMotor1;
    private final SparkMax m_ElevatorMotor2;
    public final RelativeEncoder m_RelativeEncoder1;

   // private final DigitalInput topLimitSwitch;
    //private final DigitalInput bottomLimitSwitch;
    private PIDController m_ElevatorPIDController;
    SparkMaxConfig config = new SparkMaxConfig();

   // private double m_setpoint;
    //private TrapezoidProfile.State m_startState;
    //private TrapezoidProfile.State m_endState;
    //private TrapezoidProfile m_Profile;

  /** Creates a new ExampleSubsystem. */
  @SuppressWarnings("deprecation")
  public ElevatorSubsystem() {
    m_ElevatorMotor1 = new SparkMax(MotorConstants.kSparkMaxElevatorMotor1CANID, MotorType.kBrushless);
    m_ElevatorMotor2 = new SparkMax(MotorConstants.kSparkMaxElevatorMotor2CANID, MotorType.kBrushless);
    updateMotorSettings(m_ElevatorMotor1);
    updateMotorSettings(m_ElevatorMotor2);
    m_RelativeEncoder1 = m_ElevatorMotor1.getEncoder();

    m_ElevatorMotor2.setInverted(true);
    m_ElevatorMotor2.isFollower();

    //topLimitSwitch = new DigitalInput(1);
    //bottomLimitSwitch = new DigitalInput(0);

    m_ElevatorPIDController = new PIDController(0.04, 0, 0);
    //m_ElevatorPIDController.enableContinuousInput(0, 1);
    m_ElevatorPIDController.setTolerance(1.434);

  //  m_setpoint = Constants.Levels.ElevatorHomeHeight;

   


  }



   public void updateMotorSettings(SparkMax motor) {
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kIntakeMotorsCurrentLimit);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

 // public void goToSetpoint(double setpoint) {
       // double speed = m_ElevatorPIDController.calculate(getRelativeEncoderPosition(), setpoint);
       // setSpeed(speed);
        //System.out.println("PIDElevator output (speed): " + speed + "\nset point: " + m_ElevatorPIDController.getSetpoint() + "\ncurrent position: " + getRelativeEncoderPosition());
 // }

  public boolean atSetpoint() {
    return m_ElevatorPIDController.atSetpoint();
  }

  /*public void setSpeed(double speed) {
    speed = (speed > MotorConstants.kSparkMaxElevatorMotorsMaxSpeed) ? MotorConstants.kSparkMaxElevatorMotorsMaxSpeed : speed;
    speed = (speed < -MotorConstants.kSparkMaxElevatorMotorsMaxSpeed) ? -MotorConstants.kSparkMaxElevatorMotorsMaxSpeed : speed;
    //System.out.println("speed: " + speed);
    speed = ((!topLimitSwitch.get() && speed > 0) || (bottomLimitSwitch.get() && speed < 0)) ? 0 : speed;
    //System.out.println("speed: " + speed + "\n" + bottomLimitSwitch.get() + " "  + topLimitSwitch.get());

    m_ElevatorMotor1.set(speed);
   m_ElevatorMotor2.set(speed); 
  } */


  
  public void stopElevatorMotors() {
    m_ElevatorMotor1.stopMotor();
   m_ElevatorMotor2.stopMotor();
  }
  public void ElevatorUp() {
    m_ElevatorMotor1.set(0.25);
   m_ElevatorMotor2.set( 0.25);
  }
  public void ElevatorDown(){
    m_ElevatorMotor1.set(-0.25);
   m_ElevatorMotor2.set(-0.25);

  }
  public void ElevatorStop(){
    m_ElevatorMotor1.set(0.0);
   m_ElevatorMotor2.set(0.0);
  }

  public RelativeEncoder getRelativeEncoder() {
    return m_RelativeEncoder1;
  }

  public double getRelativeEncoderPosition() {
    //System.out.println("position is " + m_RelativeEncoder.getPosition());
    return m_RelativeEncoder1.getPosition();
  }

  public void zeroEncoder() {
    m_RelativeEncoder1.setPosition(0);
    System.out.println("zero elevator encoder!!");
  }

 /*  public boolean getTopLimitSwitchState() {
    return topLimitSwitch.get();
  }

  public boolean getBottomLimitSwitchState() {
    return bottomLimitSwitch.get();
  } */

//public void ElevatorHomeHeight(){m_RelativeEncoder1.setReference(setPoint, SparkBase.ControlType.kSmartMotion);}

/* 
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator", getRelativeEncoderPosition());
    if (bottomLimitSwitch.get()) {zeroEncoder();}
  } */

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }



  //m_controller.setReference(setPoint, SparkBase.ControlType.kSmartMotion);

  /*  public void setTargetPosition(double _setpoint) {
    if (_setpoint != m_setpoint) {
      m_setpoint = _setpoint;
      updateMotionProfile();
    }
  } */

/*   private void updateMotionProfile(){
    m_startState = new TrapezoidProfile.State(m_RelativeEncoder1.getPosition(),m_RelativeEncoder1.getVelocity());
    m_endState = new TrapezoidProfile.State(m_setpoint, 0.0);
    m_Profile = new TrapezoidProfile(Constants.Levels.kElevatorMotionConstraints);
  } */

 /*public void runManual(double _power) {
    // reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and
    // passively
    m_setpoint = m_encoder.getPosition();
    updateMotionProfile();
    // update the feedforward variable with the newly zero target velocity
    m_feedforward =
        Constants.Arm.kArmFeedforward.calculate(
            m_encoder.getPosition() + Constants.Arm.kArmZeroCosineOffset, m_targetState.velocity);
    // set the power of the motor
    m_motor.set(_power + (m_feedforward / 12.0));
    m_manualValue = _power; // this variable is only used for logging or debugging if needed
  } */
}