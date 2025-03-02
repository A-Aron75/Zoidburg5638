// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  
  public static final class DriveConstants {
    // Driving Parameters -  that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.0;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(29);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    //This may need to be changed later
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;
// your the best
    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 6;
    public static final int kRearLeftDrivingCanId = 8;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 2;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 1;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;

   
   
      //public static final double kXDriveDeadband = 0.25;  //GF was .25
      public static final double kTriggerButtonThreshold = 0.5;
  
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }


  //Ella Vader (HAHAHAHA) Code beneath here

  public static class MotorConstants {
    public static final int kSparkMaxElevatorMotor1CANID = 10;
    public static final int kSparkMaxElevatorMotor2CANID = 9;

    public static final int kSparkMaxElevatorMotorsCurrentLimit = 40;
    public static final double kSparkMaxElevatorMotorsSpeed = .65;
    public static final double kSparkMaxElevatorMotorsMaxSpeed = 0.95;


    public static final int kIntakeMotorsCurrentLimit = 40;
    public static final double kIntakeMotorsSpeed = .5;
    public static final double kIntakeMotorsMaxSpeed = .6;
    
    public static final double kWristIntakeSpeed = .18;
    public static final double kWristScoreSpeed = -0.33;

    public static final double kSparkMaxWristMotorsSpeed = .2;
    public static final double kSparkMaxWristMotorsMaxSpeed = 0.3;
    public static final double kWristHomePosition = 0.0;
    public static final double kScoringPosition = 10.0;
    public static final double kIntakePosition = 5.0;

  }


  public static final class IntakeSubsystemConstants {
    public static final int kAlgaeLeftCanId = 12;
    public static final int kAlgaeRightCanId= 11;
    public static final int kWristCanId = 13;
    public static final int kCoralCanId = 14;

  }

  public static final class ClimbConstants{
    public static final int kClimbMotorLeftCANID  = 16;
    public static final int kClimbMotorRightCANID = 15; 

    public static final int kSparkMaxClimberMotorsCurrentLimit = 40;
    public static final double kSparkMaxClimberMotorsSpeed = .35;
    public static final double kSparkMaxClimberMotorsMaxSpeed = 0.65;
    public static final int kClimbMotorsCurrentLimit = 40;

  }

  public static final class Levels{
   // public static final int kCoralL1Height= ;
   // public static final int kCoralL2Height = ;
  //  public static final int kCoralL3Height = ;
   // public static final int kCoralL4Height = ;
   // public static final int kIntakeHeight = ;

    public static final int ElevatorHomeHeight = 0;
   // public static final int AlgaeL1Height = ;
    public static final int AlgaeL2Height = 54;


   // public static final TrapezoidProfile.Constraints kElevatorMotionConstraints =
   // new TrapezoidProfile.Constraints(1.0, 2.0);
   }
    public static final double kMaxElevatorHeight = 68;
    public static final double kMinElevatorHeight = 0;

     private static final double ElevatorKp = 0.1;
   
   public static double controlLoop(double measurement, double setpoint){
   double error = setpoint - measurement;
   double output = error * ElevatorKp;  

   //Feed Forward
   //Helps account for gravity
   double feedForward = 0.1;
   output = output + feedForward;

   //Tolerance
   //Lets feed forward NOT continuously take power.
   double tolerance = 0.5; //Tolerance for control loop in inches
   if (Math.abs(error) < Units.inchesToMeters(tolerance)){
     output = 0;
   }

       //Minimum Output
       double sign = Math.signum(output);
       double minOutput = 0.05;
       if (Math.abs(output) < minOutput){
         output = minOutput * sign;
       }
   
       //Min and max values
       //If we are at our min value don't let us drive down, if we are at max don't let us drive up
       if (output < (0 + feedForward) && measurement < kMinElevatorHeight){
         output = 0;
       }
       if (output > 0 && measurement > kMaxElevatorHeight ){
         output = 0;
       }
   return output;
}

}
