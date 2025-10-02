// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;

import frc.lib.util.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double stickDeadband = 0.1;

  public static final class Controller {
    //USB port number of the xbox controllers
    public static final int USB_DRIVECONTROLLER = 0;
    public static final int USB_AUXCONTROLLER = 1;
  }

  //AUTO SWITCHES
  public static final int DIO_AUTO_1 = 0;
  public static final int DIO_AUTO_2 = 1;
  public static final int DIO_AUTO_3 = 2;
  public static final int DIO_AUTO_4 = 3;

  public static class MotorControllers {
    public static final int SMART_CURRENT_LIMIT = 40;
   //Motor ID Numbers
    //Elevator 
    public static final int ID_ELEVATOR_LEFT_TALON = 11;
    public static final int ID_ELEVATOR_RIGHT_TALON = 12;
    //AlgaeHold
    public static final int ID_ALGAE_HOLD = 56; 
    //CoralHold
    public static final int ID_CORAL_HOLD_MOTOR = 1;
    //AlgaePivot 
    public static final int ID_ALGAE_PIVOT = 57;
    //CoralPivot 
    public static final int ID_CORAL_PIVOT = 2;//BRUSHED!!! 
  }

public static final class PathPlanner {
  public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(4.5, 0.0, 0.0);
  public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(5.0, 0.0, 0.0);
}

public static final class Swerve {
        public static final int pigeonID = 1; //gryo

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L2);
      
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23.5); //2024 testbed
        public static final double wheelBase = Units.inchesToMeters(23.5); //2024 testbed
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25; //TODO check compared to last year
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */    
        public static final double driveKP = 2.5; //0.5, 1 //TODO: This must be tuned to specific robot, default is 0.1
        public static final double driveKI = 0; //2
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0; 

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0; //0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 0; //1.51;
        public static final double driveKA = 0; //.27; 

        /* Swerve Profiling Values, Meters per Second*/
        public static final double maxSpeed = 2.5; // 4.5 old //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */

        /* FRONT LEFT Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-120.15);//(81.1+180); TESTBED//-119.79
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        /* FRONT RIGHT Module - Module 1 */
            public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(131.045);//(-20.83+180)Estbed; 130.87 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }    
        /* BACK LEFT Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(106.87);//(8.1+180); TESTBED //106.35
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        /* BACK RIGHT Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(130.95);//(-17.75+180);  TESTBED//130.87
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: these are for example auto - must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3.0; //4 //2.
        public static final double kMaxAccelerationMetersPerSecondSquared = 2.0; //4 //2.5
        public static final double kMaxAngularSpeedRadiansPerSecond = 4*Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 4*Math.PI;
        //X = forward, Y = to the left, Theta positive CCW for swerve
        public static final double kPXController = 12; //4//  12 
        public static final double kPYController = 8;//7;//12; //6//TODO: RETUNE!
        public static final double kPThetaController = 10; //TODO: RETUNE!
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

      //Center Auto path (inches)
        public static final double CENTER_FWD_DIST = 53.5; 
    }

  public static class Elevator {
    public static final int DIO_ELEV_TOP = 4;
    public static final int DIO_ELEV_BOTTOM = 5;

    public static final double ELEV_UP_SPEED = 0.1;
    public static final double ELEV_DOWN_SPEED = -0.2;
    public static final double ELEV_CLIMB_DOWN_SPEED = -0.2;

    //conversion factors
    public static final double ELEV_REV_TO_METERS = 1.362*0.0254;
    public static final double ELEV_REV_TO_IN = 1.362;
    public static final double ELEV_IN_TO_REV = 1/ELEV_REV_TO_IN;

    public static final double BOTTOM_HEIGHT = 0;
    public static final double TELEOP_HEIGHT = 5;//3.47;
    public static final double L1_HEIGHT = 0;
    public static final double L2_HEIGHT = 4;//6;
    public static final double L3_HEIGHT = 21;//23;
    public static final double L4_HEIGHT = 53;//57;//no more than 57.5
    public static final double PICK_ALGAE_L2_HEIGHT = 22; //TODO find actual
    public static final double PICK_ALGAE_L3_HEIGHT = 38;//TODO find actual
    public static final double SCORE_ALGAE_NET_HEIGHT = 60; //TODO find actual
    public static final double CLIMB_START_HEIGHT = 12;  
    public static final double CLIMB_END_HEIGHT = 0; 
    public static final double MAX_HEIGHT = 60.15;

    //PID values
    public static final double KP_ELEV = 0.039; //0.04;
    public static final double KI_ELEV = 0;
    public static final double KD_ELEV = 0;
  }

public static class AlgaeHold {
  public static final double HOLD_SPEED1 = -0.5;//MUST BE NEGATIVE!
  public static final double HOLD_SPEED2 = -0.04;//MUST BE NEGATIVE!
  public static final double RELEASE_SPEED = 1.0;
  public static final double PROCESSOR_SPEED = 0.125;
  public static final int DIO_AH_LIMIT = 9;
}

public static class CoralHold {
  public static final int DIO_COUNTER = 10;
  public static final double HOLD_SPEED = 0.2;
  public static final double L1_RELEASE_SPEED = 0.25;//0.2;
  public static final double L2_RELEASE_SPEED = 0.25;//0.5;
  public static final double L3_RELEASE_SPEED = 0.25;//0.5;
  public static final double L4_RELEASE_SPEED = 0.25;//0.1;
}

  public static class AlgaePivot {
    public static final int DIO_LIMIT = 6;
    public static final int DIO_ENC_A = 7;
    public static final int DIO_ENC_B = 13;
    //TODO find actual values with new limit switch position (approx 167 difference)
    public static final double ENC_REVS_MAX = -855-167; //TODO determine max revs
    public static final double ENC_REVS_BUMP = -250-167;
    public static final double ENC_REVS_REEF_PICKUP = -500-167;
    public static final double ENC_REVS_FLOOR_PICKUP = -23.7-167;
    public static final double ENC_REVS_SCORE_NET = 0; //TODO find actual
    public static final double ENC_REVS_ELEVATOR_SAFE_POSITION = 0;//TODO verify safe at retract limit
    public static final double MAN_EXT_SPEED = -0.4;
    public static final double MAN_RET_SPEED = 0.4;
    public static final double KP = 0.01;  //TODO find actual
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KFF = 0;
  }
  
  public static class CoralPivot {
    public static final int DIO_LIMIT = 8; 
    public static final double ENC_REVS_MAX = -125;
    public static final double ENC_REVS_LEVEL1 = 0;
    public static final double ENC_REVS_LEVEL2 = -10;//-45;
    public static final double ENC_REVS_LEVEL3 = -10;//-45; 
    public static final double ENC_REVS_LEVEL4 = -95;//-125;
    public static final double ENC_REVS_FULL_RETRACT = 0;
    public static final double MAN_EXT_SPEED = -0.6;
    public static final double MAN_RET_SPEED = 0.6;
    public static final double ENC_REVS_LOADING = 0;
    public static final double KP = 0.05; //TODO tune better?
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KFF = 0;
    public static final int DIO_ENC_A = 11;
    public static final int DIO_ENC_B = 12;
  }
  
  public static class XboxController {
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int LB = 5;
    public static final int RB = 6;
    public static final int VIEW = 7;
    public static final int MENU = 8;
    public static final int LM = 9;
    public static final int RM = 10;

    public static class AxesXbox {
      public static final int LX = 0;
      public static final int LY = 1;
      public static final int LTrig = 2;
      public static final int RTrig = 3;
      public static final int RX = 4;
      public static final int RY = 5;
    }

    public class POVXbox {
      public static final int UP_ANGLE = 0;
      public static final int RIGHT_ANGLE = 90;
      public static final int DOWN_ANGLE = 180;
      public static final int LEFT_ANGLE = 270;
    }
}

}