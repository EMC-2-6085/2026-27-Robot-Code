package org.firstinspires.ftc.teamcode.Util;

public class Constants {
  public static class driveTrainConstants {
    public static final double WHEEL_POWER_LARGE = 1.0;
    public static final double WHEEL_POWER_TINY = 0.5;
    
    public static final double WHEEL_P = 0.0065;
    public static final double WHEEL_I = 0.0001;
    public static final double WHEEL_D = 0.0001125;
    public static final double HEADING_P = 0.02;
    
    public static final double INTERGRAL_LIMIT = 200.0;
    public static final double ACCEPTABLE_ERROR = 1.0; //Encoder ticks
  }

  public static class localizationConstants {
    public static final double WHEEL_DIAMETER = 0.104; //Meters
    public static final double TICKS_PER_REV = 537.6; //Encoder ticks per revolution
    public static final double GEAR_RATIO = 1.0; //Motor:wheel

    public static final double ENCODER_WEIGHT = 0.45;
    public static final double CAMERA_WEIGHT = 0.55;
    public static final long CAMERA_ENCODE = 150;
    public static final boolean USE_ENCODER = true;
    public static final boolean USE_VISION = true;
  }

  public static class mephiConstants {
    public static final double FORWARD_SCALE = 1.8;
    public static final double STRAFE_SCALE = 1; 
    public static final double TURN_SCALE = -0.8;
    public static final double TURN_D = -0.05;
  }

  public static class shooterConstants {
    public static final double SHOOT_DISTANCE = 2.0; //Meters temp
    public static final double[] CAMERA_OFFSET = {6.9, 4.20}; //Meters from robot center to lens (x, y) temp
  }

  public static class intakeArmConstants {
    public static final double ARM_KP = 0.0065; //temp
    public static final double ARM_KI = 0.0001; //temp
    public static final double ARM_KD = 0.0001125; //temp
    public static final double INTEGRAL_LIMIT = 200.0; //temp

    public static final double ARM_UP_RADIANS = 0; //temp
    public static final double ARM_DOWN_RADIANS = 1.570795; //temp
    public static final double ARM_IN_RADIANS = 3.14159; //temp
    public static final double ACCEPTABLE_ERROR = 1.0; //temp

    public static final double RADIANS_TO_TICKS = 171.16; //temp
  }

  public static class intakeConstants {
    public static final double ENCODER_CPR = 537.7; //temp
    public static final double INTAKE_KP = 0.0032; //temp
    public static final double INTAKE_KI = 0.0; //temp
    public static final double INTAKE_KD = 0.0; //temp
    public static final double INTEGRAL_LIMIT = 200.0; //temp
    public static final double ACCEPTABLE_ERROR = 1.0; //RPM temp
    public static final double INTAKE_RPM = 312.0; //temp
  }

  public static class indexerConstants {
    public static final double ENCODER_CPR = 537.7; //temp
    public static final double INDEXER_KP = 0.0032; //temp
    public static final double INDEXER_KI = 0.0; //temp
    public static final double INDEXER_KD = 0.0; //temp
    public static final double INTEGRAL_LIMIT = 200.0; //temp
    public static final double ACCEPTABLE_ERROR = 1.0; //RPM temp
    public static final double INDEXER_RPM = 312.0; //temp
  }
}