package org.firstinspires.ftc.team5898.Hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import androidx.annotation.*;
public class Constants {
  private Constants() {}
  
  /** Hardware device names */
  public static class Hardware {
    @NonNull public static String MOTOR_FRONT_LEFT = "fl";
    @NonNull public static String MOTOR_FRONT_RIGHT = "fr";
    @NonNull public static String MOTOR_BACK_LEFT = "bl";
    @NonNull public static String MOTOR_BACK_RIGHT = "br";
    @NonNull public static String IMU = "imu";
  }

  /** Robot physical dimensions and drivetrain parameters; Basic Autonomous only!!! */
  public static class Drive {
    public static double COUNTS_PER_ROTATION   = 537.7;
    public static double WHEEL_DIAMETER_INCHES = 3.779;
    public static double DRIVE_GEAR_REDUCTION  = 1.0;
    public static double COUNTS_PER_INCH =
        (COUNTS_PER_ROTATION * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_INCHES * Math.PI);
  }

  /** PIDF values with default values for Launcher1 and Launcher2. Use with DcMotorEx class */
  @Configurable
  public static class PIDFLauncher1 {
    @Sorter(sort = 0)
    public static double kP = 0.0;
    @Sorter(sort = 1)
    public static double kI = 0.0;
    @Sorter(sort = 2)
    public static double kD = 0.0;
    @Sorter(sort = 3)
    public static double kF = 0.0;
  }

  @Configurable
  public static class PIDFLauncher2 {
    @Sorter(sort = 4)
    public static double kP = 0.0;
    @Sorter(sort = 5)
    public static double kI = 0.0;
    @Sorter(sort = 6)
    public static double kD = 0.0;
    @Sorter(sort = 7)
    public static double kF = 0.0;
  }

  @Configurable
  public static class Limelight {
    @Sorter(sort = 8)
    public static double TARGET_AREA_THRESHOLD = 2.0;
    @Sorter(sort = 9)
    public static double BLUE_ALLIANCE_TX = 3.30;
    @Sorter(sort = 10)
    public static double RED_ALLIANCE_TX = 0.31;
  }
}
