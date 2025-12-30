package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;

import java.util.List;

/**
 * Configuration constants for robot hardware and subsystems.
 * 
 * @author FTC Team 23070 Royal Turtles
 */
@Configurable
public final class BotConstants {

    @IgnoreConfigurable
    public static final class Drivetrain {
        public static final String LEFT_FRONT_DRIVE_MOTOR = "frontLeft";
        public static final String LEFT_BACK_DRIVE_MOTOR = "backLeft";
        public static final String RIGHT_FRONT_DRIVE_MOTOR = "frontRight";
        public static final String RIGHT_BACK_DRIVE_MOTOR = "backRight";
        public static final String IMU_NAME = "imu";
    }

    public static final class Shooter {
        @IgnoreConfigurable
        public static final String LEFT_SHOOTER_MOTOR = "lShooter";
        @IgnoreConfigurable
        public static final String RIGHT_SHOOTER_MOTOR = "rShooter";

        // --- PID Controller Gains ---
        // Note: Final tuned values should be placed here after tuning
        public static final double SHOOTER_P_GAIN = 1;
        public static final double SHOOTER_I_GAIN = 0.001;
        public static final double SHOOTER_D_GAIN = 0.0;
        public static final double SHOOTER_F_GAIN = 0.0;

        // --- Physical Properties ---
        @IgnoreConfigurable
        public static final double SHOOTER_MOTOR_MAX_RPM = 6000;
        @IgnoreConfigurable
        public static final double SHOOTER_MOTOR_MAX_VELOCITY = 5600; // Ticks per sec
        @IgnoreConfigurable
        public static final double SHOOTER_MOTOR_MAX_PPR = 28;
        @IgnoreConfigurable
        public static final double SHOOTER_GEAR_RATIO = 2;
    }



    public static final class Index {
        @IgnoreConfigurable
        public static final String INDEX_MOTOR = "index";

        // --- PID Controller Gains ---
        public static final double INDEX_P_GAIN = 0.5;
        public static final double INDEX_I_GAIN = 0.0;
        public static final double INDEX_D_GAIN = 0.0;
        public static final double INDEX_F_GAIN = 0.0;

        // --- Physical Properties ---
        @IgnoreConfigurable
        public static final double INDEX_MOTOR_MAX_RPM = 1150;
        @IgnoreConfigurable
        public static final double INDEX_MOTOR_MAX_VELOCITY = 805; // Ticks per sec
        @IgnoreConfigurable
        public static final double INDEX_MOTOR_GEAR_RATIO = 1.5;
        @IgnoreConfigurable
        public static final double INDEX_MOTOR_MAX_PPR = 28;
    }

    public static final class AutoAlign {
        public static final double ALIGN_P_GAIN = 0.5;
        public static final double ALIGN_I_GAIN = 0;
        public static final double ALIGN_D_GAIN = 0;
    }

    @IgnoreConfigurable
    public static final class Limelight {
        // how many degrees back is your limelight rotated from perfectly vertical?
        public static final double limelightMountAngleDegrees = 12.0;

        // distance from the center of the Limelight lens to the floor
        public static final double limelightLensHeightInches = 14.587;

        // distance from the target to the floor
        public static final double goalHeightInches = 38.75 - 9.25;
    }

    public static final class AutoShoot {
        public static final double[] distances = new double[]{

        };

        public static final double[] velocities = new double[]{

        };

        public static final double degree = 0;
    }
}