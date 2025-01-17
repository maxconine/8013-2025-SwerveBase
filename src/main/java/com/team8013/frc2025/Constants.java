package com.team8013.frc2025;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.team8013.frc2025.subsystems.Drive.KinematicLimits;
import com.team8013.lib.Conversions;
import com.team8013.lib.swerve.SwerveModule.SwerveModuleConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

import com.team8013.lib.swerve.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.util.Units;

public class Constants {

    // toggle constants between comp bot and practice bot ("epsilon")
    public static boolean isBeta = false;
    public static boolean isComp = true;

    public static boolean isCompBot() {
        return isComp;
    }

    public static boolean isBetaBot() {
        return isBeta;
    }

    // Disables extra smart dashboard outputs that slow down the robot
    public static final boolean disableExtraTelemetry = true;

    public static final boolean isManualControlMode = false;

    // robot loop time
    public static final double kLooperDt = 0.02;

    /* Control Board */
    public static final double kTriggerThreshold = 0.2;

    public static final double stickDeadband = 0.05;
    public static final int leftXAxis = 0;
    public static final int leftYAxis = 1;
    public static final int rightXAxis = 3;
    public static final int rightYAxis = 4;

    public static final class SwerveConstants {
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24.25);// 24.25);
        public static final double wheelBase = Units.inchesToMeters(24.25);

        public static final double wheelDiameter = Units.inchesToMeters(3.85);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        // this is for the comp bot
        // public static final double driveGearRatio = 6.75; //flipped gear ratio
        // https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options
        // public static final double angleGearRatio = 15.43; //8:32:24--14:72 = 15.43
        // ratio

        // can tune this value by driving a certain distance and multiplying a const to
        // fix the error
        public static final double driveGearRatio = 6.12; //((5.3 / 1.07) / 1.04); // it's 4.76:1
        public static final double angleGearRatio = 21.4285714;// (150/7);// 10.29; // 72:14:24:12

        public static final edu.wpi.first.math.geometry.Translation2d[] swerveModuleLocations = {
                new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
                new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, trackWidth / 2.0)
        };

        public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(swerveModuleLocations);

        /* Swerve Current Limiting - very neccesary! */
        public static final int angleContinuousCurrentLimit = 25;
        // public static final int anglePeakCurrentLimit = 40;
        // public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 62;
        // public static final int drivePeakCurrentLimit = 65;
        // public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.32 / 12);
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.8; // meters per second MAX : 5.02 m/s
        public static final double maxAngularVelocity = 8.0;

        public static final double maxAttainableSpeed = maxSpeed * 0.85; // Max out at 85% to make sure speeds are
                                                                         // attainable (4.6 mps)

        /* Angle Motor PID Values */
        public static final double angleKP = 0.6; // 0.3 falcon - higher with kraken
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 4.0;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 12.0 / Conversions.MPSToRPS(maxSpeed, wheelCircumference, driveGearRatio);

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Motor Inverts */
        public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive; // false

        /* Controller Invert */
        public static final boolean invertYAxis = false;
        public static final boolean invertRAxis = false;
        public static final boolean invertXAxis = true;

        public static final KinematicLimits kUncappedLimits = new KinematicLimits();
        static {
            kUncappedLimits.kMaxDriveVelocity = maxSpeed;
            kUncappedLimits.kMaxAccel = Double.MAX_VALUE;
            kUncappedLimits.kMaxAngularVelocity = maxAngularVelocity;
            kUncappedLimits.kMaxAngularAccel = Double.MAX_VALUE;
        }

        public static final KinematicLimits kScoringLimits = new KinematicLimits();
        static {
            kScoringLimits.kMaxDriveVelocity = 2.0;
            kScoringLimits.kMaxAccel = Double.MAX_VALUE;
            kScoringLimits.kMaxAngularVelocity = Math.PI; // Rad/Sec
            kScoringLimits.kMaxAngularAccel = 10 * Math.PI; // 2 * Math.PI;
        }

        public static final KinematicLimits kLoadingStationLimits = new KinematicLimits();
        static {
            kLoadingStationLimits.kMaxDriveVelocity = 1.5;
            kLoadingStationLimits.kMaxAccel = Double.MAX_VALUE;
            kLoadingStationLimits.kMaxAngularVelocity = maxAngularVelocity;
            kLoadingStationLimits.kMaxAngularAccel = Double.MAX_VALUE;
        }

        public static final KinematicLimits kAutoLimits = new KinematicLimits();
        static {
            kAutoLimits.kMaxDriveVelocity = maxAttainableSpeed;
            kAutoLimits.kMaxAccel = Double.MAX_VALUE;
            kAutoLimits.kMaxAngularVelocity = Double.MAX_VALUE; // Rad/Sec
            kAutoLimits.kMaxAngularAccel = Double.MAX_VALUE; // 2 * Math.PI;
        }

        /***
         * MODULE SPECIFIC CONSTANTS **
         * 
         * which way to zero modules?
         * if you tip the robot up on its back side, allign the bevel gears to the right
         * side (from lookers perspective) on all the wheels. Make sure all the wheels
         * are in line then record canCoder offset values in shuffleboard
         * 
         * Zero them so that odometry is x positive going forwards and y positive going
         * left
         * 
         * 
         */

        /*** MODULE SPECIFIC CONSTANTS ***/
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final double compAngleOffset = 89.38;

            public static SwerveModuleConstants SwerveModuleConstants() {
                return new SwerveModuleConstants(Ports.FL_DRIVE, Ports.FL_ROTATION, Ports.FL_CANCODER,
                        compAngleOffset);
            }
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final double compAngleOffset = 335.3;

            public static SwerveModuleConstants SwerveModuleConstants() {
                return new SwerveModuleConstants(Ports.FR_DRIVE, Ports.FR_ROTATION, Ports.FR_CANCODER, compAngleOffset);
            }
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final double compAngleOffset = 73.12;

            public static SwerveModuleConstants SwerveModuleConstants() {
                return new SwerveModuleConstants(Ports.BL_DRIVE, Ports.BL_ROTATION, Ports.BL_CANCODER, compAngleOffset);
            }
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final double compAngleOffset = 191.42;

            public static SwerveModuleConstants SwerveModuleConstants() {
                return new SwerveModuleConstants(Ports.BR_DRIVE, Ports.BR_ROTATION, Ports.BR_CANCODER, compAngleOffset);
            }
        }

        public static TalonFXConfiguration swerveDriveFXConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits.SupplyCurrentLimitEnable = Constants.SwerveConstants.driveEnableCurrentLimit;
            config.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.driveContinuousCurrentLimit;

            config.Voltage.PeakForwardVoltage = 12.0;
            config.Voltage.PeakReverseVoltage = -12.0;

            config.Slot0.kP = Constants.SwerveConstants.driveKP;
            config.Slot0.kI = Constants.SwerveConstants.driveKI;
            config.Slot0.kD = Constants.SwerveConstants.driveKD;

            config.MotorOutput.NeutralMode = Constants.SwerveConstants.driveNeutralMode;
            config.MotorOutput.Inverted = Constants.SwerveConstants.driveMotorInvert;

            config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;
            config.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;
            return config;
        }

        public static TalonFXConfiguration swerveAngleFXConfig() {
            TalonFXConfiguration angleConfig = new TalonFXConfiguration();
            angleConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.SwerveConstants.angleEnableCurrentLimit;
            angleConfig.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.angleContinuousCurrentLimit;

            angleConfig.Slot0.kP = Constants.SwerveConstants.angleKP;
            angleConfig.Slot0.kI = Constants.SwerveConstants.angleKI;
            angleConfig.Slot0.kD = Constants.SwerveConstants.angleKD;
            angleConfig.Slot0.kV = Constants.SwerveConstants.angleKF;

            angleConfig.MotorOutput.NeutralMode = Constants.SwerveConstants.angleNeutralMode;
            angleConfig.MotorOutput.Inverted = Constants.SwerveConstants.angleMotorInvert;

            return angleConfig;
        }

        public static CANcoderConfiguration swerveCancoderConfig() {
            CANcoderConfiguration config = new CANcoderConfiguration();
            config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1; // used to be
                                                                      // AbsoluteSensorRangeValue.Unsigned_0To1;
            config.MagnetSensor.SensorDirection = Constants.SwerveConstants.canCoderInvert;
            return config;
        }

        public static Pigeon2Configuration pigeon2Config() {
            Pigeon2Configuration config = new Pigeon2Configuration();
            return config;
        }
    }

    public static final class SnapConstants {
        public static final double kP = 6.0;
        public static final double kI = 0.5;
        public static final double kD = 0.2;
        public static final double snapTimeout = 0.25;
        public static final double snapEpsilon = 1.0;

    }

    public static final class AutoConstants {

        public static final double kPXController = 6.7;
        public static final double kPYController = 6.7;

        public static final double kDXController = 0.0;
        public static final double kDYController = 0.0;

        public static final double kPThetaController = 2.75; // was 2, changed to 4 -- faster it turns = more wheels
                                                             // slip

        // Constraint for the motion profilied robot angle controller (Radians)
        public static final double kMaxAngularSpeed = 2.0 * Math.PI;
        public static final double kMaxAngularAccel = 2.0 * Math.PI * kMaxAngularSpeed;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeed, kMaxAngularAccel);

        // Static factory for creating trajectory configs
        public static TrajectoryConfig createConfig(double maxSpeed, double maxAccel, double startSpeed,
                double endSpeed) {
            TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxAccel);
            config.setStartVelocity(startSpeed);
            config.setEndVelocity(endSpeed);
            config.addConstraint(new CentripetalAccelerationConstraint(10.0));
            return config;
        }
    }

    public static final class VisionAlignConstants {
        public static final double kP = 6.37;
        public static final double kI = 0.0;
        public static final double kD = 0.10;
        public static final double kTimeout = 0.25;
        public static final double kEpsilon = 5.0;

        // Constraints for the profiled angle controller
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.0 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 10.0 * Math.PI;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        /* April Tag Chase */

        public static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 1);
        public static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 1);
        public static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

        public static final int TAG_TO_CHASE = 1;
        public static final Transform3d TAG_TO_GOAL = new Transform3d(
                new Translation3d(0.0, 0.0, 0.0),
                new Rotation3d(0.0, 0.0, Math.PI));

        public static final TrajectoryConfig TAG_TRAJECTORY_CONFIG = Constants.AutoConstants.createConfig(2.5, 2.0, 0.0,
                0.0);

        public static final double POSITION_OFF = 0.1;

        /**
         * Standard deviations of model states. Increase these numbers to trust your
         * model's state estimates less. This matrix is in the form [x, y, theta]ᵀ,
         * with units in meters and radians, then meters.
         */
        public static final Vector<N3> STATE_STDS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

        /**
         * Standard deviations of the vision measurements. Increase these numbers to
         * trust global measurements from vision less. This matrix is in the form
         * [x, y, theta]ᵀ, with units in meters and radians.
         */
        public static final Vector<N3> VISION_STDS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));

    }

    public static final class MacAddressConstants {
        public static final byte[] COMP_ADDRESS = new byte[] {
                // values are for comp -> 00:80:2f:35:b8:ca
                (byte) 0x00, (byte) 0x80, (byte) 0x2f, (byte) 0x35, (byte) 0xb8, (byte) 0xca
        };

        public static final byte[] BETA_ADDRESS = new byte[] {
                // values are for beta -> 00:80:2f:34:0B:9B
                (byte) 0x00, (byte) 0x80, (byte) 0x2f, (byte) 0x34, (byte) 0x0b, (byte) 0x9b
        };
    }

    /*** SUBSYSTEM CONSTANTS ***/

    public static final class ControllerConstants {
        public static final boolean isMamboController = true; // this overrides everything
        public static final boolean isControllerOne = true;

        // Controller 1 left side:
        public static final double ControllerOneLeftThrottleZero = -0.125;
        public static final double ControllerOneLeftYawZero = 0.039370;

        public static final double ControllerOneLeftThrottleHigh = 0.787402;
        public static final double ControllerOneLeftThrottleLow = 0.968750;

        public static final double ControllerOneLeftYawHigh = 0.86612;
        public static final double ControllerOneLeftYawLow = 0.77338;

        // Controller 1 right side:
        public static final double ControllerOneRightThrottleZero = 0.055118;
        public static final double ControllerOneRightYawZero = 0.055118;

        public static final double ControllerOneRightYawHigh = 0.866142;
        public static final double ControllerOneRightYawLow = 0.765625;

        public static final double ControllerOneRightThrottleHigh = 0.732283;
        public static final double ControllerOneRightThrottleLow = 0.601563;

        // Controller 2 left side:
        public static final double ControllerTwoLeftThrottleZero = -0.023438;
        public static final double ControllerTwoLeftYawZero = -0.078125;

        public static final double ControllerTwoLeftThrottleHigh = 0.834646;
        public static final double ControllerTwoLeftThrottleLow = 0.867188;

        public static final double ControllerTwoLeftYawHigh = 0.748031;
        public static final double ControllerTwoLeftYawLow = 0.890625;

        // Controller 2 right side:
        public static final double ControllerTwoRightThrottleZero = -0.054688; // high 0.007874
        public static final double ControllerTwoRightYawZero = 0.062992;

        public static final double ControllerTwoRightYawHigh = 0.866142;
        public static final double ControllerTwoRightYawLow = 0.664063;

        public static final double ControllerTwoRightThrottleHigh = 0.669291;
        public static final double ControllerTwoRightThrottleLow = 0.664063;

        // Controller left side:
        public static final double ControllerLeftThrottleZero = isControllerOne ? ControllerOneLeftThrottleZero
                : ControllerTwoLeftThrottleZero;
        public static final double ControllerLeftYawZero = isControllerOne ? ControllerOneLeftYawZero
                : ControllerTwoLeftYawZero;

        public static final double ControllerLeftThrottleHigh = isControllerOne ? ControllerOneLeftThrottleHigh
                : ControllerTwoLeftThrottleHigh;
        public static final double ControllerLeftThrottleLow = isControllerOne ? ControllerOneLeftThrottleLow
                : ControllerTwoLeftThrottleLow;

        public static final double ControllerLeftYawHigh = isControllerOne ? ControllerOneLeftYawHigh
                : ControllerTwoLeftYawHigh;
        public static final double ControllerLeftYawLow = isControllerOne ? ControllerOneLeftYawLow
                : ControllerTwoLeftYawLow;

        // Controller right side:
        public static final double ControllerRightThrottleZero = isControllerOne ? ControllerOneRightThrottleZero
                : ControllerTwoRightThrottleZero;
        public static final double ControllerRightYawZero = isControllerOne ? ControllerOneRightYawZero
                : ControllerTwoRightYawZero;

        public static final double ControllerRightYawHigh = isControllerOne ? ControllerOneRightYawHigh
                : ControllerTwoRightYawHigh;
        public static final double ControllerRightYawLow = isControllerOne ? ControllerOneRightYawLow
                : ControllerTwoRightYawLow;

        public static final double ControllerRightThrottleHigh = isControllerOne ? ControllerOneRightThrottleHigh
                : ControllerTwoRightThrottleHigh;
        public static final double ControllerRightThrottleLow = isControllerOne ? ControllerOneRightThrottleLow
                : ControllerTwoRightThrottleLow;

    }

    // Timeout constants
    public static final int kLongCANTimeoutMs = 100;
    public static final int kCANTimeoutMs = 10;
}
