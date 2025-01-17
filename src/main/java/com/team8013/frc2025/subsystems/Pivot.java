package com.team8013.frc2025.subsystems;

// import com.team8013.frc2024.Constants;
// import com.team8013.frc2024.Ports;
// import com.team8013.frc2024.loops.ILooper;
// import com.team8013.frc2024.loops.Loop;
// import com.team8013.lib.Conversions;
// import com.team8013.lib.Util;
// import com.team8013.lib.logger.Log;

// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class Pivot extends Subsystem {

//     private static Pivot mInstance;
//     private TalonFX mMaster;
//     private TalonFX mSlave;
//     private CANcoder mCANcoder;

//     private mPeriodicIO mPeriodicIO = new mPeriodicIO();

//     public static Pivot getInstance() {
//         if (mInstance == null) {
//             mInstance = new Pivot();
//         }
//         return mInstance;
//     }

//     private Pivot() {
//         mMaster = new TalonFX(Ports.PIVOT_B, Ports.CANBUS_UPPER);
//         mSlave = new TalonFX(Ports.PIVOT_A, Ports.CANBUS_UPPER);
//         mCANcoder = new CANcoder(Ports.PIVOT_CANCODER, Ports.CANBUS_LOWER);
//         CANcoderConfiguration CANCoderConfig = Constants.PivotConstants.pivotCancoderConfig();

//         // configs from constants
//         mMaster.getConfigurator().apply(Constants.PivotConstants.pivotFastMotorConfig());
//         mSlave.getConfigurator().apply(Constants.PivotConstants.pivotFastMotorConfig());

//         mCANcoder.getConfigurator().apply(CANCoderConfig);

//         mSlave.setControl(new Follower(Ports.PIVOT_B, true));
//         setWantNeutralBrake(true);
//         resetToAbsolute();
//     }

//     public void resetToAbsolute() {
//         double absolutePosition = Conversions.degreesToRotation(getCanCoder(),
//                 Constants.PivotConstants.PivotGearRatio);
//         mMaster.setPosition(absolutePosition);
//     }

//     private void setWantNeutralBrake(boolean brake) {
//         NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
//         mMaster.setNeutralMode(mode);
//         mSlave.setNeutralMode(mode);
//     }

//     @Override
//     public void registerEnabledLoops(ILooper mEnabledLooper) {
//         mEnabledLooper.register(new Loop() {
//             @Override
//             public void onStart(double timestamp) {
//                 resetToAbsolute();
//             }

//             @Override
//             public void onLoop(double timestamp) {

//             }

//             @Override
//             public void onStop(double timestamp) {
//                 setWantNeutralBrake(true);
//             }
//         });
//     }

//     @Override
//     public synchronized void writePeriodicOutputs() {
//         if (mPeriodicIO.mControlModeState == ControlModeState.MOTION_MAGIC) {
//             mMaster.setControl(new MotionMagicDutyCycle(mPeriodicIO.demand, true, 0, 0, false, false, false));
//         } else if (mPeriodicIO.mControlModeState == ControlModeState.OPEN_LOOP) {
//             if (mPeriodicIO.demand > 1 || mPeriodicIO.demand < -1) {
//                 mMaster.setControl(new VoltageOut(mPeriodicIO.demand)); // Enable FOC in the future?
//             } else {

//                 mMaster.setControl(new DutyCycleOut(mPeriodicIO.demand)); // needs a feedforward
//             }
//         }

//     }


//     public void setSetpointMotionMagic(double degrees) {
//         if (mPeriodicIO.mControlModeState != ControlModeState.MOTION_MAGIC) {
//             mPeriodicIO.mControlModeState = ControlModeState.MOTION_MAGIC;
//         }

//         // Limit forward and backward movement
//         // if (degrees > Constants.PivotConstants.kMaxAngle) {
//         // degrees = Constants.PivotConstants.kMaxAngle;
//         // } else if (degrees < Constants.PivotConstants.kMinAngle) {
//         // degrees = Constants.PivotConstants.kMinAngle;
//         // }

//         double rotationDemand = Conversions.degreesToRotation(degrees, Constants.PivotConstants.PivotGearRatio);
//         mPeriodicIO.demand = rotationDemand;
//     }

//     public void setDemandOpenLoop(double demand) {
//         if (mPeriodicIO.mControlModeState != ControlModeState.OPEN_LOOP) {
//             mPeriodicIO.mControlModeState = ControlModeState.OPEN_LOOP;
//         }
//         mPeriodicIO.demand = demand;
//     }

//     public double getCanCoder() {
//         return Util.placeIn0To360Scope(
//                 (mCANcoder.getAbsolutePosition().getValueAsDouble() * 360) - Constants.PivotConstants.CANCODER_OFFSET);
//     }

//     public void setMotorConfig(TalonFXConfiguration config) {
//         mMaster.getConfigurator().apply(config);
//     }

//     @Log
//     public double getPivotAngleDeg() {
//         return mPeriodicIO.position_degrees;
//     }

//     @Log
//     public double getPivotDemand() {
//         return mPeriodicIO.demand;
//     }

//     @Log
//     public double getPivotVelocity() {
//         return mPeriodicIO.velocity_radPerSec;
//     }

//     @Log
//     public double getPivotVolts() {
//         return mPeriodicIO.output_voltage;
//     }

//     @Log
//     public double getPivotCurrent() {
//         return mPeriodicIO.current;
//     }

//     @Log
//     public double getTimestamp() {
//         return mPeriodicIO.timestamp;
//     }

//     @Log
//     public double getMainMotorBusVolts() {
//         return mMaster.getSupplyVoltage().getValueAsDouble();
//     }

//     public static class mPeriodicIO {
//         // Inputs
//         public double timestamp = 0.0;
//         public double targetVelocity = 0.0;
//         public double position_degrees = 0.0;
//         public double velocity_radPerSec = 0.0;

//         public double current = 0.0;
//         public double output_voltage = 0.0;

//         // Outputs
//         public double demand = 0;
//         public ControlModeState mControlModeState = ControlModeState.OPEN_LOOP;
//     }

//     private enum ControlModeState {
//         OPEN_LOOP,
//         MOTION_MAGIC
//     }

//     @Override
//     public synchronized void readPeriodicInputs() {
//         mPeriodicIO.position_degrees = Conversions.rotationsToDegrees(mMaster.getRotorPosition().getValueAsDouble(),
//                 Constants.PivotConstants.PivotGearRatio);
//         mPeriodicIO.current = mMaster.getTorqueCurrent().getValueAsDouble();
//         mPeriodicIO.output_voltage = mMaster.getMotorVoltage().getValueAsDouble();
//         mPeriodicIO.velocity_radPerSec = Conversions.rotationsToDegrees(mMaster.getVelocity().getValueAsDouble(),
//                 Constants.PivotConstants.PivotGearRatio) * Math.PI / 180;
//     }

//     @Override
//     public void outputTelemetry() {
//         SmartDashboard.putNumber("Pivot Angle (degrees)", mPeriodicIO.position_degrees);
//         SmartDashboard.putNumber("Pivot CANCODER (degrees)", getCanCoder());
//         SmartDashboard.putNumber("Pivot Motor Rotations", mMaster.getRotorPosition().getValueAsDouble());
//         SmartDashboard.putNumber("Pivot Demand", mPeriodicIO.demand);
//         SmartDashboard.putNumber("Pivot" + " Velocity rad/s", mPeriodicIO.velocity_radPerSec);
//         SmartDashboard.putNumber("Pivot Demand", mPeriodicIO.demand);
//         SmartDashboard.putNumber("Pivot Volts", mPeriodicIO.output_voltage);
//         SmartDashboard.putNumber("Pivot Current", mPeriodicIO.current);
//         SmartDashboard.putString("Pivot Control State", mPeriodicIO.mControlModeState.toString());
//     }
// }