package com.team8013.frc2025.subsystems;

import com.team8013.frc2025.loops.ILooper;
import com.team8013.frc2025.loops.Loop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem {

    // private Elevator mElevator = Elevator.getInstance();
    // private Wrist mWrist = Wrist.getInstance();
    // // private Drive mDrive = Drive.getInstance();
    // private Limelight mLimelight = Limelight.getInstance();
    // private ControlBoard mControlBoard = ControlBoard.getInstance();
    // private Pivot mPivot = Pivot.getInstance();
    // private Shooter mShooter = Shooter.getInstance();
    // private ClimberHook mClimberHook = ClimberHook.getInstance();

    // private double pivotManualPosition = mPivot.getPivotAngleDeg() + 4;
    // private double elevatorManualPosition = mElevator.getElevatorUnits() + 0.02;
    // private double wristManualPosition = mWrist.getWristAngleDeg();
    // private double climberHookManualPosition = mClimberHook.getAngleDeg();
    private SuperstructureState mSuperstructureState;


    /* Singleton Instance */
    private static Superstructure mInstance;

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    };

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    Timer flash_timeout = new Timer();

    public enum SuperstructureState {
        // INTAKING_GROUND,
        // INTAKING_SOURCE,
        // INTAKING_SHOOTER_SOURCE,
        // SCORE_AMP,
        // TRANSFER_TO_SHOOTER,
        // STOW,
        // CLIMB,
        // DECLIMB,
        // SHOOTER_TO_END_EFFECTOR,
        // SHOOTER_TO_AMP,
        // LOW_PASS,
        // INTAKING_SOURCE_MANUAL
    }

   

    @Override
    public void writePeriodicOutputs() {
        
    }

    // this already exists in util.kepsilonEquals
    public boolean outsideError(double a, double error) {
        return ((Math.abs(a) - Math.abs(error)) > 0);
    }

}