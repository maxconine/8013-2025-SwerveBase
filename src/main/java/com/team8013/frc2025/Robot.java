// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team8013.frc2025;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import com.team254.lib.util.Util;
import com.team8013.frc2025.auto.AutoModeBase;
import com.team8013.frc2025.auto.AutoModeExecutor;
import com.team8013.frc2025.auto.AutoModeSelector;
import com.team8013.frc2025.controlboard.ControlBoard;
import com.team8013.frc2025.loops.CrashTracker;
import com.team8013.frc2025.loops.Looper;
import com.team8013.frc2025.shuffleboard.ShuffleBoardInteractions;
import com.team8013.frc2025.subsystems.Drive;
import com.team8013.frc2025.subsystems.Superstructure;
import com.team8013.lib.swerve.ChassisSpeeds;

public class Robot extends TimedRobot {

	// util instances
	private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
	private final ControlBoard mControlBoard = ControlBoard.getInstance();
	private final ShuffleBoardInteractions mShuffleboard = ShuffleBoardInteractions.getInstance();
	// private final LoggingSystem mLogger = LoggingSystem.getInstance();

	// subsystem instances
	private final Superstructure mSuperstructure = Superstructure.getInstance();
	private final Drive mDrive = Drive.getInstance();
	//private final Limelight mLimelight = Limelight.getInstance();
	// private final Pivot mPivot = Pivot.getInstance();
	// private final Elevator mElevator = Elevator.getInstance();
	// private final Wrist mWrist = Wrist.getInstance();
	// private final EndEffectorREV mEndEffector = EndEffectorREV.getInstance();
	// private final Shooter mShooter = Shooter.getInstance();
	// private final ClimberHook mClimberHook = ClimberHook.getInstance();

	// instantiate enabled and disabled loopers
	private final Looper mEnabledLooper = new Looper();
	private final Looper mDisabledLooper = new Looper();
	// private final Looper mLoggingLooper = new Looper(0.002);

	// auto instances
	private AutoModeExecutor mAutoModeExecutor;
	public final static AutoModeSelector mAutoModeSelector = new AutoModeSelector();

	public static boolean is_red_alliance = false;
	public static boolean flip_trajectories = false;
	// private boolean autoAllignBoolean = false;

	// private final int kDpadUp = 0;
	// private final int kDpadRight = 90;
	// private final int kDpadDown = 180;
	// private final int kDpadLeft = 270;

	public Robot() {
		CrashTracker.logRobotConstruction();
	}

	@Override
	public void robotInit() {
		try {
			CrashTracker.logRobotInit();

			LiveWindow.disableAllTelemetry();

			mSubsystemManager.setSubsystems(
					mDrive,
					mSuperstructure
					// mPivot,
					// mElevator,
					// mWrist,
					// mEndEffector,
					// mShooter,
					// mClimberHook

			);

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);

			// mLoggingLooper.register(mLogger.Loop());

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void robotPeriodic() {
		mShuffleboard.update();
		mSubsystemManager.outputToSmartDashboard();
		mEnabledLooper.outputToSmartDashboard();
	}

	@Override
	public void autonomousInit() {

		try {
			mDisabledLooper.stop();
			Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
			if (autoMode.isPresent()) {
				mDrive.resetOdometry(autoMode.get().getStartingPose());
				System.out.println("ODOMETRY RESET FOR AUTO");
			}

			mEnabledLooper.start();
			mAutoModeExecutor.start();
			// mLoggingLooper.start();
			mControlBoard.setAutoSnapToTarget(false);

			mDrive.setNeutralBrake(true);

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
		CrashTracker.logAutoInit();

	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
		mDrive.orientModules(List.of(
				Rotation2d.fromDegrees(45),
				Rotation2d.fromDegrees(-45),
				Rotation2d.fromDegrees(-45),
				Rotation2d.fromDegrees(45)));
	}

	@Override
	public void teleopInit() {
		try {
			if (is_red_alliance) {
				mDrive.zeroGyro(mDrive.getHeading().getDegrees() + 180.0);
				flip_trajectories = false;
			}
			mDisabledLooper.stop();
			mEnabledLooper.start();
			// mLoggingLooper.start();
			mSuperstructure.stop();

			mDrive.setAutoSpinFast(false);

			mControlBoard.setAutoSnapToTarget(false);

			mDrive.setNeutralBrake(true);
			// mClimberHook.setWantNeutralBrake(true);
			// mSuperstructure.disableAutoShot();

			// mLimelight.setShootingFromMid2Piece(false);
			// mLimelight.setShootingFromStage2Piece(false);
			// mLimelight.setShootingFromAmp2Piece(false);
			// mLimelight.setShootingSideOfSubwoofer(false);

			// mSuperstructure.setSuperstuctureShoot(false); // prevents robot from catching note after 1st shot

			// mSuperstructure.setManualControlMode(Constants.isManualControlMode);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopPeriodic() {
		try {

			/* Drive */
			if (mControlBoard.zeroGyro()) {
				mDrive.zeroGyro();
				mDrive.resetModulesToAbsolute();
			}

			if (mControlBoard.getBrake()) {
				mDrive.orientModules(List.of(
						Rotation2d.fromDegrees(45),
						Rotation2d.fromDegrees(-45),
						Rotation2d.fromDegrees(-45),
						Rotation2d.fromDegrees(45)));
			} else {
				mDrive.feedTeleopSetpoint(ChassisSpeeds.fromFieldRelativeSpeeds(
						mControlBoard.getSwerveTranslation().x(),
						mControlBoard.getSwerveTranslation().y(),
						mControlBoard.getSwerveRotation(),
						mDrive.getHeading()));

			}

			// if (mControlBoard.allignWithHumanPlayer()) {
			// 	if (!is_red_alliance) { // keep in mind the alliance is flipped
			// 		mDrive.setHeadingControlTarget(60);
			// 	} else {
			// 		mDrive.setHeadingControlTarget(-60);
			// 	}
			// } else if (mControlBoard.passNoteFromMidAllign()) {
			// 	if (!is_red_alliance) { // keep in mind the alliance is flipped
			// 		mDrive.setHeadingControlTarget(-140);
			// 	} else {
			// 		mDrive.setHeadingControlTarget(140);
			// 	}
			// } else if (mControlBoard.shootFromPodiumAllign()) {
			// 	if (!is_red_alliance) { // keep in mind the alliance is flipped
			// 		mDrive.setHeadingControlTarget(207);
			// 	} else {
			// 		mDrive.setHeadingControlTarget(-207);
			// 	}
			// } else if (mControlBoard.shootFromOppositePodiumAllign()) {
			// 	// if (!is_red_alliance) { //keep in mind the alliance is flipped
			// 	// mDrive.setHeadingControlTarget(-207);
			// 	// } else {
			// 	// mDrive.setHeadingControlTarget(207);
			// 	// }
			// } else if (mControlBoard.shootFromPodium() && (mControlBoard.farLeftSwitchUp()
			// 		&& !Util.epsilonEquals(207, mDrive.getHeading().getDegrees(), 5))) { // extra check to make sure it
			// 																				// stays the right angle
			// 	if (!is_red_alliance) { // keep in mind the alliance is flipped
			// 		mDrive.setHeadingControlTarget(207);
			// 	} else {
			// 		mDrive.setHeadingControlTarget(-207);
			// 	}
			// }

			//put rest of funtions here, check if manual control mode is on or not and send updates to superstructure
			

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledInit() {
		try {

			CrashTracker.logDisabledInit();
			mEnabledLooper.stop();
			// mLoggingLooper.stop();
			mDisabledLooper.start();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

		if (mAutoModeExecutor != null) {
			mAutoModeExecutor.stop();
		}

		// Reset all auto mode state.
		mAutoModeSelector.reset();
		mAutoModeSelector.updateModeCreator(false);
		mAutoModeExecutor = new AutoModeExecutor();

	}

	@Override
	public void disabledPeriodic() {
		try {

			mDrive.resetModulesToAbsolute();
			// mPivot.resetToAbsolute();
			// mWrist.resetToAbsolute();
			// mElevator.zeroWhenDisabled();

			// mDrive.outputTelemetryDisabled();

			boolean alliance_changed = false;
			if (DriverStation.isDSAttached()) {
				Optional<Alliance> ally = DriverStation.getAlliance();
				if (ally.isPresent()) {
					if (ally.get() == Alliance.Red) {
						if (!is_red_alliance) {
							alliance_changed = true;
						} else {
							alliance_changed = false;
						}
						is_red_alliance = true;
					}
					if (ally.get() == Alliance.Blue) {
						if (is_red_alliance) {
							alliance_changed = true;
						} else {
							alliance_changed = false;
						}
						is_red_alliance = false;
					}
					is_red_alliance = !is_red_alliance; // had to flip this because I drew the trajectories on the red
														// side
				}
			} else {
				alliance_changed = true;
			}
			flip_trajectories = is_red_alliance;

			// SmartDashboard.putBoolean("is_red_alliance", is_red_alliance);
			mAutoModeSelector.updateModeCreator(alliance_changed);
			Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
			if (autoMode.isPresent()) {
				mAutoModeExecutor.setAutoMode(autoMode.get());
			}

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {
		try {
			mDisabledLooper.stop();
			mEnabledLooper.stop();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void simulationPeriodic() {
		// PhysicsSim.getInstance().run();
	}
}