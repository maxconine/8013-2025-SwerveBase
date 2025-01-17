package com.team8013.frc2025.auto.modes;

// import java.util.List;

// import com.team8013.frc2025.Constants;
// import com.team8013.frc2025.Robot;
// import com.team8013.frc2025.auto.AutoModeBase;
// import com.team8013.frc2025.auto.AutoModeEndedException;
// import com.team8013.frc2025.auto.AutoTrajectoryReader;
// import com.team8013.frc2025.auto.actions.LambdaAction;
// import com.team8013.frc2025.auto.actions.ParallelAction;
// import com.team8013.frc2025.auto.actions.SeriesAction;
// import com.team8013.frc2025.auto.actions.SwerveTrajectoryAction;
// import com.team8013.frc2025.auto.actions.WaitAction;
// import com.team8013.frc2025.auto.actions.WaitToPassXCoordinateAction;
// import com.team8013.frc2025.controlboard.ControlBoard;
// import com.team8013.frc2025.shuffleboard.ShuffleBoardInteractions;
// import com.team8013.frc2025.subsystems.Drive;
// import com.team8013.frc2025.subsystems.Superstructure;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.trajectory.Trajectory;

// public class ThreePieceMiddleStart extends AutoModeBase {

//         private Superstructure mSuperstructure;
//         private ControlBoard mControlBoard;
//         private double degreeToTurn = 2;

//         // required PathWeaver trajectory paths
//         String path_B = "paths/2024Paths/3PieceMiddleStart_C.path";
//         String path_C = "paths/2024Paths/3PieceMiddleStart_D.path";
//         String path_A = "paths/2024Paths/TwoMiddleSmooth.path";
//         // String path_C = "paths/2024Paths/3PieceMiddleStart_A.path";
//         // String path_D = "paths/2024Paths/3PieceMiddleStart_B.path";
//         String path_D = "paths/2024Paths/TwoMiddle_C_0.path";

//         // trajectories
//         SwerveTrajectoryAction pathA;
//         final Trajectory drivePath_A;

//         SwerveTrajectoryAction pathB;
//         final Trajectory drivePath_B;

//         SwerveTrajectoryAction pathC;
//         final Trajectory drivePath_C;

//         SwerveTrajectoryAction pathD;
//         final Trajectory drivePath_D;

//         // SwerveTrajectoryAction driveOut;
//         // final Trajectory drivePath_E;

//         public ThreePieceMiddleStart() {
//                 mSuperstructure = Superstructure.getInstance();
//                 mControlBoard = ControlBoard.getInstance();

//                 // read trajectories from PathWeaver and generate trajectory actions
//                 drivePath_A = AutoTrajectoryReader.generateTrajectoryFromFile(path_A,
//                                 Constants.AutoConstants.createConfig(0.7, 1.2, 0.0, 0));
//                 pathA = new SwerveTrajectoryAction(drivePath_A, Rotation2d.fromDegrees(180));
//                 ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drivePath_A);

//                 drivePath_B = AutoTrajectoryReader.generateTrajectoryFromFile(path_B,
//                                 Constants.AutoConstants.createConfig(1.2, 1.5, 0.0, 0));
//                 pathB = new SwerveTrajectoryAction(drivePath_B, Rotation2d.fromDegrees(180));
//                 ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drivePath_B);

//                 drivePath_C = AutoTrajectoryReader.generateTrajectoryFromFile(path_C,
//                                 Constants.AutoConstants.createConfig(1.4, 1.5, 0.0, 0));
//                 pathC = new SwerveTrajectoryAction(drivePath_C, Rotation2d.fromDegrees(20));
//                 ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drivePath_C);

//                 drivePath_D = AutoTrajectoryReader.generateTrajectoryFromFile(path_D,
//                                 Constants.AutoConstants.createConfig(5.5, 3, 0.0, 0));
//                 pathD = new SwerveTrajectoryAction(drivePath_D, Rotation2d.fromDegrees(180));
//                 ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drivePath_D);
//         }

//         @Override
//         protected void routine() throws AutoModeEndedException {
//                 runAction(new LambdaAction(() -> Drive.getInstance().resetOdometry(getStartingPose())));

//                 System.out.println("Running 3 note auto");
//                 // mSuperstructure.setSuperstuctureTransferToShooter();
//                 // runAction(new WaitAction(0.2));
//                 mSuperstructure.autoShot();
//                 runAction(new WaitAction(1.4));
//                 mSuperstructure.disableAutoShot();
//                 mSuperstructure.setSuperstuctureStow();

//                 if (Robot.is_red_alliance) { // if blue spin towards stage
//                         degreeToTurn = -2;
//                 }

//                 runAction(new ParallelAction(List.of(
//                                 pathA,
//                                 new SeriesAction(List.of(

//                                                 new WaitAction(0.1),
//                                                 new LambdaAction(() -> Drive.getInstance()
//                                                                 .setAutoHeading(Rotation2d.fromDegrees(degreeToTurn))), // originally
//                                                                                                                         // turned
//                                                                                                                         // ccw

//                                                 new WaitAction(0.35),
//                                                 new LambdaAction(() -> mSuperstructure
//                                                                 .setSuperstuctureIntakingGround()),
//                                                 new WaitAction(1.5), // 1.5 before wpi
//                                                 new LambdaAction(() -> mSuperstructure
//                                                                 .setSuperstuctureStow()),
//                                                 new WaitAction(0.05),
//                                                 new LambdaAction(() -> Drive.getInstance()
//                                                                 .setAutoHeading(Rotation2d.fromDegrees(180))),
//                                                 new WaitAction(0.8),
//                                                 new LambdaAction(() -> mSuperstructure
//                                                                 .setSuperstuctureTransferToShooter()))))));
//                 runAction(new WaitAction(0.1));
//                 mSuperstructure.autoShot();
//                 runAction(new WaitAction(0.3));
//                 mSuperstructure.disableAutoShot();
//                 mSuperstructure.setSuperstuctureStow();

//                 runAction(new ParallelAction(List.of(
//                                 pathB,
//                                 new SeriesAction(List.of(
//                                                 new WaitAction(0.1),
//                                                 new LambdaAction(() -> Drive.getInstance()
//                                                                 .setAutoHeading(Rotation2d.fromDegrees(20))),
//                                                 new WaitAction(0.35),
//                                                 new LambdaAction(
//                                                                 () -> mSuperstructure.setSuperstuctureIntakingGround()),
//                                                 new WaitAction(1.2),
//                                                 new LambdaAction(() -> Drive.getInstance()
//                                                                 .setAutoHeading(Rotation2d.fromDegrees(8))))))));

//                 runAction(new ParallelAction(List.of(
//                                 pathC,
//                                 new SeriesAction(List.of(
//                                                 new WaitAction(0.2),
//                                                 new LambdaAction(
//                                                                 () -> mSuperstructure.setSuperstuctureStow()),
//                                                 new LambdaAction(() -> Drive.getInstance()
//                                                                 .setAutoHeading(Rotation2d.fromDegrees(180.0))),
//                                                 new WaitAction(0.6),
//                                                 new LambdaAction(() -> mSuperstructure
//                                                                 .setSuperstuctureTransferToShooter()))))));
//                 runAction(new WaitAction(0.1));
//                 mSuperstructure.autoShot();
//                 runAction(new WaitAction(0.3));
//                 mSuperstructure.disableAutoShot();

//                 runAction(new ParallelAction(List.of(
//                                 pathD,
//                                 new SeriesAction(List.of(
//                                                 new WaitAction(0.3),
//                                                 new LambdaAction(
//                                                                 () -> mSuperstructure.setSuperstuctureStow()),
//                                                 new WaitToPassXCoordinateAction(13),
//                                                 new LambdaAction((() -> mSuperstructure
//                                                                 .setSuperstuctureIntakingGround())),
//                                                 new WaitAction(0.1),
//                                                 new LambdaAction(() -> Drive.getInstance()
//                                                                 .setAutoHeading(Rotation2d.fromDegrees(-180))), // changed
//                                                                                                                 // during
//                                                                                                                 // districts
//                                                 new WaitAction(1.9),
//                                                 new LambdaAction(() -> Drive.getInstance()
//                                                                 .setAutoHeading(Rotation2d.fromDegrees(180))),
//                                                 new WaitToPassXCoordinateAction(13),
//                                                 new LambdaAction(() -> mSuperstructure
//                                                                 .setSuperstuctureTransferToShooter()),
//                                                 new WaitAction(0.2),
//                                                 new LambdaAction(() -> mControlBoard.setAutoSnapToTarget(true)))))));

//                 mControlBoard.setAutoSnapToTarget(true);
//                 runAction(new WaitAction(0.1));
//                 mSuperstructure.autoShot();

//         }

//         @Override
//         public Pose2d getStartingPose() {
//                 Rotation2d startingRotation = Rotation2d.fromDegrees(180);
//                 if (Robot.is_red_alliance) {
//                         startingRotation = Rotation2d.fromDegrees(0);
//                 }
//                 return new Pose2d(drivePath_A.getInitialPose().getTranslation(), startingRotation);
//         }
// }
