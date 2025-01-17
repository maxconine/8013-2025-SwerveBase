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
// import com.team8013.frc2025.shuffleboard.ShuffleBoardInteractions;
// import com.team8013.frc2025.subsystems.Drive;
// import com.team8013.frc2025.subsystems.Superstructure;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.trajectory.Trajectory;

// public class FourPieceMiddleStart extends AutoModeBase {

//         private Superstructure mSuperstructure;

//         // required PathWeaver trajectory paths
//         String path_A = "paths/2024Paths/TwoMiddleSmooth.path";
//         String path_B = "paths/2024Paths/AmpSideSmooth.path";
//         String path_C = "paths/2024Paths/StageSideSmooth.path";

//         // trajectories
//         SwerveTrajectoryAction pathA;
//         final Trajectory drivePath_A;

//         SwerveTrajectoryAction pathB;
//         final Trajectory drivePath_B;

//         SwerveTrajectoryAction pathC;
//         final Trajectory drivePath_C;

//         public FourPieceMiddleStart() {
//                 mSuperstructure = Superstructure.getInstance();

//                 // read trajectories from PathWeaver and generate trajectory actions
//                 drivePath_A = AutoTrajectoryReader.generateTrajectoryFromFile(path_A,
//                                 Constants.AutoConstants.createConfig(1.2, 1.3, 0.0, 0));
//                 pathA = new SwerveTrajectoryAction(drivePath_A, Rotation2d.fromDegrees(180));
//                 ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drivePath_A);

//                 drivePath_B = AutoTrajectoryReader.generateTrajectoryFromFile(path_B,
//                                 Constants.AutoConstants.createConfig(3.2, 2, 0.0, 0));
//                 pathB = new SwerveTrajectoryAction(drivePath_B, Rotation2d.fromDegrees(180));
//                 ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drivePath_B);

//                 drivePath_C = AutoTrajectoryReader.generateTrajectoryFromFile(path_C,
//                                 Constants.AutoConstants.createConfig(1.5, 1.45, 0.0, 0));
//                 pathC = new SwerveTrajectoryAction(drivePath_C, Rotation2d.fromDegrees(180));
//                 ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drivePath_C);
//         }

//         @Override
//         protected void routine() throws AutoModeEndedException {
//                 runAction(new LambdaAction(() -> Drive.getInstance().resetOdometry(getStartingPose())));

//                 System.out.println("Running 4 note auto");
//                 mSuperstructure.autoShot();
//                 runAction(new WaitAction(1.15));

//                 runAction(new ParallelAction(List.of(
//                                 pathA,
//                                 new SeriesAction(List.of(
//                                                 new WaitAction(0.05),
//                                                 new LambdaAction(() -> Drive.getInstance()
//                                                                 .setAutoHeading(Rotation2d.fromDegrees(2))),
//                                                 new WaitAction(0.25),
//                                                 new LambdaAction(() -> mSuperstructure
//                                                                 .setSuperstuctureIntakingGround()),
//                                                 new WaitAction(1.25),
//                                                 new LambdaAction(() -> mSuperstructure
//                                                                 .setSuperstuctureStow()),
//                                                 new WaitAction(0.05),
//                                                 new LambdaAction(() -> Drive.getInstance()
//                                                                 .setAutoHeading(Rotation2d.fromDegrees(180))),
//                                                 new WaitAction(0.15),
//                                                 new LambdaAction(() -> mSuperstructure
//                                                                 .setSuperstuctureTransferToShooter()))))));
//                 if (mSuperstructure.hasGamePiece()) {
//                         mSuperstructure.autoShot();
//                         runAction(new WaitAction(0.3));
//                 }

//                 runAction(new ParallelAction(List.of(
//                                 pathB,
//                                 new SeriesAction(List.of(

//                                                 new WaitAction(0.1),
//                                                 new LambdaAction(() -> mSuperstructure.disableAutoShot()),
//                                                 new LambdaAction(() -> Drive.getInstance()
//                                                                 .setAutoHeading(Rotation2d.fromDegrees(270))),

//                                                 new WaitAction(0.35),
//                                                 new LambdaAction(() -> mSuperstructure
//                                                                 .setSuperstuctureIntakingGround()),
//                                                 new WaitAction(1.4),
//                                                 new LambdaAction(() -> mSuperstructure
//                                                                 .setSuperstuctureStow()),
//                                                 new WaitAction(0.15),
//                                                 new LambdaAction(() -> Drive.getInstance()
//                                                                 .setAutoHeading(Rotation2d.fromDegrees(180))),
//                                                 new WaitAction(0.1),
//                                                 new LambdaAction(() -> mSuperstructure
//                                                                 .setSuperstuctureTransferToShooter()))))));
//                 if (mSuperstructure.hasGamePiece()) {
//                         mSuperstructure.autoShot();
//                         runAction(new WaitAction(0.3));
//                 }
//                 runAction(new ParallelAction(List.of(
//                                 pathC,
//                                 new SeriesAction(List.of(

//                                                 new WaitAction(0.05),
//                                                 new LambdaAction(() -> mSuperstructure.disableAutoShot()),
//                                                 new LambdaAction(() -> Drive.getInstance()
//                                                                 .setAutoHeading(Rotation2d.fromDegrees(20))),

//                                                 new WaitAction(0.3),
//                                                 new LambdaAction(() -> mSuperstructure
//                                                                 .setSuperstuctureIntakingGround()),
//                                                 new WaitAction(1),
//                                                 new LambdaAction(() -> Drive.getInstance()
//                                                                 .setAutoHeading(Rotation2d.fromDegrees(12))),
//                                                 new WaitAction(0.3),
//                                                 new LambdaAction(() -> mSuperstructure
//                                                                 .setSuperstuctureStow()),
//                                                 new WaitAction(0.1),
//                                                 new LambdaAction(() -> Drive.getInstance()
//                                                                 .setAutoHeading(Rotation2d.fromDegrees(180))),
//                                                 new WaitAction(0.15),
//                                                 new LambdaAction(() -> mSuperstructure
//                                                                 .setSuperstuctureTransferToShooter()))))));

//                 mSuperstructure.autoShot();
//                 runAction(new WaitAction(0.45));
//                 mSuperstructure.setSuperstuctureStow();
//                 mSuperstructure.disableAutoShot();

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
