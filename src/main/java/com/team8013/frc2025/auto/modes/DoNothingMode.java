package com.team8013.frc2025.auto.modes;

import com.team8013.frc2025.auto.AutoModeBase;
import com.team8013.frc2025.auto.AutoModeEndedException;

import edu.wpi.first.math.geometry.Pose2d;

public class DoNothingMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("doing nothing");
    }

    @Override
    public Pose2d getStartingPose() {
        return new Pose2d();
    }
}
