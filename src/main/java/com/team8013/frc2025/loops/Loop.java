package com.team8013.frc2025.loops;

/**
 * Interface for loops, which are routine that run periodically in the robot code (such as periodic gyroscope
 * calibration, etc.)
 */
public interface Loop {

    public void onStart(double timestamp);

    public void onLoop(double timestamp);

    public void onStop(double timestamp);
}
