package com.team8013.frc2025;

public class Ports {

    //public static final String CANBUS_UPPER = "Jonas";
    public static final String CANBUS_LOWER = "Skibidi";

    public static final int DRIVER_PORT = 2;
    public static final int OPERATOR_PORT = 1;

    public static final int PIGEON = 13;

    public static final int ELEVATOR_A = 15;
    public static final int ELEVATOR_B = 14;

    public static final int PIVOT_A = 16;
    public static final int PIVOT_B = 17;
    public static final int PIVOT_CANCODER = 21;

    public static final int WRIST = 18;
    public static final int WRIST_CANCODER = 22;

    public static final int END_EFFECTOR_A = 2; // (Beam break side) //end effector 1 is (2, 42) #2 is (1,43)
    public static final int END_EFFECTOR_B = 42;
    public static final int END_EFFECTOR_BEAM_BREAK = 9;
    public static final int SHOOTER_BEAM_BREAK = 8;

    public static final int Shooter_A = 24;
    public static final int Shooter_B = 23;

    public static final int CLIMBER_HOOK = 25;

    /*** SWERVE MODULE PORTS ***/

    /*
     * Swerve Modules go: (I did this from the view on bottom, if doing from view on
     * top - change drivetrain kinematics in constants to match the configuration)
     * 0 1
     * 2 3
     */

    public static final int FR_DRIVE = 7;
    public static final int FR_ROTATION = 10;
    public static final int FR_CANCODER = 1;

    public static final int FL_DRIVE = 9;
    public static final int FL_ROTATION = 12;
    public static final int FL_CANCODER = 2;

    public static final int BR_DRIVE = 5;
    public static final int BR_ROTATION = 8;
    public static final int BR_CANCODER = 3;

    public static final int BL_DRIVE = 6;
    public static final int BL_ROTATION = 11;
    public static final int BL_CANCODER = 4;

}
