// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double neo550TicksPerRev = 42; // although I think getPosition() might return in native units of rotations so this would then be 1
    public static final double neoTicksPerRev = 42;
    public static final double falconTicksPerRev = 2048;
    public static final boolean competition = false; // if enabled, disables a lot of the automatic shuffleboard values

    public final class DriveK {
        public static final double ticksPerRev = neoTicksPerRev;
        public static final double gearRatio = 1/9.64; //probably wrote that correctly
        public static final double wheelDiameter = 6;

        public static final double distanceP = 0.065;
        public static final double distanceI = 0.002;
        public static final double distanceD = 0.001;

        public static final double headingP = 0.07;
        public static final double headingI = 0;
        public static final double headingD = 0.00001;

        public static final double turnP = 0.05;
        public static final double turnI = 0.000;
        public static final double turnD = 0.009;
    }

    public final class ShootK {
        public static final double shootP = 0.00001;
        public static final double shootI = 0;
        public static final double shootD = 0.000005;
        public static final double shootF = 0.4;

        public static final double normalShot = 9000;
        public static final double againstHub = 8000;
        public static final double lowGoal = 1500;
        public static final double farShot = 12000;
    }

    public final class IntakeK {
        //TODO test and tune this
        public static final double deployP = 0.1;
        public static final double deployI = 0;
        public static final double deployD = 0.01;

        public static final double gearRatio = 1/5 * 1/5 * 1/5; // gear ratio on the intake gearbox
        public static final double rotations = 0.4; // intake needs to rotate 0.4 rotations to reach deployed state

        //TODO test and tune this
        public static final double RAISED = 0; // starting position (aka not extended aka raised) is 0
        public static final double EXTENDED = gearRatio * -rotations; // rotations for intake being extended
    }

    public final class ClimbK {
        //TODO test and tune the entire climber subsystem
        public static final double tiltP = 0;
        public static final double tiltD = 0;
        public static final double tiltI = 0;
        
        public static final double extendP = 0.00001;
        public static final double extendI = 0;
        public static final double extendD = 0.00000001;
        public static final double extendF = 0;
        

        public static final double gearRatio = 1/625;
        public static final double pulleyDiameter = 3; // inches
        public static final double extendLength = 322281; // 30 * falconTicksPerRev  /  Math.PI * pulleyDiameter;
    }

    public final class VisionK {
        //TODO test and tune
        public static final double kP = 0.03;
        public static final double kI = 0;
        public static final double kD = 0.001;
        public static final double heightOfGoal = 0; //TODO change this to be the actual value
        public static final double heightOfCamera = 0; //TODO change this to be the actual value
        public static final double cameraAngle = 0; //TODO change this to be the actual value
    }
}
