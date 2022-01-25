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
    public final class DriveK {
        public static final double ticksPerRev = 2048;
        public static final double gearRatio = 1;
        public static final double wheelDiameter = 6;

        public static final double distanceP = 0.1;
        public static final double distanceI = 0;
        public static final double distanceD = 0.001;

        public static final double headingP = 0;
        public static final double headingI = 0;
        public static final double headingD = 0;

        public static final double turnP = 0;
        public static final double turnI = 0;
        public static final double turnD = 0;
    }

    public final class ArmK {

    }

    public final class ShootK {
        public static final double shootP = 0.5;
        public static final double shootI = 0;
        public static final double shootD = 0.01;
        public static final double shootF = 0;

        public static final double preset1 = 1000; // in RPM, close launchpad distance
        public static final double preset2 = 2000; // in RPM, far launchpad distance
        public static final double tarmacPreset = 500; //TODO test and tune
    }

    public final class IntakeK {
        public static final double deployP = 0.1;
        public static final double deployI = 0;
        public static final double deployD = 0.01;

        public static final double RAISED = 0; // ticks for intake at raised position (actually this would be 0)
        public static final double EXTENDED = 1000; // ticks for intake being extended
    }

    public final class ClimbK {
        public static final double leftP = 0.1;
        public static final double leftI = 0.1;
        public static final double leftD = 0.1;

        public static final double rightP = 0.1;
        public static final double rightI = 0.1;
        public static final double rightD = 0.1;
    }

    public final class VisionK {
        public static final double kP = 0.1;
        public static final double kI = 0.1;
        public static final double kD = 0.1;
    }
}
