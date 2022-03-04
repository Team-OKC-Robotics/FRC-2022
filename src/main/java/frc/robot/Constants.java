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
    public static final double neo550TicksPerRev = 42;
    public static final double neoTicksPerRev = 42;
    public static final double falconTicksPerRev = 2048;

    public final class DriveK {
        public static final double ticksPerRev = neoTicksPerRev;
        public static final double gearRatio = 9.64/1; //probably wrote that correctly
        public static final double wheelDiameter = 6;

        public static final double distanceP = 0.1;
        public static final double distanceI = 0;
        public static final double distanceD = 0.001;

        public static final double headingP = 0.03;
        public static final double headingI = 0;
        public static final double headingD = 0;

        public static final double turnP = 0.1;
        public static final double turnI = 0;
        public static final double turnD = 0;
    }

    public final class ShootK {
        //TODO test and tune this
        public static final double shootP = 0.5;
        public static final double shootI = 0;
        public static final double shootD = 0.01;
        public static final double shootF = 0;

        public static final double preset1 = 9000; // in RPM, close launchpad distance
        public static final double preset2 = 18000; // in RPM, far launchpad distance
        public static final double tarmacPreset = 1500; //TODO test and tune
        public static final double lowGoalPreset = 1000;
    }

    public final class IntakeK {
        //TODO test and tune this
        public static final double deployP = 0.1;
        public static final double deployI = 0;
        public static final double deployD = 0.01;

        //TODO test and tune this
        public static final double RAISED = 0; // ticks for intake at raised position (actually this would be 0)
        public static final double EXTENDED = neo550TicksPerRev ; // ticks for intake being extended

        public static final float maxDeploy = 1000000;
    }

    public final class ClimbK {
        //TODO test and tune the entire climber subsystem
        public static final double leftTiltP = 0;
        public static final double leftTiltD = 0;
        public static final double leftTiltI = 0;
        
        public static final double leftExtendP = 0.00001;
        public static final double leftExtendI = 0;
        public static final double leftExtendD = 0.00000001;
        public static final double leftExtendF = 0;
        
        public static final double rightTiltP = 0;
        public static final double rightTiltI = 0;
        public static final double rightTiltD = 0;
        
        public static final double rightExtendP = 0;
        public static final double rightExtendI = 0;
        public static final double rightExtendD = 0;
        public static final double rightExtendF = 0;


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

    public static final double logTime = 0.1; // 100 milliseconds
}
