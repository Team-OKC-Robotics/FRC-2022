package frc.robot.util;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.RobotController;

public class Logger {
    private FileWriter out;
    private BufferedWriter writer;

    public enum Subsystem {
        Drivetrain,
        Shooter,
        Intake,
        Climber,
        Vision
    }

    /**
     * Requirements
        A logging system should start as a general framework that can be used by various subsystems
        When the code starts, the logging system should create a new folder in sd card memory for the logs of each subsystem to be placed
        Each subsystem should log all of its data to a separate log file in the aforementioned folder at a rate consistent with it's operation.
        Logged data should have a timestamp in seconds from the initial time the robot was turned on
        Data should be logged in CSV format for future analysis
        Data logged should include: sensor measurements, subsystem state, user inputs, controller states, controller input/output
     */
    public Logger(String subsystem, int matchNumber) throws IOException {
        out = new FileWriter(subsystem + matchNumber + ".csv");
        writer = new BufferedWriter(out);
    }

    public void headers(String headers) {
        writer.write(headers);
    }

    public void newline() {
        writer.write("\n" + RobotController.getFPGATime());
    }

    public void log(String name, double data) {
        writer.write(data + ",");
    }

    // public void log(String data, Subsystem subsystem) throws IOException {
    //     writer.write(RobotController.getFPGATime());
    // }

    // public void log(String data) throws IOException {
    //     writer.write(data);
    // }
}
