package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto extends SequentialCommandGroup {
    public String name;
    public String desc;
    public int points;

    /**
     * Creates a new Auto, which is a wrapper for SequentialCommandGroup.
     * This class is here so our auto chooser is extra-fancy. Should require very little change
     * in the way we write autos. All you have to do is add the describing information to the constructor
     * 
     * @param name the name of the auto
     * @param desc a short description of what the auto does
     * @param points how many points the auto scores, although idk how useful this is
     * @param commands an arbitrary number of commands, to run in sequence
     */
    public Auto(String name, String desc, int points, Command... commands) {
        super(commands);
        
        this.name = name;
        this.desc = desc;
        this.points = points;
    }
}