package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto extends SequentialCommandGroup {
    public String name;
    public String desc;
    public int points;

    public Auto(String name, String desc, int points, Command commands...) {
        this.name = name;
        this.desc = desc;
        this.points = points;
        
        super(commands);
    }
}