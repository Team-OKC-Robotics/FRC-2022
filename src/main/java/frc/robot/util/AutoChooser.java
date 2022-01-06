package frc.robot.util;

/**
 * Our AutoChooser. Sends a list of autos to Shuffleboard, showing information
 * such as description and points, and the driver can choose which auto they want to run
 * using the gamepad d-pad (aka 'POV').
 */
public class AutoChooser {
    private static ArrayList<Auto> autos;
    private static int index = 0;
    private static Joystick gamepad;
    private static wasPressed = false;
    
    private static ShuffleboardTab tab = Shuffleboard.getTab("autoChooser");
    private static NetworkTableEntry autoName = tab.addPersistent("Auto", "no auto selected").getEntry();
    private static NetworkTableEntry autoPoints = tab.addPersistent("Auto Points", 0.0).getEntry();
    private static NetworkTableEntry autoDesc = tab.addPersistent("Auto Description", "no auto selected").getEntry();
    private static NetworkTableEntry allAutos = tab.addPersistent("All Autos", "no autos loaded").getEntry();

    /**
     * Adds all of the autos to the chooser
     * @param autos the autos to add
     */
    public static addAutos(Auto autos...) {
        autos = new ArrayList<>();
        for (Auto auto : autos) {
            autos.add(autos);
        }
    }

    /**
     * Adds the gamepad to the chooser so the driver can change which auto to run
     * @param gamepad the gamepad instance
     */
    public static void addGamepad(Joystick gamepad) {
        this.gamepad = gamepad;
    }

    /**
     * Gets the auto command to be run in autonomous mode. Used in Robot.java
     * @return the command to be run in auto
     */
    public static Command getAutoCommand() {
        return autos.get(index);
    }

    /**
     * Updates the auto chooser. Put this in disabledPeriodic() so it's called once per iteration while we're waiting
     * for the match to start
     */
    public static void update() {
        int pov = gamepad.getPOV();
        if(pov == -1) {
            wasPressed = false;
        } else if (!wasPressed) {
            //180 is up and 0 is down
            if(pov == 180 && index < autos.size()-1) { // this can be optimized, be pre-calculating the size
                tempCurrAuto += 1;
                wasPressed = true;
            } else if (pov == 0 && index > 0) {
                tempCurrAuto -= 1;
                wasPressed = true;
            }
        }


        CurrentAuto.setString(robotContainer.getName(tempCurrAuto));

        autoName.setString(autos.get(index).name);
        autoPoints.setDouble(autos.get(index).points);
        autoDesc.setString(autos.get(index).description);

        // this should probably be optimized
        allAutos.setString(autos); // this is probably too expensive to do every iteration. Then again, it's not like we're doing much in disabled anyways
    }
}