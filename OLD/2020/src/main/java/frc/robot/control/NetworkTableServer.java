package frc.robot.control;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;
import frc.robot.pathfinding.Node;

/**
 * NetworkTableServer
 */
public class NetworkTableServer {

    NetworkTableEntry xPositionEntry;
    NetworkTableEntry yPositionEntry;
    NetworkTableEntry angle;
    int index = 0;

    public void init() {
        
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable table = instance.getTable("robotData");

        xPositionEntry = table.getEntry("robot_x_position");
        yPositionEntry = table.getEntry("robot_y_position");
        angle = table.getEntry("robot_angle");

    }

    public void sendPackets(Robot robot) {
        xPositionEntry.setDouble(robot.getxPosition());
        yPositionEntry.setDouble(robot.getyPosition());
        angle.setDouble(robot.getAngle());

        // System.out.println("sending " + robot.angle);
        // System.out.println("sending packets...");
    }

}