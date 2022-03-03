package frc.robot.util;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.Properties;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * Values
 */
public class Values {

    private static Properties properties;


    public static void init() {
        properties = new Properties();

        try {
            properties.load(new FileInputStream(Filesystem.getDeployDirectory() + "/properties.cfg"));
            System.out.println(Filesystem.getDeployDirectory() + "/properties.cfg");
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
			e.printStackTrace();
        }
    }

    public static String getString(String key) {
        return properties.getProperty(key);
    }

    public static Double getNumber(String key) {
        String number = properties.getProperty(key);

        try {
            return Double.valueOf(number);
        } catch (NullPointerException e) {
            return null;
        } catch (NumberFormatException e) {
            return null;
        }
    }
}