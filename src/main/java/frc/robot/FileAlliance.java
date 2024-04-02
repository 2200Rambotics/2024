package frc.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.OpenOption;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class FileAlliance {
    private FileAlliance() {
    }

    private static final Path FILE_PATH = Paths.get("/home/lvuser/alliance");
    private static final String RED_STRING = "red";
    private static final String BLUE_STRING = "blue";

    public static void writeAlliance(Alliance newAlliance) {
        try {
            switch (newAlliance) {
                case Red:
                    Files.writeString(FILE_PATH, RED_STRING, StandardOpenOption.SYNC,
                            StandardOpenOption.CREATE, StandardOpenOption.WRITE, StandardOpenOption.TRUNCATE_EXISTING);
                    break;
                case Blue:
                    Files.writeString(FILE_PATH, BLUE_STRING, StandardOpenOption.SYNC,
                            StandardOpenOption.CREATE, StandardOpenOption.WRITE, StandardOpenOption.TRUNCATE_EXISTING);
                    break;
                default:
                    break;
            }
        } catch (Exception ex) {
            System.err.println(
                    "FATAL: Could not write to the alliance file `" + FILE_PATH + "`! [`" + ex.getMessage() + "`]");
            System.err.println(ex);
        }
    }

    public static Alliance readAlliance() {
        try {
            String content = Files.readString(FILE_PATH).trim();
            if (content.equalsIgnoreCase(RED_STRING)) {
                return Alliance.Red;
            } else if (content.equalsIgnoreCase(BLUE_STRING)) {
                return Alliance.Blue;
            } else {
                System.err.println("Contents of the alliance file (`" + FILE_PATH +
                        "`) are invalid! Expected `blue`|`red`, got `" + content + "` ");
                writeAlliance(Alliance.Blue);
                return Alliance.Blue;
            }
        } catch (Exception ex) {
            System.err.println("Alliance file (`" + FILE_PATH + "`) cannot be accessed! [`" + ex.getMessage() + "`]");
            System.err.println(ex);
            writeAlliance(Alliance.Blue);
            return Alliance.Blue;
        }
    }
}
