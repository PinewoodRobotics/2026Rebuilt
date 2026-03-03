package frc.robot.util;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Filesystem;

public class PathedAuto extends PathPlannerAuto {
  private final List<PathPlannerPath> paths = new ArrayList<>();

  public PathedAuto(String name, boolean shouldFlip) {
    super(name, shouldFlip);

    File autoFile = getAutoFile(name);

    JSONObject autoJson;
    try {
      autoJson = getAutoJson(autoFile);
    } catch (IOException | ParseException e) {
      e.printStackTrace();
      return;
    }

    JSONObject commandObj = asObject(autoJson.get("command"));
    if (commandObj == null)
      return;

    collectPathsFromCommand(commandObj);
  }

  public List<PathPlannerPath> getPaths() {
    return paths;
  }

  private File getAutoFile(String autoName) {
    return new File(Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoName + ".auto");
  }

  private JSONObject getAutoJson(File autoFile) throws IOException, ParseException {
    String rawJson = Files.readString(autoFile.toPath());
    return (JSONObject) new JSONParser().parse(rawJson);
  }

  private void collectPathsFromCommand(JSONObject commandObj) {
    String type = asString(commandObj.get("type"));
    if (type == null)
      return;

    if (type.equals("path")) {
      JSONObject data = asObject(commandObj.get("data"));
      if (data == null)
        return;

      String pathName = asString(data.get("pathName"));
      if (pathName == null)
        return;

      try {
        paths.add(PathPlannerPath.fromPathFile(pathName));
      } catch (Exception e) {
        e.printStackTrace();
      }
      return;
    }

    JSONObject data = asObject(commandObj.get("data"));
    if (data == null)
      return;

    JSONArray commands = asArray(data.get("commands"));
    if (commands == null)
      return;

    for (Object child : commands) {
      JSONObject childObj = asObject(child);
      if (childObj != null) {
        collectPathsFromCommand(childObj);
      }
    }
  }

  private static JSONObject asObject(Object o) {
    return (o instanceof JSONObject) ? (JSONObject) o : null;
  }

  private static JSONArray asArray(Object o) {
    return (o instanceof JSONArray) ? (JSONArray) o : null;
  }

  private static String asString(Object o) {
    return (o instanceof String) ? (String) o : null;
  }
}