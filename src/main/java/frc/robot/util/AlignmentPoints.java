package frc.robot.util;

import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import lombok.Getter;
import lombok.Setter;

public class AlignmentPoints {
  @Setter
  public static double kFieldWidth = 6.0;
  @Setter
  public static double kFieldLength = 6.0;

  private static AlignmentMap POINTS;

  /**
   * Functional interface for providing the current robot position.
   * Implement this to integrate with your odometry/position tracking system.
   */
  @FunctionalInterface
  public interface PositionSupplier {
    Pose2d get();
  }

  private static PositionSupplier positionSupplier = () -> new Pose2d();

  /**
   * Set the position supplier for closest point calculations.
   * @param supplier A supplier that returns the current robot Pose2d
   */
  public static void setPositionSupplier(PositionSupplier supplier) {
    positionSupplier = supplier;
  }

  public static void setPoints(AlignmentMap points) {
    POINTS = points;
  }

  @Getter
  public static class AlignmentMap {
    private final Map<String, Map<String, Map<String, Pose2d>>> points = new HashMap<>();

    public AlignmentMap category(String name, CategoryBuilder builder) {
      points.put(name, builder.build());
      return this;
    }

    public Optional<Pose2d> get(String category, String subCategory, String pointName) {
      return Optional.ofNullable(points.get(category))
          .map(cat -> cat.get(subCategory))
          .map(sub -> sub.get(pointName));
    }
  }

  public static class CategoryBuilder {
    private final Map<String, Map<String, Pose2d>> subCategories = new HashMap<>();

    public CategoryBuilder subCategory(String name, SubCategoryBuilder builder) {
      subCategories.put(name, builder.build());
      return this;
    }

    Map<String, Map<String, Pose2d>> build() {
      return subCategories;
    }
  }

  public static class SubCategoryBuilder {
    private final Map<String, Pose2d> points = new HashMap<>();
    @Getter
    private Pose2d center;

    public SubCategoryBuilder point(String name, double x, double y, double rotationRadians) {
      points.put(name, new Pose2d(x, y, new Rotation2d(rotationRadians)));
      return this;
    }

    public SubCategoryBuilder point(String name, Pose2d pose) {
      points.put(name, pose);
      return this;
    }

    Map<String, Pose2d> build() {
      double avgX = 0, avgY = 0, avgTheta = 0;
      int count = points.size();
      if (count > 0) {
        avgX = points.values().stream().mapToDouble(Pose2d::getX).sum() / count;
        avgY = points.values().stream().mapToDouble(Pose2d::getY).sum() / count;
        double sumSin = points.values().stream().mapToDouble(p -> Math.sin(p.getRotation().getRadians())).sum();
        double sumCos = points.values().stream().mapToDouble(p -> Math.cos(p.getRotation().getRadians())).sum();
        avgTheta = Math.atan2(sumSin / count, sumCos / count);
      }
      center = new Pose2d(avgX, avgY, new Rotation2d(avgTheta));

      return points;
    }
  }

  private static Alliance alliance = Alliance.Blue;

  public static void setAlliance(Alliance fieldSide) {
    alliance = fieldSide;
  }

  public static Optional<Pose2d> getPoint(String category, String subCategory, String pointName) {
    return POINTS.get(category, subCategory, pointName)
        .map(pose -> alliance == Alliance.Red ? mirrorPose(pose) : pose);
  }

  public static Optional<Pose2d> getPoint(String category) {
    var subString = category.split(":");
    if (subString.length != 3) {
      return Optional.empty();
    }

    String categoryName = subString[1];
    if (categoryName.toLowerCase().equals("closest")) {
      categoryName = getClosestCategory(subString[0], positionSupplier.get()).orElse(null);
      if (categoryName == null) {
        return Optional.empty();
      }
    }

    String pointName = subString[2];
    if (pointName.toLowerCase().equals("closest")) {
      pointName = getClosestPoint(subString[0], categoryName, positionSupplier.get()).orElse(null);
      if (pointName == null) {
        return Optional.empty();
      }
    }

    return getPoint(subString[0], categoryName, pointName);
  }

  private static Optional<String> getClosestPoint(String category, String subCategory, Pose2d position) {
    var points = POINTS.getPoints().get(category).get(subCategory);
    if (points == null) {
      return Optional.empty();
    }
    return points.entrySet().stream()
        .min(Comparator.comparingDouble(
            entry -> entry.getValue().getTranslation().getDistance(position.getTranslation())))
        .map(Map.Entry::getKey);
  }

  private static Optional<String> getClosestCategory(String category, Pose2d position) {
    var cat = POINTS.getPoints().get(category);
    if (cat == null) {
      return Optional.empty();
    }
    return cat.entrySet().stream()
        .min(Comparator.comparingDouble(
            entry -> calculateCenter(entry.getValue()).getTranslation().getDistance(position.getTranslation())))
        .map(Map.Entry::getKey);
  }

  private static Pose2d calculateCenter(Map<String, Pose2d> points) {
    double avgX = 0, avgY = 0, avgTheta = 0;
    int count = points.size();
    if (count > 0) {
      avgX = points.values().stream().mapToDouble(Pose2d::getX).sum() / count;
      avgY = points.values().stream().mapToDouble(Pose2d::getY).sum() / count;
      double sumSin = points.values().stream().mapToDouble(p -> Math.sin(p.getRotation().getRadians())).sum();
      double sumCos = points.values().stream().mapToDouble(p -> Math.cos(p.getRotation().getRadians())).sum();
      avgTheta = Math.atan2(sumSin / count, sumCos / count);
    }
    return new Pose2d(avgX, avgY, new Rotation2d(avgTheta));
  }

  private static Pose2d mirrorPose(Pose2d pose) {
    return new Pose2d(
        kFieldLength - pose.getX(),
        kFieldWidth - pose.getY(),
        pose.getRotation().rotateBy(new Rotation2d(Math.PI)));
  }
}
