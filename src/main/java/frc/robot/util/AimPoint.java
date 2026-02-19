package frc.robot.util;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constant.FieldConstants;

public final class AimPoint {

  public enum ZoneName {
    LEFT_CENTER,
    RIGHT_CENTER,
    FRONT_OF_HUB,
  }

  private static final double FIELD_LENGTH_METERS = FieldConstants.kFieldLayout.getFieldLength();
  private static final double FIELD_WIDTH_METERS = FieldConstants.kFieldLayout.getFieldWidth();

  private static final List<Zone> ZONES = List.of(
      new Zone(ZoneName.FRONT_OF_HUB,
          atFieldPercent(0.0, 0.0),
          atFieldPercent(0.25, 1.00),
          atFieldPercent(0.30, 0.50)),
      new Zone(
          ZoneName.LEFT_CENTER,
          atFieldPercent(0.25, 0.00),
          atFieldPercent(0.50, 0.50),
          atFieldPercent(0.05, 0.25)),
      new Zone(
          ZoneName.RIGHT_CENTER,
          atFieldPercent(0.25, 0.50),
          atFieldPercent(0.50, 1.00),
          atFieldPercent(0.05, 0.75)));

  private AimPoint() {
  }

  public static ZoneName getZone(Pose2d pose) {
    return getZone(pose, DriverStation.getAlliance());
  }

  public static ZoneName getZone(Pose2d pose, Optional<Alliance> alliance) {
    Pose2d bluePose = toBluePerspective(pose, alliance);

    for (Zone zone : ZONES) {
      if (zone.contains(bluePose)) {
        return zone.name();
      }
    }

    Translation2d robotPosition = bluePose.getTranslation();
    Zone nearest = ZONES.get(0);
    double nearestDistance = Double.POSITIVE_INFINITY;
    for (Zone zone : ZONES) {
      double distance = robotPosition.getDistance(zone.center());
      if (distance < nearestDistance) {
        nearestDistance = distance;
        nearest = zone;
      }
    }
    return nearest.name();
  }

  public static Translation2d getTarget(Pose2d pose) {
    return getTarget(pose, DriverStation.getAlliance());
  }

  public static Translation2d getTarget(Pose2d pose, Optional<Alliance> alliance) {
    return getTarget(getZone(pose, alliance), alliance);
  }

  public static Translation2d getTarget(ZoneName zoneName) {
    return getTarget(zoneName, DriverStation.getAlliance());
  }

  public static Translation2d getTarget(ZoneName zoneName, Optional<Alliance> alliance) {
    return fromBluePerspective(getBlueTarget(zoneName), alliance);
  }

  private static Translation2d getBlueTarget(ZoneName zoneName) {
    for (Zone zone : ZONES) {
      if (zone.name() == zoneName) {
        return zone.target();
      }
    }
    return ZONES.get(0).target();
  }

  public static void logZoneForAdvantageScope(ZoneName zoneName) {
    logZoneForAdvantageScope(zoneName, "AimPoint/Zones/" + zoneName);
  }

  public static void logZoneForAdvantageScope(ZoneName zoneName, String keyPrefix) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    Zone zone = getBlueZone(zoneName);
    Translation2d min = zone.minCorner();
    Translation2d max = zone.maxCorner();

    Translation2d c0 = fromBluePerspective(new Translation2d(min.getX(), min.getY()), alliance);
    Translation2d c1 = fromBluePerspective(new Translation2d(max.getX(), min.getY()), alliance);
    Translation2d c2 = fromBluePerspective(new Translation2d(max.getX(), max.getY()), alliance);
    Translation2d c3 = fromBluePerspective(new Translation2d(min.getX(), max.getY()), alliance);
    Translation2d target = fromBluePerspective(zone.target(), alliance);

    Pose2d[] outline = new Pose2d[] {
        new Pose2d(c0, new Rotation2d()),
        new Pose2d(c1, new Rotation2d()),
        new Pose2d(c2, new Rotation2d()),
        new Pose2d(c3, new Rotation2d()),
        new Pose2d(c0, new Rotation2d())
    };

    Logger.recordOutput(keyPrefix + "/Outline", outline);
    Logger.recordOutput(keyPrefix + "/Target", new Pose2d(target, new Rotation2d()));
    Logger.recordOutput(keyPrefix + "/Name", zoneName.toString());
  }

  private static Translation2d atFieldPercent(double xPercent, double yPercent) {
    return new Translation2d(FIELD_LENGTH_METERS * xPercent, FIELD_WIDTH_METERS * yPercent);
  }

  private static Pose2d toBluePerspective(Pose2d fieldPose, Optional<Alliance> alliance) {
    if (!isRed(alliance)) {
      return fieldPose;
    }
    return new Pose2d(
        flipX(fieldPose.getX()),
        fieldPose.getY(),
        fieldPose.getRotation());
  }

  private static Translation2d fromBluePerspective(Translation2d bluePoint, Optional<Alliance> alliance) {
    if (!isRed(alliance)) {
      return bluePoint;
    }
    return new Translation2d(flipX(bluePoint.getX()), bluePoint.getY());
  }

  private static boolean isRed(Optional<Alliance> alliance) {
    return alliance.isPresent() && alliance.get() == Alliance.Red;
  }

  private static double flipX(double xMeters) {
    return FIELD_LENGTH_METERS - xMeters;
  }

  private static Zone getBlueZone(ZoneName zoneName) {
    for (Zone zone : ZONES) {
      if (zone.name() == zoneName) {
        return zone;
      }
    }
    return ZONES.get(0);
  }

  private record Zone(ZoneName name, Translation2d minCorner, Translation2d maxCorner, Translation2d target) {

    boolean contains(Pose2d pose) {
      Translation2d translation = pose.getTranslation();
      return translation.getX() >= minCorner.getX()
          && translation.getX() < maxCorner.getX()
          && translation.getY() >= minCorner.getY()
          && translation.getY() < maxCorner.getY();
    }

    Translation2d center() {
      return minCorner.interpolate(maxCorner, 0.5);
    }
  }
}
