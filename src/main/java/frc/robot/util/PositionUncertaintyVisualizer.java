package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class PositionUncertaintyVisualizer {
  private static final int kDefaultEllipseSamples = 32;
  private static final int kDefaultProbabilityPoseSamples = 20;
  private static final double kDefaultCovarianceSigmaScale = 2.0;
  private static final double kDefaultProbabilityMass = 0.95;
  private static final double kGoldenAngleRad = Math.PI * (3.0 - Math.sqrt(5.0));

  public static enum PositionVisualizationMode {
    PROBABILITY_MAP,
    PROBABILITY_ELLIPSE,
    COVARIANCE_ELLIPSE,
  }

  private PositionUncertaintyVisualizer() {
  }

  public static double[][] covarianceMatrix(double[] covariance) {
    if (covariance.length < 9) {
      return new double[0][0];
    }

    return new double[][] {
        { covariance[0], covariance[1], covariance[2] },
        { covariance[3], covariance[4], covariance[5] },
        { covariance[6], covariance[7], covariance[8] }
    };
  }

  public static double[] covarianceDiagonal(double[] covariance) {
    if (covariance.length < 9) {
      return new double[0];
    }

    return new double[] { covariance[0], covariance[4], covariance[8] };
  }

  public static double[] covarianceStdDev(double[] covariance) {
    double[] diagonal = covarianceDiagonal(covariance);
    double[] result = new double[diagonal.length];
    for (int i = 0; i < diagonal.length; i++) {
      result[i] = Math.sqrt(Math.max(0.0, diagonal[i]));
    }
    return result;
  }

  public static Pose2d[] visualization(Pose2d center, double[] covariance, PositionVisualizationMode mode) {
    if (mode == null) {
      return new Pose2d[0];
    }

    switch (mode) {
      case PROBABILITY_MAP:
        return probabilityMap(center, covariance, kDefaultProbabilityMass, kDefaultProbabilityPoseSamples);
      case PROBABILITY_ELLIPSE:
        return probabilityEllipse(center, covariance, kDefaultProbabilityMass, kDefaultEllipseSamples);
      case COVARIANCE_ELLIPSE:
        return covarianceEllipse(center, covariance, kDefaultCovarianceSigmaScale, kDefaultEllipseSamples);
      default:
        return new Pose2d[0];
    }
  }

  private static Pose2d[] probabilityEllipse(Pose2d center, double[] covariance, double probabilityMass,
      int samples) {
    EllipseParameters ellipse = probabilityEllipseParameters(covariance, probabilityMass);
    if (ellipse == null || samples < 4) {
      return new Pose2d[0];
    }

    Pose2d[] outline = new Pose2d[samples + 1];
    for (int i = 0; i <= samples; i++) {
      double t = 2.0 * Math.PI * i / samples;
      double localX = ellipse.majorRadius() * Math.cos(t);
      double localY = ellipse.minorRadius() * Math.sin(t);

      double worldX = center.getX() + localX * Math.cos(ellipse.axisAngleRad())
          - localY * Math.sin(ellipse.axisAngleRad());
      double worldY = center.getY() + localX * Math.sin(ellipse.axisAngleRad())
          + localY * Math.cos(ellipse.axisAngleRad());

      outline[i] = new Pose2d(worldX, worldY, new Rotation2d());
    }
    return outline;
  }

  private static Pose2d[] covarianceEllipse(Pose2d center, double[] covariance, double sigmaScale, int samples) {
    EllipseParameters ellipse = ellipseParameters(covariance, sigmaScale);
    if (ellipse == null || samples < 4) {
      return new Pose2d[0];
    }

    Pose2d[] outline = new Pose2d[samples + 1];
    for (int i = 0; i <= samples; i++) {
      double t = 2.0 * Math.PI * i / samples;
      double localX = ellipse.majorRadius() * Math.cos(t);
      double localY = ellipse.minorRadius() * Math.sin(t);

      double worldX = center.getX() + localX * Math.cos(ellipse.axisAngleRad())
          - localY * Math.sin(ellipse.axisAngleRad());
      double worldY = center.getY() + localX * Math.sin(ellipse.axisAngleRad())
          + localY * Math.cos(ellipse.axisAngleRad());

      outline[i] = new Pose2d(worldX, worldY, new Rotation2d());
    }
    return outline;
  }

  private static Pose2d[] probabilityMap(Pose2d center, double[] covariance, double probabilityMass,
      int samples) {
    EllipseParameters ellipse = probabilityEllipseParameters(covariance, probabilityMass);
    if (ellipse == null || samples < 1) {
      return new Pose2d[0];
    }

    Pose2d[] poses = new Pose2d[samples];
    poses[0] = center;

    for (int i = 1; i < samples; i++) {
      double radiusFraction = Math.sqrt((double) i / (samples - 1));
      double theta = i * kGoldenAngleRad;
      double localX = ellipse.majorRadius() * radiusFraction * Math.cos(theta);
      double localY = ellipse.minorRadius() * radiusFraction * Math.sin(theta);

      double worldX = center.getX() + localX * Math.cos(ellipse.axisAngleRad())
          - localY * Math.sin(ellipse.axisAngleRad());
      double worldY = center.getY() + localX * Math.sin(ellipse.axisAngleRad())
          + localY * Math.cos(ellipse.axisAngleRad());

      poses[i] = new Pose2d(worldX, worldY, center.getRotation());
    }

    return poses;
  }

  private static EllipseParameters probabilityEllipseParameters(double[] covariance, double probabilityMass) {
    double sigmaScale = probabilitySigmaScale(probabilityMass);
    if (!Double.isFinite(sigmaScale)) {
      return null;
    }
    return ellipseParameters(covariance, sigmaScale);
  }

  private static EllipseParameters ellipseParameters(double[] covariance, double sigmaScale) {
    if (covariance.length < 5 || !Double.isFinite(sigmaScale) || sigmaScale < 0.0) {
      return null;
    }

    double a = sanitizedVariance(covariance[0]);
    double b = covarianceXY(covariance);
    double d = sanitizedVariance(covariance[4]);

    double trace = a + d;
    double diff = a - d;
    double root = Math.sqrt(diff * diff + 4.0 * b * b);

    double lambdaMajor = Math.max(0.0, 0.5 * (trace + root));
    double lambdaMinor = Math.max(0.0, 0.5 * (trace - root));

    return new EllipseParameters(
        majorAxisAngleRad(a, b, d, lambdaMajor),
        sigmaScale * Math.sqrt(lambdaMajor),
        sigmaScale * Math.sqrt(lambdaMinor));
  }

  private static double sanitizedVariance(double variance) {
    if (!Double.isFinite(variance)) {
      return 0.0;
    }
    return Math.max(0.0, variance);
  }

  private static double covarianceXY(double[] covariance) {
    double xy = covariance.length > 1 && Double.isFinite(covariance[1]) ? covariance[1] : 0.0;
    double yx = covariance.length > 3 && Double.isFinite(covariance[3]) ? covariance[3] : xy;
    return 0.5 * (xy + yx);
  }

  private static double probabilitySigmaScale(double probabilityMass) {
    if (!Double.isFinite(probabilityMass) || probabilityMass <= 0.0 || probabilityMass >= 1.0) {
      return Double.NaN;
    }

    // 2D Gaussian confidence region from chi-square with 2 dof.
    return Math.sqrt(-2.0 * Math.log(1.0 - probabilityMass));
  }

  private static double majorAxisAngleRad(double a, double b, double d, double lambdaMajor) {
    if (Math.abs(b) < 1e-9) {
      return a >= d ? 0.0 : Math.PI / 2.0;
    }

    double vx = b;
    double vy = lambdaMajor - a;
    if (Math.hypot(vx, vy) < 1e-9) {
      vx = lambdaMajor - d;
      vy = b;
    }
    return Math.atan2(vy, vx);
  }

  private record EllipseParameters(double axisAngleRad, double majorRadius, double minorRadius) {
  }
}
