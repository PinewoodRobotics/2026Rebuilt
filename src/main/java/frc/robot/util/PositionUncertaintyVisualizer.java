package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class PositionUncertaintyVisualizer {
  private static final int kDefaultEllipseSamples = 32;
  private static final double kDefaultSigmaScale = 2.0;

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

  public static Pose2d[] covarianceEllipse(Pose2d center, double[] covariance) {
    return covarianceEllipse(center, covariance, kDefaultSigmaScale, kDefaultEllipseSamples);
  }

  public static Pose2d[] covarianceEllipse(Pose2d center, double[] covariance, double sigmaScale, int samples) {
    if (covariance.length < 5 || samples < 4) {
      return new Pose2d[0];
    }

    double a = Math.max(0.0, covariance[0]);
    double b = covariance[1];
    double d = Math.max(0.0, covariance[4]);

    double trace = a + d;
    double diff = a - d;
    double root = Math.sqrt(diff * diff + 4.0 * b * b);

    double lambdaMajor = Math.max(0.0, 0.5 * (trace + root));
    double lambdaMinor = Math.max(0.0, 0.5 * (trace - root));

    double axisAngle = majorAxisAngleRad(a, b, d, lambdaMajor);
    double majorRadius = sigmaScale * Math.sqrt(lambdaMajor);
    double minorRadius = sigmaScale * Math.sqrt(lambdaMinor);

    Pose2d[] outline = new Pose2d[samples + 1];
    for (int i = 0; i <= samples; i++) {
      double t = 2.0 * Math.PI * i / samples;
      double localX = majorRadius * Math.cos(t);
      double localY = minorRadius * Math.sin(t);

      double worldX = center.getX() + localX * Math.cos(axisAngle) - localY * Math.sin(axisAngle);
      double worldY = center.getY() + localX * Math.sin(axisAngle) + localY * Math.cos(axisAngle);

      outline[i] = new Pose2d(worldX, worldY, new Rotation2d());
    }
    return outline;
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
}
