import type {
  GenericMatrix,
  GenericVector,
} from "generated/thrift/gen-nodejs/common_types";

export type TransformationMatrix3D = GenericMatrix;

/**
 * Converts a WPILib quaternion (WXYZ order, NWU coordinate system) to a rotation matrix.
 * WPILib uses NWU: X=forward/North, Y=left/West, Z=up
 *
 * The quaternion represents the rotation of the tag's coordinate frame in the NWU world frame.
 * We convert it directly to a rotation matrix since our backend also uses a similar coordinate system.
 */
export function fromQuaternionNoRoll_ZYX(q: number[]): GenericMatrix {
  let [w, x, y, z] = q;
  const n = Math.sqrt(w * w + x * x + y * y + z * z) || 1;
  w /= n;
  x /= n;
  y /= n;
  z /= n;

  // Convert quaternion (WXYZ) to rotation matrix
  // Standard quaternion to rotation matrix conversion
  const r11 = 1 - 2 * (y * y + z * z);
  const r12 = 2 * (x * y - w * z);
  const r13 = 2 * (x * z + w * y);
  const r21 = 2 * (x * y + w * z);
  const r22 = 1 - 2 * (x * x + z * z);
  const r23 = 2 * (y * z - w * x);
  const r31 = 2 * (x * z - w * y);
  const r32 = 2 * (y * z + w * x);
  const r33 = 1 - 2 * (x * x + y * y);

  return MatrixUtil.buildMatrix([
    [r11, r12, r13],
    [r21, r22, r23],
    [r31, r32, r33],
  ]);
}

export class MatrixUtil {
  static createTransformationMatrix3D(
    rotation: GenericMatrix,
    translation: GenericVector,
  ): GenericMatrix {
    return {
      values: [
        [
          rotation.values[0][0],
          rotation.values[0][1],
          rotation.values[0][2],
          translation.values[0],
        ],
        [
          rotation.values[1][0],
          rotation.values[1][1],
          rotation.values[1][2],
          translation.values[1],
        ],
        [
          rotation.values[2][0],
          rotation.values[2][1],
          rotation.values[2][2],
          translation.values[2],
        ],
        [0, 0, 0, 1],
      ],
      rows: 4,
      cols: 4,
    };
  }

  /**
   *
   * @param array [[1, 2, 3], [4, 5, 6], [7, 8, 9]] --> 3x3 matrix with [1, 2, 3] as the first row
   * @returns
   */
  static buildMatrix(array: number[][]): GenericMatrix {
    return {
      values: array,
      rows: array.length,
      cols: array[0].length,
    } as GenericMatrix;
  }

  static buildRotationMatrixFromYaw(yawDegrees: number): GenericMatrix {
    const yawRadians = (yawDegrees * Math.PI) / 180;
    const cos = Math.cos(yawRadians);
    const sin = Math.sin(yawRadians);

    return {
      values: [
        [cos, -sin, 0],
        [sin, cos, 0],
        [0, 0, 1],
      ],
      rows: 3,
      cols: 3,
    } as GenericMatrix;
  }

  static buildMatrixFromDiagonal(diagonal: number[]): GenericMatrix {
    const size = diagonal.length;
    const values = Array.from({ length: size }, (_, i) =>
      Array.from({ length: size }, (_, j) => (i === j ? diagonal[i] : 0)),
    );
    return {
      values,
      rows: size,
      cols: size,
    } as GenericMatrix;
  }
}

export class VectorUtil {
  /**
   *
   * @param array [1, 2, 3] --> x = 1, y = 2, z = 3
   * @returns
   */
  static fromArray(array: number[]): GenericVector {
    return {
      values: array,
      size: array.length,
    } as GenericVector;
  }
}
