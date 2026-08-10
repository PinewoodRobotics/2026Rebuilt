import * as fs from "fs";
import type {
  GenericMatrix,
  GenericVector,
} from "generated/thrift/gen-nodejs/common_types";
import { fromQuaternionNoRoll_ZYX, VectorUtil } from "./math";

export type WPILibTagPose = {
  position: GenericVector;
  rotation: GenericMatrix;
};

/** Same shape as tag map exports under `src/config/pos_extrapolator/tag_config/`. */
export type WPILibFieldTagMap = Record<number, WPILibTagPose>;

export interface WPILibConvertedField {
  tagPositionConfig: WPILibFieldTagMap;
  fieldDimensionsMeters: { length: number; width: number };
}

type WPILibJsonQuaternion = {
  W: number;
  X: number;
  Y: number;
  Z: number;
};

type WPILibJsonTag = {
  ID: number;
  pose: {
    translation: { x: number; y: number; z: number };
    rotation: { quaternion: WPILibJsonQuaternion };
  };
};

type WPILibJsonRoot = {
  tags: WPILibJsonTag[];
  field: { length: number; width: number };
};

function parseWPILibJson(raw: string): WPILibJsonRoot {
  const parsed: unknown = JSON.parse(raw);
  if (typeof parsed !== "object" || parsed === null) {
    throw new Error("WPILib field JSON must be an object");
  }
  const root = parsed as Partial<WPILibJsonRoot>;
  if (!Array.isArray(root.tags)) {
    throw new Error('WPILib field JSON must have a "tags" array');
  }
  if (
    typeof root.field !== "object" ||
    root.field === null ||
    typeof root.field.length !== "number" ||
    typeof root.field.width !== "number"
  ) {
    throw new Error(
      'WPILib field JSON must have a "field" object with numeric length and width',
    );
  }
  return root as WPILibJsonRoot;
}

function tagFromWPILibEntry(entry: WPILibJsonTag): WPILibTagPose {
  const { x, y, z } = entry.pose.translation;
  const q = entry.pose.rotation.quaternion;
  return {
    position: VectorUtil.fromArray([x, y, z]),
    rotation: fromQuaternionNoRoll_ZYX([q.W, q.X, q.Y, q.Z]),
  };
}

export class WPILibConverter {
  /**
   * Reads a WPILib {@link https://github.com/wpilibsuite/allwpilib | AprilTag} field JSON file
   * and builds the same tag map structure used in `pos_extrapolator` tag config (position +
   * rotation matrix per ID). Numbers are taken as-is from the file (no axis flips or
   * re-normalization beyond `fromQuaternionNoRoll_ZYX`).
   */
  static fromWPILibField(jsonPath: string): WPILibConvertedField {
    const raw = fs.readFileSync(jsonPath, { encoding: "utf8" });
    const data = parseWPILibJson(raw);
    const tagPositionConfig: WPILibFieldTagMap = {};
    for (const tag of data.tags) {
      tagPositionConfig[tag.ID] = tagFromWPILibEntry(tag);
    }
    return {
      tagPositionConfig,
      fieldDimensionsMeters: {
        length: data.field.length,
        width: data.field.width,
      },
    };
  }
}
