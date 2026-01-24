use common_core::proto::util::Vector3;

#[derive(Debug, Clone, Copy)]
pub struct BallFlightTrackerConfig {
    /// Minimum number of points required to consider the ball "detected".
    pub min_points: usize,
    /// Velocity smoothing factor in [0, 1]. Higher = less smoothing.
    pub velocity_alpha: f32,
    /// When centroid.z <= this threshold (meters), treat as ground contact.
    pub ground_z_threshold_m: f32,
    /// Require N consecutive frames below threshold to declare landed.
    pub ground_confirm_frames: u32,
    /// Reject updates with dt outside this range (seconds) as timing glitches.
    pub min_dt_s: f32,
    pub max_dt_s: f32,
}

impl Default for BallFlightTrackerConfig {
    fn default() -> Self {
        Self {
            min_points: 4,
            velocity_alpha: 0.55,
            ground_z_threshold_m: 0.05,
            ground_confirm_frames: 2,
            min_dt_s: 1e-3,
            max_dt_s: 0.5,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct BallSample {
    pub t_s: f64,
    pub position_m: nalgebra::Vector3<f32>,
    pub relative_position_m: nalgebra::Vector3<f32>,
    pub velocity_m_s: nalgebra::Vector3<f32>,
    pub speed_m_s: f32,
    pub point_count: usize,
}

#[derive(Debug, Clone, Copy)]
pub struct FlightSummary {
    pub start_t_s: f64,
    pub end_t_s: f64,
    pub time_of_flight_s: f32,
    pub start_position_m: nalgebra::Vector3<f32>,
    pub end_position_m: nalgebra::Vector3<f32>,
    pub peak_z_m: f32,
    pub max_speed_m_s: f32,
    pub horizontal_distance_m: f32,
    pub sample_count: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UpdateStatus {
    NoDetection,
    InFlight,
    Landed,
}

#[derive(Debug, Clone, Copy)]
struct TrackState {
    last_t_s: f64,
    position_m: nalgebra::Vector3<f32>,
    velocity_m_s: nalgebra::Vector3<f32>,
}

/// Minimal flight tracker:
/// - centroid(pointcloud) = ball position
/// - velocity from consecutive centroids
/// - stop collecting when centroid.z <= threshold for N frames
pub struct BallFlightTracker {
    cfg: BallFlightTrackerConfig,
    origin_m: Option<nalgebra::Vector3<f32>>,
    start_t_s: Option<f64>,
    peak_z_m: f32,
    max_speed_m_s: f32,
    below_ground_frames: u32,
    state: Option<TrackState>,
    history: Vec<BallSample>,
    summary: Option<FlightSummary>,
}

impl BallFlightTracker {
    pub fn new(cfg: BallFlightTrackerConfig) -> Self {
        Self {
            cfg,
            origin_m: None,
            start_t_s: None,
            peak_z_m: f32::NEG_INFINITY,
            max_speed_m_s: 0.0,
            below_ground_frames: 0,
            state: None,
            history: Vec::new(),
            summary: None,
        }
    }

    pub fn history(&self) -> &[BallSample] {
        &self.history
    }

    pub fn summary(&self) -> Option<&FlightSummary> {
        self.summary.as_ref()
    }

    pub fn take_summary(&mut self) -> Option<FlightSummary> {
        self.summary.take()
    }

    pub fn reset(&mut self) {
        self.origin_m = None;
        self.start_t_s = None;
        self.peak_z_m = f32::NEG_INFINITY;
        self.max_speed_m_s = 0.0;
        self.below_ground_frames = 0;
        self.state = None;
        self.history.clear();
        self.summary = None;
    }

    /// Update from a point cloud (meters, world/lidar frame) at time `now_t_s`.
    /// Returns `UpdateStatus::Landed` once landing is detected, then stays landed.
    pub fn update(&mut self, points: &[Vector3], now_t_s: f64) -> UpdateStatus {
        if self.summary.is_some() {
            return UpdateStatus::Landed;
        }

        if points.len() < self.cfg.min_points {
            return UpdateStatus::NoDetection;
        }

        let position_m = centroid(points);

        if self.origin_m.is_none() {
            self.origin_m = Some(position_m);
        }
        if self.start_t_s.is_none() {
            self.start_t_s = Some(now_t_s);
        }
        let origin = self.origin_m.unwrap_or(position_m);
        let relative_position_m = position_m - origin;

        self.peak_z_m = self.peak_z_m.max(position_m.z);

        let velocity_m_s = self.update_velocity(position_m, now_t_s);
        let speed_m_s = velocity_m_s.norm();
        self.max_speed_m_s = self.max_speed_m_s.max(speed_m_s);

        self.history.push(BallSample {
            t_s: now_t_s,
            position_m,
            relative_position_m,
            velocity_m_s,
            speed_m_s,
            point_count: points.len(),
        });

        // Landing detection (threshold + confirm frames)
        if position_m.z <= self.cfg.ground_z_threshold_m && self.history.len() > 1 {
            self.below_ground_frames += 1;
        } else {
            self.below_ground_frames = 0;
        }

        if self.below_ground_frames >= self.cfg.ground_confirm_frames {
            self.finalize_summary(now_t_s, position_m);
            return UpdateStatus::Landed;
        }

        UpdateStatus::InFlight
    }

    fn update_velocity(
        &mut self,
        pos_m: nalgebra::Vector3<f32>,
        now_t_s: f64,
    ) -> nalgebra::Vector3<f32> {
        match self.state {
            None => {
                self.state = Some(TrackState {
                    last_t_s: now_t_s,
                    position_m: pos_m,
                    velocity_m_s: nalgebra::Vector3::zeros(),
                });
                nalgebra::Vector3::zeros()
            }
            Some(mut s) => {
                let dt = (now_t_s - s.last_t_s).max(0.0) as f32;
                if dt < self.cfg.min_dt_s || dt > self.cfg.max_dt_s {
                    // Timing glitch: re-seed without producing a crazy velocity.
                    s.last_t_s = now_t_s;
                    s.position_m = pos_m;
                    s.velocity_m_s = nalgebra::Vector3::zeros();
                    self.state = Some(s);
                    return nalgebra::Vector3::zeros();
                }

                let raw_v = (pos_m - s.position_m) / dt;
                let alpha = self.cfg.velocity_alpha.clamp(0.0, 1.0);
                let v = s.velocity_m_s * (1.0 - alpha) + raw_v * alpha;

                s.last_t_s = now_t_s;
                s.position_m = pos_m;
                s.velocity_m_s = v;
                self.state = Some(s);
                v
            }
        }
    }

    fn finalize_summary(&mut self, end_t_s: f64, end_position_m: nalgebra::Vector3<f32>) {
        let Some(start_t_s) = self.start_t_s else {
            return;
        };
        let Some(start_position_m) = self.origin_m else {
            return;
        };

        let tof = (end_t_s - start_t_s).max(0.0) as f32;
        let horizontal_distance_m = nalgebra::Vector2::new(
            end_position_m.x - start_position_m.x,
            end_position_m.y - start_position_m.y,
        )
        .norm();

        self.summary = Some(FlightSummary {
            start_t_s,
            end_t_s,
            time_of_flight_s: tof,
            start_position_m,
            end_position_m,
            peak_z_m: self.peak_z_m,
            max_speed_m_s: self.max_speed_m_s,
            horizontal_distance_m,
            sample_count: self.history.len(),
        });
    }
}

fn centroid(points: &[Vector3]) -> nalgebra::Vector3<f32> {
    let mut sum = nalgebra::Vector3::<f32>::zeros();
    for p in points {
        sum += nalgebra::Vector3::new(p.x, p.y, p.z);
    }
    sum / (points.len() as f32)
}
