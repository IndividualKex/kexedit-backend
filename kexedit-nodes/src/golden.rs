use kexedit_core::{Float3, InterpolationType, Point};
use serde::{Deserialize, Serialize};

use crate::DurationType;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct GoldTrackData {
    pub metadata: GoldMetadata,
    pub graph: GoldGraph,
    pub sections: Vec<GoldSection>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct GoldMetadata {
    pub source_file: String,
    pub exported_at: String,
    pub kex_edit_version: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct GoldGraph {
    pub root_node_id: u32,
    pub node_order: Vec<u32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct GoldSection {
    pub node_id: u32,
    pub node_type: String,
    pub position: GoldVec2,
    pub inputs: GoldInputs,
    pub outputs: GoldOutputs,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct GoldInputs {
    pub anchor: GoldPointData,
    #[serde(default)]
    pub duration: Option<GoldDuration>,
    #[serde(default)]
    pub property_overrides: Option<GoldPropertyOverrides>,
    #[serde(default)]
    pub steering: bool,
    #[serde(default)]
    pub curve_data: Option<GoldCurveData>,
    #[serde(default)]
    pub keyframes: Option<GoldKeyframes>,
    #[serde(default)]
    pub source_path: Vec<GoldPointData>,
    #[serde(default)]
    pub start: f32,
    #[serde(default)]
    pub end: f32,
    #[serde(default)]
    pub target_anchor: Option<GoldPointData>,
    #[serde(default)]
    pub out_weight: f32,
    #[serde(default)]
    pub in_weight: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct GoldCurveData {
    pub radius: f32,
    pub arc: f32,
    pub axis: f32,
    pub lead_in: f32,
    pub lead_out: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct GoldOutputs {
    pub point_count: i32,
    pub total_length: f32,
    pub points: Vec<GoldPointData>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct GoldDuration {
    #[serde(rename = "type")]
    pub duration_type: String,
    pub value: f32,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct GoldPropertyOverrides {
    #[serde(default)]
    pub fixed_velocity: bool,
    #[serde(default)]
    pub heart: bool,
    #[serde(default)]
    pub friction: bool,
    #[serde(default)]
    pub resistance: bool,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct GoldKeyframes {
    #[serde(default)]
    pub roll_speed: Vec<GoldKeyframe>,
    #[serde(default)]
    pub normal_force: Vec<GoldKeyframe>,
    #[serde(default)]
    pub lateral_force: Vec<GoldKeyframe>,
    #[serde(default)]
    pub pitch_speed: Vec<GoldKeyframe>,
    #[serde(default)]
    pub yaw_speed: Vec<GoldKeyframe>,
    #[serde(default)]
    pub fixed_velocity: Vec<GoldKeyframe>,
    #[serde(default)]
    pub heart: Vec<GoldKeyframe>,
    #[serde(default)]
    pub friction: Vec<GoldKeyframe>,
    #[serde(default)]
    pub resistance: Vec<GoldKeyframe>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct GoldKeyframe {
    pub id: u32,
    pub time: f32,
    pub value: f32,
    pub in_interpolation: String,
    pub out_interpolation: String,
    pub handle_type: String,
    pub in_tangent: f32,
    pub out_tangent: f32,
    pub in_weight: f32,
    pub out_weight: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct GoldPointData {
    pub position: GoldVec3,
    pub direction: GoldVec3,
    pub lateral: GoldVec3,
    pub normal: GoldVec3,
    pub roll: f32,
    pub velocity: f32,
    pub energy: f32,
    pub normal_force: f32,
    pub lateral_force: f32,
    pub distance_from_last: f32,
    pub heart_distance_from_last: f32,
    pub angle_from_last: f32,
    pub pitch_from_last: f32,
    pub yaw_from_last: f32,
    pub roll_speed: f32,
    pub total_length: f32,
    pub total_heart_length: f32,
    pub friction_compensation: f32,
    pub heart: f32,
    pub friction: f32,
    pub resistance: f32,
    pub facing: i32,
    #[serde(default)]
    pub effective_friction_distance: f32,
    #[serde(default)]
    pub kinetic_energy: f32,
    #[serde(default)]
    pub gravitational_pe: f32,
    #[serde(default)]
    pub friction_pe: f32,
    #[serde(default)]
    pub center_y: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoldVec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoldVec2 {
    pub x: f32,
    pub y: f32,
}

impl GoldTrackData {
    pub fn load(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let contents = std::fs::read_to_string(path)?;
        let contents = contents.trim_start_matches('\u{FEFF}');
        let data = serde_json::from_str(contents)?;
        Ok(data)
    }

    pub fn get_force_sections(&self) -> Vec<&GoldSection> {
        self.sections
            .iter()
            .filter(|s| s.node_type == "ForceSection")
            .collect()
    }

    pub fn get_geometric_sections(&self) -> Vec<&GoldSection> {
        self.sections
            .iter()
            .filter(|s| s.node_type == "GeometricSection")
            .collect()
    }

    pub fn get_curved_sections(&self) -> Vec<&GoldSection> {
        self.sections
            .iter()
            .filter(|s| s.node_type == "CurvedSection")
            .collect()
    }

    pub fn get_copy_path_sections(&self) -> Vec<&GoldSection> {
        self.sections
            .iter()
            .filter(|s| s.node_type == "CopyPathSection")
            .collect()
    }

    pub fn get_bridge_sections(&self) -> Vec<&GoldSection> {
        self.sections
            .iter()
            .filter(|s| s.node_type == "Bridge")
            .collect()
    }
}

impl GoldPointData {
    pub fn to_point(&self) -> Point {
        Point::new(
            Float3::new(self.position.x, self.position.y, self.position.z),
            Float3::new(self.direction.x, self.direction.y, self.direction.z),
            Float3::new(self.normal.x, self.normal.y, self.normal.z),
            Float3::new(self.lateral.x, self.lateral.y, self.lateral.z),
            self.velocity,
            self.energy,
            self.normal_force,
            self.lateral_force,
            self.total_length,
            self.total_heart_length,
            self.heart_distance_from_last,
            self.friction_compensation,
            self.roll_speed,
            self.heart,
            self.friction,
            self.resistance,
        )
    }
}

impl GoldKeyframe {
    pub fn to_keyframe(&self) -> kexedit_core::Keyframe {
        kexedit_core::Keyframe::new(
            self.time,
            self.value,
            parse_interpolation_type(&self.in_interpolation),
            parse_interpolation_type(&self.out_interpolation),
            self.in_tangent,
            self.out_tangent,
            self.in_weight,
            self.out_weight,
        )
    }
}

impl GoldDuration {
    pub fn to_duration_type(&self) -> DurationType {
        match self.duration_type.as_str() {
            "Time" => DurationType::Time,
            "Distance" => DurationType::Distance,
            _ => DurationType::Time,
        }
    }
}

fn parse_interpolation_type(s: &str) -> InterpolationType {
    match s {
        "Constant" => InterpolationType::Constant,
        "Linear" => InterpolationType::Linear,
        "Bezier" => InterpolationType::Bezier,
        _ => InterpolationType::Bezier,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_load_shuttle_json() {
        let path = "../../Assets/Tests/TrackData/shuttle.json";
        let result = GoldTrackData::load(path);
        assert!(result.is_ok(), "Failed to load shuttle.json: {:?}", result.err());

        let data = result.unwrap();
        assert_eq!(data.metadata.source_file, "shuttle");
        assert!(!data.sections.is_empty());
    }

    #[test]
    fn test_load_veloci_json() {
        let path = "../../Assets/Tests/TrackData/veloci.json";
        let result = GoldTrackData::load(path);
        assert!(result.is_ok(), "Failed to load veloci.json: {:?}", result.err());

        let data = result.unwrap();
        assert_eq!(data.metadata.source_file, "veloci");
        assert!(!data.sections.is_empty());
    }

    #[test]
    fn test_filter_force_sections() {
        let path = "../../Assets/Tests/TrackData/shuttle.json";
        let data = GoldTrackData::load(path).unwrap();
        let force_sections = data.get_force_sections();
        assert!(!force_sections.is_empty());
    }

    #[test]
    fn test_filter_geometric_sections() {
        let path = "../../Assets/Tests/TrackData/shuttle.json";
        let data = GoldTrackData::load(path).unwrap();
        let geometric_sections = data.get_geometric_sections();
        assert!(!geometric_sections.is_empty());
    }
}
