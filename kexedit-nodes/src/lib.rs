#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PortId {
    Anchor = 0,
    Path = 1,
    Duration = 2,
    Radius = 3,
    Arc = 4,
    Axis = 5,
    LeadIn = 6,
    LeadOut = 7,
    InWeight = 8,
    OutWeight = 9,
    Start = 10,
    End = 11,
    Position = 12,
    Rotation = 13,
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PropertyId {
    RollSpeed = 0,
    NormalForce = 1,
    LateralForce = 2,
    PitchSpeed = 3,
    YawSpeed = 4,
    DrivenVelocity = 5,
    HeartOffset = 6,
    Friction = 7,
    Resistance = 8,
    TrackStyle = 9,
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum NodeType {
    Force = 0,
    Geometric = 1,
    Curved = 2,
    CopyPath = 3,
    Bridge = 4,
    Anchor = 5,
    Reverse = 6,
    ReversePath = 7,
}

impl NodeType {
    const COUNT: usize = 8;

    const fn as_index(self) -> usize {
        self as usize
    }
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum DurationType {
    Time = 0,
    Distance = 1,
}

#[derive(Debug, Copy, Clone)]
pub struct IterationConfig {
    pub duration: f32,
    pub duration_type: DurationType,
}

impl IterationConfig {
    pub const fn new(duration: f32, duration_type: DurationType) -> Self {
        Self {
            duration,
            duration_type,
        }
    }
}

const INVALID_PORT: u8 = 255;
const INVALID_PROPERTY: u8 = 255;

const INPUT_COUNTS: [usize; NodeType::COUNT] = [2, 2, 6, 4, 3, 2, 1, 1];

const INPUT_PORTS: [[u8; 6]; NodeType::COUNT] = [
    [PortId::Anchor as u8, PortId::Duration as u8, INVALID_PORT, INVALID_PORT, INVALID_PORT, INVALID_PORT], // Force
    [PortId::Anchor as u8, PortId::Duration as u8, INVALID_PORT, INVALID_PORT, INVALID_PORT, INVALID_PORT], // Geometric
    [PortId::Anchor as u8, PortId::Radius as u8, PortId::Arc as u8, PortId::Axis as u8, PortId::LeadIn as u8, PortId::LeadOut as u8], // Curved
    [PortId::Anchor as u8, PortId::Path as u8, PortId::Start as u8, PortId::End as u8, INVALID_PORT, INVALID_PORT], // CopyPath
    [PortId::Anchor as u8, PortId::InWeight as u8, PortId::OutWeight as u8, INVALID_PORT, INVALID_PORT, INVALID_PORT], // Bridge
    [PortId::Position as u8, PortId::Rotation as u8, INVALID_PORT, INVALID_PORT, INVALID_PORT, INVALID_PORT], // Anchor
    [PortId::Anchor as u8, INVALID_PORT, INVALID_PORT, INVALID_PORT, INVALID_PORT, INVALID_PORT], // Reverse
    [PortId::Path as u8, INVALID_PORT, INVALID_PORT, INVALID_PORT, INVALID_PORT, INVALID_PORT], // ReversePath
];

const OUTPUT_COUNTS: [usize; NodeType::COUNT] = [2, 2, 2, 2, 2, 1, 1, 1];

const OUTPUT_PORTS: [[u8; 2]; NodeType::COUNT] = [
    [PortId::Anchor as u8, PortId::Path as u8], // Force
    [PortId::Anchor as u8, PortId::Path as u8], // Geometric
    [PortId::Anchor as u8, PortId::Path as u8], // Curved
    [PortId::Anchor as u8, PortId::Path as u8], // CopyPath
    [PortId::Anchor as u8, PortId::Path as u8], // Bridge
    [PortId::Anchor as u8, INVALID_PORT],        // Anchor
    [PortId::Anchor as u8, INVALID_PORT],        // Reverse
    [PortId::Path as u8, INVALID_PORT],          // ReversePath
];

const PROPERTY_COUNTS: [usize; NodeType::COUNT] = [7, 7, 5, 4, 5, 0, 0, 0];

const PROPERTIES: [[u8; 7]; NodeType::COUNT] = [
    [PropertyId::RollSpeed as u8, PropertyId::NormalForce as u8, PropertyId::LateralForce as u8, PropertyId::DrivenVelocity as u8, PropertyId::HeartOffset as u8, PropertyId::Friction as u8, PropertyId::Resistance as u8], // Force
    [PropertyId::RollSpeed as u8, PropertyId::PitchSpeed as u8, PropertyId::YawSpeed as u8, PropertyId::DrivenVelocity as u8, PropertyId::HeartOffset as u8, PropertyId::Friction as u8, PropertyId::Resistance as u8], // Geometric
    [PropertyId::RollSpeed as u8, PropertyId::DrivenVelocity as u8, PropertyId::HeartOffset as u8, PropertyId::Friction as u8, PropertyId::Resistance as u8, INVALID_PROPERTY, INVALID_PROPERTY], // Curved
    [PropertyId::DrivenVelocity as u8, PropertyId::HeartOffset as u8, PropertyId::Friction as u8, PropertyId::Resistance as u8, INVALID_PROPERTY, INVALID_PROPERTY, INVALID_PROPERTY], // CopyPath
    [PropertyId::DrivenVelocity as u8, PropertyId::HeartOffset as u8, PropertyId::Friction as u8, PropertyId::Resistance as u8, PropertyId::TrackStyle as u8, INVALID_PROPERTY, INVALID_PROPERTY], // Bridge
    [INVALID_PROPERTY, INVALID_PROPERTY, INVALID_PROPERTY, INVALID_PROPERTY, INVALID_PROPERTY, INVALID_PROPERTY, INVALID_PROPERTY], // Anchor
    [INVALID_PROPERTY, INVALID_PROPERTY, INVALID_PROPERTY, INVALID_PROPERTY, INVALID_PROPERTY, INVALID_PROPERTY, INVALID_PROPERTY], // Reverse
    [INVALID_PROPERTY, INVALID_PROPERTY, INVALID_PROPERTY, INVALID_PROPERTY, INVALID_PROPERTY, INVALID_PROPERTY, INVALID_PROPERTY], // ReversePath
];

pub struct NodeSchema;

impl NodeSchema {
    pub const fn input_count(node_type: NodeType) -> usize {
        INPUT_COUNTS[node_type.as_index()]
    }

    pub const fn input(node_type: NodeType, index: usize) -> Option<PortId> {
        if index >= 6 {
            return None;
        }
        let port_u8 = INPUT_PORTS[node_type.as_index()][index];
        if port_u8 == INVALID_PORT {
            None
        } else {
            Some(unsafe { core::mem::transmute(port_u8) })
        }
    }

    pub const fn output_count(node_type: NodeType) -> usize {
        OUTPUT_COUNTS[node_type.as_index()]
    }

    pub const fn output(node_type: NodeType, index: usize) -> Option<PortId> {
        if index >= 2 {
            return None;
        }
        let port_u8 = OUTPUT_PORTS[node_type.as_index()][index];
        if port_u8 == INVALID_PORT {
            None
        } else {
            Some(unsafe { core::mem::transmute(port_u8) })
        }
    }

    pub const fn property_count(node_type: NodeType) -> usize {
        PROPERTY_COUNTS[node_type.as_index()]
    }

    pub const fn property(node_type: NodeType, index: usize) -> Option<PropertyId> {
        if index >= 7 {
            return None;
        }
        let property_u8 = PROPERTIES[node_type.as_index()][index];
        if property_u8 == INVALID_PROPERTY {
            None
        } else {
            Some(unsafe { core::mem::transmute(property_u8) })
        }
    }
}

pub struct PropertyIndex;

impl PropertyIndex {
    pub const fn to_index(property: PropertyId, node: NodeType) -> i32 {
        let count = NodeSchema::property_count(node);
        let mut i = 0;
        while i < count {
            if let Some(prop) = NodeSchema::property(node, i) {
                if prop as u8 == property as u8 {
                    return i as i32;
                }
            }
            i += 1;
        }
        -1
    }

    pub const fn from_index(index: i32, node: NodeType) -> Option<PropertyId> {
        if index < 0 || index as usize >= NodeSchema::property_count(node) {
            None
        } else {
            NodeSchema::property(node, index as usize)
        }
    }
}

pub mod force;
pub mod geometric;
pub mod anchor;
pub mod reverse;
pub mod reverse_path;
pub mod curved;
pub mod bridge;
pub mod copy_path;

#[cfg(test)]
mod golden;

#[cfg(test)]
mod point_comparer;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn property_index_roundtrip() {
        let test_cases = vec![
            (PropertyId::RollSpeed, NodeType::Force, 0),
            (PropertyId::NormalForce, NodeType::Force, 1),
            (PropertyId::LateralForce, NodeType::Force, 2),
            (PropertyId::PitchSpeed, NodeType::Geometric, 1),
            (PropertyId::YawSpeed, NodeType::Geometric, 2),
            (PropertyId::DrivenVelocity, NodeType::Curved, 1),
            (PropertyId::TrackStyle, NodeType::Bridge, 4),
        ];

        for (property, node, expected_index) in test_cases {
            let index = PropertyIndex::to_index(property, node);
            assert_eq!(index, expected_index, "to_index failed for {:?}, {:?}", property, node);

            let recovered = PropertyIndex::from_index(index, node);
            assert_eq!(recovered, Some(property), "from_index failed for index {}, {:?}", index, node);
        }
    }

    #[test]
    fn property_index_invalid() {
        assert_eq!(PropertyIndex::to_index(PropertyId::NormalForce, NodeType::Geometric), -1);
        assert_eq!(PropertyIndex::to_index(PropertyId::TrackStyle, NodeType::Force), -1);
        assert_eq!(PropertyIndex::from_index(99, NodeType::Force), None);
        assert_eq!(PropertyIndex::from_index(-1, NodeType::Force), None);
    }

    #[test]
    fn node_schema_input_counts() {
        assert_eq!(NodeSchema::input_count(NodeType::Force), 2);
        assert_eq!(NodeSchema::input_count(NodeType::Geometric), 2);
        assert_eq!(NodeSchema::input_count(NodeType::Curved), 6);
        assert_eq!(NodeSchema::input_count(NodeType::CopyPath), 4);
        assert_eq!(NodeSchema::input_count(NodeType::Bridge), 3);
        assert_eq!(NodeSchema::input_count(NodeType::Anchor), 2);
        assert_eq!(NodeSchema::input_count(NodeType::Reverse), 1);
        assert_eq!(NodeSchema::input_count(NodeType::ReversePath), 1);
    }

    #[test]
    fn node_schema_output_counts() {
        assert_eq!(NodeSchema::output_count(NodeType::Force), 2);
        assert_eq!(NodeSchema::output_count(NodeType::Geometric), 2);
        assert_eq!(NodeSchema::output_count(NodeType::Curved), 2);
        assert_eq!(NodeSchema::output_count(NodeType::CopyPath), 2);
        assert_eq!(NodeSchema::output_count(NodeType::Bridge), 2);
        assert_eq!(NodeSchema::output_count(NodeType::Anchor), 1);
        assert_eq!(NodeSchema::output_count(NodeType::Reverse), 1);
        assert_eq!(NodeSchema::output_count(NodeType::ReversePath), 1);
    }

    #[test]
    fn node_schema_property_counts() {
        assert_eq!(NodeSchema::property_count(NodeType::Force), 7);
        assert_eq!(NodeSchema::property_count(NodeType::Geometric), 7);
        assert_eq!(NodeSchema::property_count(NodeType::Curved), 5);
        assert_eq!(NodeSchema::property_count(NodeType::CopyPath), 4);
        assert_eq!(NodeSchema::property_count(NodeType::Bridge), 5);
        assert_eq!(NodeSchema::property_count(NodeType::Anchor), 0);
        assert_eq!(NodeSchema::property_count(NodeType::Reverse), 0);
        assert_eq!(NodeSchema::property_count(NodeType::ReversePath), 0);
    }

    #[test]
    fn node_schema_force_inputs() {
        assert_eq!(NodeSchema::input(NodeType::Force, 0), Some(PortId::Anchor));
        assert_eq!(NodeSchema::input(NodeType::Force, 1), Some(PortId::Duration));
        assert_eq!(NodeSchema::input(NodeType::Force, 2), None);
    }

    #[test]
    fn node_schema_curved_inputs() {
        assert_eq!(NodeSchema::input(NodeType::Curved, 0), Some(PortId::Anchor));
        assert_eq!(NodeSchema::input(NodeType::Curved, 1), Some(PortId::Radius));
        assert_eq!(NodeSchema::input(NodeType::Curved, 2), Some(PortId::Arc));
        assert_eq!(NodeSchema::input(NodeType::Curved, 3), Some(PortId::Axis));
        assert_eq!(NodeSchema::input(NodeType::Curved, 4), Some(PortId::LeadIn));
        assert_eq!(NodeSchema::input(NodeType::Curved, 5), Some(PortId::LeadOut));
        assert_eq!(NodeSchema::input(NodeType::Curved, 6), None);
    }

    #[test]
    fn node_schema_outputs() {
        assert_eq!(NodeSchema::output(NodeType::Force, 0), Some(PortId::Anchor));
        assert_eq!(NodeSchema::output(NodeType::Force, 1), Some(PortId::Path));
        assert_eq!(NodeSchema::output(NodeType::Anchor, 0), Some(PortId::Anchor));
        assert_eq!(NodeSchema::output(NodeType::ReversePath, 0), Some(PortId::Path));
    }

    #[test]
    fn node_schema_force_properties() {
        assert_eq!(NodeSchema::property(NodeType::Force, 0), Some(PropertyId::RollSpeed));
        assert_eq!(NodeSchema::property(NodeType::Force, 1), Some(PropertyId::NormalForce));
        assert_eq!(NodeSchema::property(NodeType::Force, 2), Some(PropertyId::LateralForce));
        assert_eq!(NodeSchema::property(NodeType::Force, 3), Some(PropertyId::DrivenVelocity));
        assert_eq!(NodeSchema::property(NodeType::Force, 4), Some(PropertyId::HeartOffset));
        assert_eq!(NodeSchema::property(NodeType::Force, 5), Some(PropertyId::Friction));
        assert_eq!(NodeSchema::property(NodeType::Force, 6), Some(PropertyId::Resistance));
        assert_eq!(NodeSchema::property(NodeType::Force, 7), None);
    }

    #[test]
    fn node_schema_geometric_properties() {
        assert_eq!(NodeSchema::property(NodeType::Geometric, 0), Some(PropertyId::RollSpeed));
        assert_eq!(NodeSchema::property(NodeType::Geometric, 1), Some(PropertyId::PitchSpeed));
        assert_eq!(NodeSchema::property(NodeType::Geometric, 2), Some(PropertyId::YawSpeed));
    }

    #[test]
    fn node_schema_bridge_properties() {
        assert_eq!(NodeSchema::property(NodeType::Bridge, 4), Some(PropertyId::TrackStyle));
    }

    #[test]
    fn iteration_config_creation() {
        let config = IterationConfig::new(5.0, DurationType::Time);
        assert_eq!(config.duration, 5.0);
        assert_eq!(config.duration_type, DurationType::Time);

        let config2 = IterationConfig::new(100.0, DurationType::Distance);
        assert_eq!(config2.duration, 100.0);
        assert_eq!(config2.duration_type, DurationType::Distance);
    }

    #[test]
    fn all_node_types_complete() {
        for node_type in [
            NodeType::Force,
            NodeType::Geometric,
            NodeType::Curved,
            NodeType::CopyPath,
            NodeType::Bridge,
            NodeType::Anchor,
            NodeType::Reverse,
            NodeType::ReversePath,
        ] {
            let input_count = NodeSchema::input_count(node_type);
            for i in 0..input_count {
                assert!(
                    NodeSchema::input(node_type, i).is_some(),
                    "Missing input {} for {:?}",
                    i,
                    node_type
                );
            }
            assert!(NodeSchema::input(node_type, input_count).is_none());

            let output_count = NodeSchema::output_count(node_type);
            for i in 0..output_count {
                assert!(
                    NodeSchema::output(node_type, i).is_some(),
                    "Missing output {} for {:?}",
                    i,
                    node_type
                );
            }
            assert!(NodeSchema::output(node_type, output_count).is_none());

            let property_count = NodeSchema::property_count(node_type);
            for i in 0..property_count {
                assert!(
                    NodeSchema::property(node_type, i).is_some(),
                    "Missing property {} for {:?}",
                    i,
                    node_type
                );
            }
            assert!(NodeSchema::property(node_type, property_count).is_none());
        }
    }

    #[test]
    fn all_properties_have_index() {
        let test_matrix = vec![
            (NodeType::Force, vec![
                PropertyId::RollSpeed,
                PropertyId::NormalForce,
                PropertyId::LateralForce,
                PropertyId::DrivenVelocity,
                PropertyId::HeartOffset,
                PropertyId::Friction,
                PropertyId::Resistance,
            ]),
            (NodeType::Geometric, vec![
                PropertyId::RollSpeed,
                PropertyId::PitchSpeed,
                PropertyId::YawSpeed,
                PropertyId::DrivenVelocity,
                PropertyId::HeartOffset,
                PropertyId::Friction,
                PropertyId::Resistance,
            ]),
            (NodeType::Curved, vec![
                PropertyId::RollSpeed,
                PropertyId::DrivenVelocity,
                PropertyId::HeartOffset,
                PropertyId::Friction,
                PropertyId::Resistance,
            ]),
            (NodeType::CopyPath, vec![
                PropertyId::DrivenVelocity,
                PropertyId::HeartOffset,
                PropertyId::Friction,
                PropertyId::Resistance,
            ]),
            (NodeType::Bridge, vec![
                PropertyId::DrivenVelocity,
                PropertyId::HeartOffset,
                PropertyId::Friction,
                PropertyId::Resistance,
                PropertyId::TrackStyle,
            ]),
        ];

        for (node_type, properties) in test_matrix {
            for property in properties {
                let index = PropertyIndex::to_index(property, node_type);
                assert!(
                    index >= 0,
                    "{:?} should have valid index for {:?}",
                    property,
                    node_type
                );
                let recovered = PropertyIndex::from_index(index, node_type);
                assert_eq!(
                    recovered,
                    Some(property),
                    "Roundtrip failed for {:?} on {:?}",
                    property,
                    node_type
                );
            }
        }
    }

    #[test]
    fn const_evaluation_works() {
        const INPUT_COUNT: usize = NodeSchema::input_count(NodeType::Force);
        assert_eq!(INPUT_COUNT, 2);

        const PROPERTY_COUNT: usize = NodeSchema::property_count(NodeType::Geometric);
        assert_eq!(PROPERTY_COUNT, 7);
    }
}
