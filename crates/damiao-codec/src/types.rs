//! Damiao motor type and register-ID enums.

use motor_codec::MotorTypeId;

/// Damiao motor SKUs supported by v1 of this codec.
///
/// Each variant maps to a unique `u16` discriminant carried by
/// [`MotorTypeId::Damiao`]. Discriminants are intentionally explicit so they
/// remain stable across refactors.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[repr(u16)]
#[non_exhaustive]
pub enum DamiaoMotorType {
    /// DM-3507 — small ankle/wrist actuator.
    DM3507 = 0,
    /// DM-4310 24V.
    DM4310 = 1,
    /// DM-4310 48V.
    DM4310_48V = 2,
    /// DM-4340 24V, also the DM-4340P protocol profile.
    DM4340 = 3,
    /// DM-4340 48V.
    DM4340_48V = 4,
    /// DM-6006.
    DM6006 = 5,
    /// DM-8006.
    DM8006 = 6,
    /// DM-8009.
    DM8009 = 7,
    /// DM-10010L (low-speed, high-torque hip).
    DM10010L = 8,
    /// DM-10010 (high-speed hip).
    DM10010 = 9,
    /// DM-H3510 (high-speed micro).
    DMH3510 = 10,
    /// DM-H6215.
    DMH6215 = 11,
    /// DM-G6220 (gearbox geared).
    DMG6220 = 12,
}

impl DamiaoMotorType {
    /// Inverse of `From<u16>`; returns `None` for unknown discriminants.
    pub fn from_discriminant(d: u16) -> Option<Self> {
        Some(match d {
            0 => Self::DM3507,
            1 => Self::DM4310,
            2 => Self::DM4310_48V,
            3 => Self::DM4340,
            4 => Self::DM4340_48V,
            5 => Self::DM6006,
            6 => Self::DM8006,
            7 => Self::DM8009,
            8 => Self::DM10010L,
            9 => Self::DM10010,
            10 => Self::DMH3510,
            11 => Self::DMH6215,
            12 => Self::DMG6220,
            _ => return None,
        })
    }
}

impl From<DamiaoMotorType> for MotorTypeId {
    fn from(t: DamiaoMotorType) -> Self {
        MotorTypeId::Damiao(t as u16)
    }
}

/// Look up a Damiao SKU by string (used by the TOML loader to resolve
/// `type = "DM4340"` config fields).
///
/// Returns `None` for unknown SKU strings.
pub fn parse_motor_type(s: &str) -> Option<MotorTypeId> {
    let t = match s {
        "DM3507" => DamiaoMotorType::DM3507,
        "DM4310" => DamiaoMotorType::DM4310,
        "DM4310_48V" => DamiaoMotorType::DM4310_48V,
        "DM4340" => DamiaoMotorType::DM4340,
        "DM4340_48V" => DamiaoMotorType::DM4340_48V,
        "DM6006" => DamiaoMotorType::DM6006,
        "DM8006" => DamiaoMotorType::DM8006,
        "DM8009" => DamiaoMotorType::DM8009,
        "DM10010L" => DamiaoMotorType::DM10010L,
        "DM10010" => DamiaoMotorType::DM10010,
        "DMH3510" => DamiaoMotorType::DMH3510,
        "DMH6215" => DamiaoMotorType::DMH6215,
        "DMG6220" => DamiaoMotorType::DMG6220,
        _ => return None,
    };
    Some(t.into())
}

/// Register IDs for the Damiao parameter sub-protocol (CAN ID `0x7FF`).
///
/// Byte values match Damiao's protocol manual.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
#[non_exhaustive]
#[allow(non_camel_case_types, missing_docs)]
pub enum DamiaoRid {
    UV_Value = 0,
    KT_Value = 1,
    OT_Value = 2,
    OC_Value = 3,
    ACC = 4,
    DEC = 5,
    MAX_SPD = 6,
    MST_ID = 7,
    ESC_ID = 8,
    TIMEOUT = 9,
    CTRL_MODE = 10,
    Damp = 11,
    Inertia = 12,
    hw_ver = 13,
    sw_ver = 14,
    SN = 15,
    NPP = 16,
    Rs = 17,
    LS = 18,
    Flux = 19,
    Gr = 20,
    PMAX = 21,
    VMAX = 22,
    TMAX = 23,
    I_BW = 24,
    KP_ASR = 25,
    KI_ASR = 26,
    KP_APR = 27,
    KI_APR = 28,
    OV_Value = 29,
    GREF = 30,
    Deta = 31,
    V_BW = 32,
    IQ_c1 = 33,
    VL_c1 = 34,
    can_br = 35,
    sub_ver = 36,
}

impl From<DamiaoRid> for u8 {
    fn from(r: DamiaoRid) -> u8 {
        r as u8
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL_SKUS: &[(&str, u16)] = &[
        ("DM3507", 0),
        ("DM4310", 1),
        ("DM4310_48V", 2),
        ("DM4340", 3),
        ("DM4340_48V", 4),
        ("DM6006", 5),
        ("DM8006", 6),
        ("DM8009", 7),
        ("DM10010L", 8),
        ("DM10010", 9),
        ("DMH3510", 10),
        ("DMH6215", 11),
        ("DMG6220", 12),
    ];

    #[test]
    fn all_13_skus_parse() {
        for (name, disc) in ALL_SKUS {
            let id = parse_motor_type(name).unwrap_or_else(|| panic!("parse {name}"));
            assert_eq!(id, MotorTypeId::Damiao(*disc), "SKU {name}");
        }
    }

    #[test]
    fn unknown_string_returns_none() {
        assert!(parse_motor_type("DM_DOES_NOT_EXIST").is_none());
    }

    #[test]
    fn rid_to_u8_known() {
        assert_eq!(u8::from(DamiaoRid::CTRL_MODE), 10);
        assert_eq!(u8::from(DamiaoRid::MST_ID), 7);
        assert_eq!(u8::from(DamiaoRid::PMAX), 21);
    }
}
