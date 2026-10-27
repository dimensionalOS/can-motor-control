//! Per-motor-type physical limit table.

use motor_codec::Limits;

use crate::types::DamiaoMotorType;

/// Damiao motor limits as published in the protocol manual.
///
/// Used to scale MIT-mode setpoints into bit-packed integers and to unscale
/// state response integers back into f64.
pub(crate) const fn limits_for(t: DamiaoMotorType) -> Limits {
    match t {
        DamiaoMotorType::DM3507 => Limits {
            p_max: 12.5,
            v_max: 50.0,
            t_max: 5.0,
        },
        DamiaoMotorType::DM4310 => Limits {
            p_max: 12.5,
            v_max: 30.0,
            t_max: 10.0,
        },
        DamiaoMotorType::DM4310_48V => Limits {
            p_max: 12.5,
            v_max: 50.0,
            t_max: 10.0,
        },
        DamiaoMotorType::DM4340 => Limits {
            p_max: 12.5,
            v_max: 10.0,
            t_max: 28.0,
        },
        DamiaoMotorType::DM4340_48V => Limits {
            p_max: 12.5,
            v_max: 10.0,
            t_max: 28.0,
        },
        DamiaoMotorType::DM6006 => Limits {
            p_max: 12.5,
            v_max: 45.0,
            t_max: 20.0,
        },
        DamiaoMotorType::DM8006 => Limits {
            p_max: 12.5,
            v_max: 45.0,
            t_max: 40.0,
        },
        DamiaoMotorType::DM8009 => Limits {
            p_max: 12.5,
            v_max: 45.0,
            t_max: 54.0,
        },
        DamiaoMotorType::DM10010L => Limits {
            p_max: 12.5,
            v_max: 25.0,
            t_max: 200.0,
        },
        DamiaoMotorType::DM10010 => Limits {
            p_max: 12.5,
            v_max: 20.0,
            t_max: 200.0,
        },
        DamiaoMotorType::DMH3510 => Limits {
            p_max: 12.5,
            v_max: 280.0,
            t_max: 1.0,
        },
        DamiaoMotorType::DMH6215 => Limits {
            p_max: 12.5,
            v_max: 45.0,
            t_max: 10.0,
        },
        DamiaoMotorType::DMG6220 => Limits {
            p_max: 12.5,
            v_max: 45.0,
            t_max: 10.0,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dm4340_known_anchor() {
        let l = limits_for(DamiaoMotorType::DM4340);
        assert_eq!(l.p_max, 12.5);
        assert_eq!(l.v_max, 10.0);
        assert_eq!(l.t_max, 28.0);
    }

    #[test]
    fn every_sku_positive_limits() {
        for disc in 0..=12u16 {
            let t = DamiaoMotorType::from_discriminant(disc).unwrap();
            let l = limits_for(t);
            assert!(l.p_max > 0.0 && l.v_max > 0.0 && l.t_max > 0.0, "{t:?}");
        }
    }
}
