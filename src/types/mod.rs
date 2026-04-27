//! Value types returned and accepted by the device API.

mod enums;
mod group;
mod model;
mod status;

pub use enums::{BaudRate, ProtectionStatus, RegMode, TempUnit};
pub use group::GroupParams;
pub use model::{Model, ModelCheck};
pub use status::{OnTime, SafetyLimits, Setpoints, Status, Temperatures, Totals};
