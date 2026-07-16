pub mod barrier;
pub mod emphasis;
pub mod mip;
pub mod preprocessing;
pub mod read;
pub mod tolerances;

use std::{
    ffi::{c_double, c_int},
    time::Duration,
};

use ffi::{
    CPXPARAM_Advance, CPXPARAM_Parallel, CPXPARAM_RandomSeed, CPXPARAM_ScreenOutput,
    CPXPARAM_Threads, CPXPARAM_TimeLimit,
};

// TODO: Not all parameters have been implemented yet.
// When implementing a parameter, make sure that the rust namespace matches the CPLEX namespace.
// Next parameter to implement: https://www.ibm.com/docs/en/icos/12.9.0?topic=parameters-benders-strategy

pub(crate) mod private {
    pub trait Parameter {}
}

impl private::Parameter for Advance {}
impl private::Parameter for ParallelMode {}
impl private::Parameter for Threads {}
impl private::Parameter for ScreenOutput {}
impl private::Parameter for RandomSeed {}
impl private::Parameter for TimeLimit {}

/// Parameter trait. It is a sealed trait, as it is supposed to be implemented
/// only within the cples_rs library
pub trait Parameter: private::Parameter {
    fn value(&self) -> ParameterValue;
    fn id(&self) -> u32;
}

#[derive(Copy, Clone, Debug)]
pub enum ParameterValue {
    Integer(c_int),
    Long(i64),
    Double(c_double),
    String(&'static str),
}

/// Advanced start switch.
/// <https://www.ibm.com/docs/en/icos/22.1.1?topic=parameters-advanced-start-switch>
#[derive(Copy, Clone, Debug)]
pub enum Advance {
    Unused = 0,
    AdvancedBasis = 1,
    AdvancedBasisOrStartingVector = 2,
}

impl Parameter for Advance {
    fn value(&self) -> ParameterValue {
        ParameterValue::Integer(match self {
            Self::Unused => 0,
            Self::AdvancedBasis => 1,
            Self::AdvancedBasisOrStartingVector => 2,
        })
    }

    fn id(&self) -> u32 {
        CPXPARAM_Advance
    }
}

/// Parallel mode switch.
/// <https://www.ibm.com/docs/en/icos/22.1.1?topic=parameters-parallel-mode-switch>
#[derive(Copy, Clone, Debug)]
pub enum ParallelMode {
    Opportunistic,
    Auto,
    Deterministic,
}

impl Parameter for ParallelMode {
    fn value(&self) -> ParameterValue {
        ParameterValue::Integer(match self {
            ParallelMode::Opportunistic => -1,
            ParallelMode::Auto => 0,
            ParallelMode::Deterministic => 1,
        })
    }

    fn id(&self) -> u32 {
        CPXPARAM_Parallel
    }
}

/// Global thread count.
/// <https://www.ibm.com/docs/en/icos/12.9.0?topic=parameters-global-thread-count>
#[derive(Copy, Clone, Debug)]
pub struct Threads(pub u32);

impl Parameter for Threads {
    fn value(&self) -> ParameterValue {
        ParameterValue::Integer(self.0 as i32)
    }

    fn id(&self) -> u32 {
        CPXPARAM_Threads
    }
}

/// Messages to screen switch.
/// <https://www.ibm.com/docs/en/icos/12.9.0?topic=parameters-messages-screen-switch>
#[derive(Copy, Clone, Debug)]
pub struct ScreenOutput(pub bool);

impl Parameter for ScreenOutput {
    fn value(&self) -> ParameterValue {
        ParameterValue::Integer(if self.0 { 1 } else { 0 })
    }

    fn id(&self) -> u32 {
        CPXPARAM_ScreenOutput
    }
}

/// Random seed.
/// <https://www.ibm.com/docs/en/icos/12.9.0?topic=parameters-random-seed>
#[derive(Copy, Clone, Debug)]
pub struct RandomSeed(pub u32);

impl Parameter for RandomSeed {
    fn value(&self) -> ParameterValue {
        ParameterValue::Integer(self.0 as i32)
    }

    fn id(&self) -> u32 {
        CPXPARAM_RandomSeed
    }
}

/// Optimizer time limit in seconds.
/// <https://www.ibm.com/docs/en/icos/12.9.0?topic=parameters-optimizer-time-limit-in-seconds>
#[derive(Copy, Clone, Debug)]
pub struct TimeLimit(pub Duration);

impl Parameter for TimeLimit {
    fn value(&self) -> ParameterValue {
        ParameterValue::Double(self.0.as_secs() as f64)
    }

    fn id(&self) -> u32 {
        CPXPARAM_TimeLimit
    }
}
