use std::ffi::{c_char, c_int};

use ffi::{
    cpxenv, cpxlp, CPXPARAM_Conflict_Algorithm, CPXgetconflictext, CPXgetintparam, CPXgetstat,
    CPXrefineconflictext, CPXsetintparam, CPX_CONFLICTALG_AUTO, CPX_CONFLICT_EXCLUDED,
    CPX_CONFLICT_MEMBER, CPX_CONFLICT_POSSIBLE_MEMBER, CPX_CON_LINEAR, CPX_CON_LOWER_BOUND,
    CPX_CON_UPPER_BOUND, CPX_STAT_CONFLICT_ABORT_CONTRADICTION,
    CPX_STAT_CONFLICT_ABORT_DETTIME_LIM, CPX_STAT_CONFLICT_ABORT_IT_LIM,
    CPX_STAT_CONFLICT_ABORT_MEM_LIM, CPX_STAT_CONFLICT_ABORT_NODE_LIM,
    CPX_STAT_CONFLICT_ABORT_OBJ_LIM, CPX_STAT_CONFLICT_ABORT_TIME_LIM,
    CPX_STAT_CONFLICT_ABORT_USER, CPX_STAT_CONFLICT_FEASIBLE, CPX_STAT_CONFLICT_MINIMAL,
};
use log::error;

use crate::{errors, ConstraintId, Problem, Result, VariableId};

/// The outcome of an IIS computation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IisStatus {
    /// CPLEX proved that the reported members form a minimal conflict.
    Minimal,
    /// CPLEX proved that the model is feasible, so no IIS exists.
    Feasible,
    /// CPLEX stopped before proving minimality.
    Aborted(IisAbortReason),
    /// A status introduced by a newer CPLEX version.
    Unknown(c_int),
}

impl IisStatus {
    fn from_raw(status: c_int) -> Self {
        match status as u32 {
            CPX_STAT_CONFLICT_MINIMAL => Self::Minimal,
            CPX_STAT_CONFLICT_FEASIBLE => Self::Feasible,
            CPX_STAT_CONFLICT_ABORT_CONTRADICTION => Self::Aborted(IisAbortReason::Contradiction),
            CPX_STAT_CONFLICT_ABORT_TIME_LIM => Self::Aborted(IisAbortReason::TimeLimit),
            CPX_STAT_CONFLICT_ABORT_DETTIME_LIM => {
                Self::Aborted(IisAbortReason::DeterministicTimeLimit)
            }
            CPX_STAT_CONFLICT_ABORT_IT_LIM => Self::Aborted(IisAbortReason::IterationLimit),
            CPX_STAT_CONFLICT_ABORT_NODE_LIM => Self::Aborted(IisAbortReason::NodeLimit),
            CPX_STAT_CONFLICT_ABORT_OBJ_LIM => Self::Aborted(IisAbortReason::ObjectiveLimit),
            CPX_STAT_CONFLICT_ABORT_MEM_LIM => Self::Aborted(IisAbortReason::MemoryLimit),
            CPX_STAT_CONFLICT_ABORT_USER => Self::Aborted(IisAbortReason::User),
            _ => Self::Unknown(status),
        }
    }
}

/// Why CPLEX stopped refining a conflict before proving it minimal.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IisAbortReason {
    Contradiction,
    TimeLimit,
    DeterministicTimeLimit,
    IterationLimit,
    NodeLimit,
    ObjectiveLimit,
    MemoryLimit,
    User,
}

/// Whether CPLEX proved that an item belongs to the conflict.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IisMembership {
    Member,
    PossibleMember,
}

impl IisMembership {
    fn from_raw(status: c_int) -> Option<Self> {
        match status {
            value if value == CPX_CONFLICT_MEMBER as c_int => Some(Self::Member),
            value if value == CPX_CONFLICT_POSSIBLE_MEMBER as c_int => Some(Self::PossibleMember),
            _ => None,
        }
    }
}

/// One linear constraint reported by the IIS computation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IisConstraint {
    constraint: ConstraintId,
    membership: IisMembership,
}

impl IisConstraint {
    pub fn constraint(&self) -> ConstraintId {
        self.constraint
    }

    pub fn membership(&self) -> IisMembership {
        self.membership
    }
}

/// The side of a variable bound reported by the IIS computation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IisBound {
    Lower,
    Upper,
}

/// One variable bound reported by the IIS computation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IisVariableBound {
    variable: VariableId,
    bound: IisBound,
    membership: IisMembership,
}

impl IisVariableBound {
    pub fn variable(&self) -> VariableId {
        self.variable
    }

    pub fn bound(&self) -> IisBound {
        self.bound
    }

    pub fn membership(&self) -> IisMembership {
        self.membership
    }
}

/// A typed report from CPLEX's IIS computation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct IisReport {
    status: IisStatus,
    constraints: Vec<IisConstraint>,
    variable_bounds: Vec<IisVariableBound>,
}

impl IisReport {
    pub fn status(&self) -> IisStatus {
        self.status
    }

    pub fn constraints(&self) -> &[IisConstraint] {
        &self.constraints
    }

    pub fn variable_bounds(&self) -> &[IisVariableBound] {
        &self.variable_bounds
    }
}

struct ConflictAlgorithmGuard {
    env: *mut cpxenv,
    previous: c_int,
    active: bool,
}

impl ConflictAlgorithmGuard {
    fn restore(mut self) -> Result<()> {
        let status = unsafe {
            CPXsetintparam(
                self.env,
                CPXPARAM_Conflict_Algorithm as c_int,
                self.previous,
            )
        };
        self.active = false;
        env_result(self.env, status)
    }
}

impl Drop for ConflictAlgorithmGuard {
    fn drop(&mut self) {
        if self.active {
            let status = unsafe {
                CPXsetintparam(
                    self.env,
                    CPXPARAM_Conflict_Algorithm as c_int,
                    self.previous,
                )
            };
            if status != 0 {
                error!("Unable to restore CPLEX conflict algorithm, got status: '{status}'");
            }
        }
    }
}

fn env_result(env: *const cpxenv, status: c_int) -> Result<()> {
    if status == 0 {
        Ok(())
    } else {
        Err(errors::Cplex::from_code(env, std::ptr::null(), status).into())
    }
}

fn problem_result(env: *const cpxenv, lp: *const cpxlp, status: c_int) -> Result<()> {
    if status == 0 {
        Ok(())
    } else {
        Err(errors::Cplex::from_code(env, lp, status).into())
    }
}

fn checked_c_int(value: usize, description: &str) -> Result<c_int> {
    c_int::try_from(value).map_err(|_| {
        errors::Input::from_message(format!("{description} does not fit in a CPLEX index")).into()
    })
}

fn slice_ptr_or_null<T>(slice: &[T]) -> *const T {
    if slice.is_empty() {
        std::ptr::null()
    } else {
        slice.as_ptr()
    }
}

impl Problem {
    /// Compute an irreducible infeasible set (IIS) for this problem.
    ///
    /// The method does not consume the problem and can be called without first solving it. A
    /// [`IisStatus::Minimal`] report is a proven IIS. If a configured resource limit interrupts
    /// refinement, [`IisStatus::Aborted`] preserves both proven and possible conflict members
    /// without claiming that the result is irreducible.
    pub fn compute_iis(&mut self) -> Result<IisReport> {
        let env = self.env.inner;
        let lp = self.inner;

        let mut previous_algorithm = 0;
        env_result(env, unsafe {
            CPXgetintparam(
                env,
                CPXPARAM_Conflict_Algorithm as c_int,
                &mut previous_algorithm,
            )
        })?;
        env_result(env, unsafe {
            // The dedicated IIS algorithm treats the model as continuous. Let CPLEX select an
            // exact conflict algorithm so integer and semi-integer domains remain part of the
            // feasibility checks as well.
            CPXsetintparam(
                env,
                CPXPARAM_Conflict_Algorithm as c_int,
                CPX_CONFLICTALG_AUTO as c_int,
            )
        })?;
        let algorithm_guard = ConflictAlgorithmGuard {
            env,
            previous: previous_algorithm,
            active: true,
        };

        let report_result = (|| {
            let group_count = self
                .variables
                .len()
                .checked_mul(2)
                .and_then(|count| count.checked_add(self.constraints.len()))
                .ok_or_else(|| {
                    errors::Error::from(errors::Input::from_message(
                        "number of IIS groups overflowed usize".to_owned(),
                    ))
                })?;
            let group_count_raw = checked_c_int(group_count, "number of IIS groups")?;

            let mut preferences = Vec::with_capacity(group_count);
            let mut group_beginnings = Vec::with_capacity(group_count);
            let mut group_indices = Vec::with_capacity(group_count);
            let mut group_types = Vec::with_capacity(group_count);

            let mut add_group = |index: usize, group_type: u32| -> Result<()> {
                preferences.push(1.0);
                group_beginnings.push(checked_c_int(group_indices.len(), "IIS group offset")?);
                group_indices.push(checked_c_int(index, "IIS member index")?);
                group_types.push(group_type as c_char);
                Ok(())
            };

            for index in 0..self.variables.len() {
                add_group(index, CPX_CON_LOWER_BOUND)?;
            }
            for index in 0..self.variables.len() {
                add_group(index, CPX_CON_UPPER_BOUND)?;
            }
            for index in 0..self.constraints.len() {
                add_group(index, CPX_CON_LINEAR)?;
            }

            problem_result(env, lp, unsafe {
                CPXrefineconflictext(
                    env,
                    lp,
                    group_count_raw,
                    group_count_raw,
                    slice_ptr_or_null(&preferences),
                    slice_ptr_or_null(&group_beginnings),
                    slice_ptr_or_null(&group_indices),
                    slice_ptr_or_null(&group_types),
                )
            })?;

            let status = IisStatus::from_raw(unsafe { CPXgetstat(env, lp) });
            if status == IisStatus::Feasible || group_count == 0 {
                return Ok(IisReport {
                    status,
                    constraints: Vec::new(),
                    variable_bounds: Vec::new(),
                });
            }

            let mut group_statuses = vec![CPX_CONFLICT_EXCLUDED; group_count];
            problem_result(env, lp, unsafe {
                CPXgetconflictext(env, lp, group_statuses.as_mut_ptr(), 0, group_count_raw - 1)
            })?;

            let variable_count = self.variables.len();
            let mut variable_bounds = Vec::new();
            let mut constraints = Vec::new();
            for (group, raw_membership) in group_statuses.into_iter().enumerate() {
                let Some(membership) = IisMembership::from_raw(raw_membership) else {
                    continue;
                };

                if group < variable_count {
                    variable_bounds.push(IisVariableBound {
                        variable: VariableId(group),
                        bound: IisBound::Lower,
                        membership,
                    });
                } else if group < variable_count * 2 {
                    variable_bounds.push(IisVariableBound {
                        variable: VariableId(group - variable_count),
                        bound: IisBound::Upper,
                        membership,
                    });
                } else {
                    constraints.push(IisConstraint {
                        constraint: ConstraintId(group - variable_count * 2),
                        membership,
                    });
                }
            }

            Ok(IisReport {
                status,
                constraints,
                variable_bounds,
            })
        })();

        let restore_result = algorithm_guard.restore();
        match (report_result, restore_result) {
            (Ok(report), Ok(())) => Ok(report),
            (Err(report_error), Ok(())) => Err(report_error),
            (Ok(_), Err(restore_error)) => Err(restore_error),
            (Err(report_error), Err(restore_error)) => {
                error!("Unable to restore CPLEX conflict algorithm: {restore_error}");
                Err(report_error)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        constants::INFINITY, Constraint, ConstraintType, Environment, Variable, VariableType,
    };
    use ffi::{CPXgetintparam, CPXsetintparam, CPX_CONFLICTALG_FAST};

    #[test]
    fn maps_all_known_abort_statuses() {
        let cases = [
            (
                CPX_STAT_CONFLICT_ABORT_CONTRADICTION,
                IisAbortReason::Contradiction,
            ),
            (CPX_STAT_CONFLICT_ABORT_TIME_LIM, IisAbortReason::TimeLimit),
            (
                CPX_STAT_CONFLICT_ABORT_DETTIME_LIM,
                IisAbortReason::DeterministicTimeLimit,
            ),
            (
                CPX_STAT_CONFLICT_ABORT_IT_LIM,
                IisAbortReason::IterationLimit,
            ),
            (CPX_STAT_CONFLICT_ABORT_NODE_LIM, IisAbortReason::NodeLimit),
            (
                CPX_STAT_CONFLICT_ABORT_OBJ_LIM,
                IisAbortReason::ObjectiveLimit,
            ),
            (CPX_STAT_CONFLICT_ABORT_MEM_LIM, IisAbortReason::MemoryLimit),
            (CPX_STAT_CONFLICT_ABORT_USER, IisAbortReason::User),
        ];

        for (raw, expected) in cases {
            assert_eq!(
                IisStatus::from_raw(raw as c_int),
                IisStatus::Aborted(expected)
            );
        }
        assert_eq!(IisStatus::from_raw(123_456), IisStatus::Unknown(123_456));
    }

    #[test]
    fn maps_conflict_memberships() {
        assert_eq!(
            IisMembership::from_raw(CPX_CONFLICT_MEMBER as c_int),
            Some(IisMembership::Member)
        );
        assert_eq!(
            IisMembership::from_raw(CPX_CONFLICT_POSSIBLE_MEMBER as c_int),
            Some(IisMembership::PossibleMember)
        );
        assert_eq!(IisMembership::from_raw(CPX_CONFLICT_EXCLUDED), None);
    }

    #[test]
    fn computes_minimal_iis_for_contradictory_rows() {
        let env = Environment::new().unwrap();
        let mut problem = Problem::new(env, "iis_rows").unwrap();
        let x = problem
            .add_variable(Variable::new(
                VariableType::Continuous,
                0.0,
                -INFINITY,
                INFINITY,
                "x",
            ))
            .unwrap();
        let first = problem
            .add_constraint(Constraint::new(
                ConstraintType::Eq,
                0.0,
                Some("first".to_owned()),
                vec![(x, 1.0)],
            ))
            .unwrap();
        let second = problem
            .add_constraint(Constraint::new(
                ConstraintType::Eq,
                1.0,
                Some("second".to_owned()),
                vec![(x, 1.0)],
            ))
            .unwrap();

        let report = problem.compute_iis().unwrap();

        assert_eq!(report.status(), IisStatus::Minimal);
        assert_eq!(report.variable_bounds(), &[]);
        assert_eq!(
            report
                .constraints()
                .iter()
                .map(IisConstraint::constraint)
                .collect::<Vec<_>>(),
            vec![first, second]
        );
    }

    #[test]
    fn computes_bound_driven_iis_and_restores_algorithm() {
        let env = Environment::new().unwrap();
        let mut problem = Problem::new(env, "iis_bound").unwrap();
        let x = problem
            .add_variable(Variable::new(VariableType::Continuous, 0.0, 0.0, 1.0, "x"))
            .unwrap();
        let demand = problem
            .add_constraint(Constraint::new(
                ConstraintType::GreaterThanEq,
                2.0,
                Some("demand".to_owned()),
                vec![(x, 1.0)],
            ))
            .unwrap();

        assert_eq!(
            unsafe {
                CPXsetintparam(
                    problem.env.inner,
                    CPXPARAM_Conflict_Algorithm as c_int,
                    CPX_CONFLICTALG_FAST as c_int,
                )
            },
            0
        );

        let report = problem.compute_iis().unwrap();

        let mut restored_algorithm = -1;
        assert_eq!(
            unsafe {
                CPXgetintparam(
                    problem.env.inner,
                    CPXPARAM_Conflict_Algorithm as c_int,
                    &mut restored_algorithm,
                )
            },
            0
        );
        assert_eq!(restored_algorithm, CPX_CONFLICTALG_FAST as c_int);
        assert_eq!(report.status(), IisStatus::Minimal);
        assert_eq!(report.constraints().len(), 1);
        assert_eq!(report.constraints()[0].constraint(), demand);
        assert_eq!(report.variable_bounds().len(), 1);
        assert_eq!(report.variable_bounds()[0].variable(), x);
        assert_eq!(report.variable_bounds()[0].bound(), IisBound::Upper);
    }

    #[test]
    fn reports_feasible_problem_and_restores_algorithm() {
        let env = Environment::new().unwrap();
        let mut problem = Problem::new(env, "iis_feasible").unwrap();
        problem
            .add_variable(Variable::new(VariableType::Continuous, 0.0, 0.0, 1.0, "x"))
            .unwrap();

        assert_eq!(
            unsafe {
                CPXsetintparam(
                    problem.env.inner,
                    CPXPARAM_Conflict_Algorithm as c_int,
                    CPX_CONFLICTALG_FAST as c_int,
                )
            },
            0
        );

        let report = problem.compute_iis().unwrap();

        let mut restored_algorithm = -1;
        assert_eq!(
            unsafe {
                CPXgetintparam(
                    problem.env.inner,
                    CPXPARAM_Conflict_Algorithm as c_int,
                    &mut restored_algorithm,
                )
            },
            0
        );
        assert_eq!(restored_algorithm, CPX_CONFLICTALG_FAST as c_int);
        assert_eq!(report.status(), IisStatus::Feasible);
        assert!(report.constraints().is_empty());
        assert!(report.variable_bounds().is_empty());
    }

    #[test]
    fn reports_empty_problem_as_feasible() {
        let env = Environment::new().unwrap();
        let mut problem = Problem::new(env, "iis_empty").unwrap();

        let report = problem.compute_iis().unwrap();

        assert_eq!(report.status(), IisStatus::Feasible);
        assert!(report.constraints().is_empty());
        assert!(report.variable_bounds().is_empty());
    }

    #[test]
    fn computes_iis_for_integrality_driven_infeasibility() {
        let env = Environment::new().unwrap();
        let mut problem = Problem::new(env, "iis_integrality").unwrap();
        let x = problem
            .add_variable(Variable::new(VariableType::Integer, 0.0, 0.0, 1.0, "x"))
            .unwrap();
        let half = problem
            .add_constraint(Constraint::new(
                ConstraintType::Eq,
                0.5,
                Some("half".to_owned()),
                vec![(x, 1.0)],
            ))
            .unwrap();

        let report = problem.compute_iis().unwrap();

        assert_eq!(report.status(), IisStatus::Minimal);
        assert_eq!(report.constraints().len(), 1);
        assert_eq!(report.constraints()[0].constraint(), half);
        assert_eq!(report.constraints()[0].membership(), IisMembership::Member);
        assert!(report.variable_bounds().is_empty());
    }
}
