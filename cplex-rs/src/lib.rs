//! Safe rust bindings to the CPLEX solver API.
//!
//! # Example
//! ```
//! use cplex_rs::*;
//!
//! let env = Environment::new().unwrap();
//! let mut problem = Problem::new(env, "my_prob").unwrap();
//!
//! let v0 = problem.add_variable(Variable::new(VariableType::Continuous, 1.0, 0.0, 1.0, "x0")).unwrap();
//! let v1 = problem.add_variable(Variable::new(VariableType::Continuous, 10.0, 0.0, 1.0, "x1")).unwrap();
//! let c0 = problem.add_constraint(Constraint::new(ConstraintType::GreaterThanEq, 0.3, None, vec![(v0, 1.0)])).unwrap();
//! let c1 = problem.add_constraint(Constraint::new(ConstraintType::Eq, 1.0, None, vec![(v0, 1.0), (v1, 1.0)])).unwrap();
//!
//! let solution = problem.set_objective_type(ObjectiveType::Maximize)
//!    .unwrap()
//!    .solve_as(ProblemType::Linear)
//!    .unwrap();
//!
//! assert_eq!(solution.variable_value(v0), 0.3);
//! assert_eq!(solution.variable_value(v1), 0.7);
//! ```

pub mod constants;
mod constraints;
mod environment;
pub mod errors;
pub mod logging;
pub mod parameters;
mod solution;
mod variables;

pub use constraints::*;
pub use environment::*;
pub use errors::{Error, Result};
pub use ffi;
use ffi::{
    cpxlp, CPX_STAT_INForUNBD, CPXaddmipstarts, CPXaddrows, CPXchgobj, CPXchgobjsen,
    CPXchgprobtype, CPXchgqpcoef, CPXcreateprob, CPXfreeprob, CPXgetobjval, CPXgetstat, CPXgetx,
    CPXlpopt, CPXmipopt, CPXnewcols, CPXqpopt, CPXwriteprob, CPXMIP_UNBOUNDED, CPXPROB_LP,
    CPXPROB_MILP, CPXPROB_MIQP, CPXPROB_QP, CPX_MAX, CPX_MIN, CPX_STAT_INFEASIBLE,
    CPX_STAT_UNBOUNDED,
};
use log::debug;
pub use solution::*;
pub use variables::*;

use std::{
    ffi::{c_int, CString},
    time::Instant,
};

mod macros {
    macro_rules! cpx_lp_result {
    ( unsafe { $func:ident ( $env:expr, $lp:expr $(, $b:expr)* $(,)? ) } ) => {
        {
            let status = unsafe { $func($env, $lp $(,$b)* ) };
            if status != 0 {
                Err(errors::Error::from(errors::Cplex::from_code($env, $lp, status)))
            } else {
                Ok(())
            }
        }
    };
}

    pub(super) use cpx_lp_result;
}

/// A variable identifier, unique with respect to a given problem instance
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VariableId(usize);

impl VariableId {
    pub fn into_inner(self) -> usize {
        self.0
    }
}

/// A constraint identifier, unique with respect to a given problem instance
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ConstraintId(usize);

impl ConstraintId {
    pub fn into_inner(self) -> usize {
        self.0
    }
}

/// A CPLEX problem instance
pub struct Problem {
    inner: *mut cpxlp,
    env: Environment,
    variables: Vec<Variable>,
    constraints: Vec<Constraint>,
}

unsafe impl Send for Problem {}

#[derive(Copy, Clone, Debug)]
pub enum ObjectiveType {
    Maximize,
    Minimize,
}

impl ObjectiveType {
    fn into_raw(self) -> c_int {
        match self {
            ObjectiveType::Minimize => CPX_MIN as c_int,
            ObjectiveType::Maximize => CPX_MAX as c_int,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ProblemType {
    Linear,
    MixedInteger,
    Quadratic,
    MixedIntegerQuadratic,
}

impl ProblemType {
    fn into_raw(self) -> c_int {
        match self {
            ProblemType::Linear => CPXPROB_LP as c_int,
            ProblemType::MixedInteger => CPXPROB_MILP as c_int,
            ProblemType::Quadratic => CPXPROB_QP as c_int,
            ProblemType::MixedIntegerQuadratic => CPXPROB_MIQP as c_int,
        }
    }
}

impl Problem {
    /// Create a new CPLEX problem from a CPLEX environmant
    pub fn new<S>(env: Environment, name: S) -> Result<Self>
    where
        S: AsRef<str>,
    {
        let mut status = 0;
        let name =
            CString::new(name.as_ref()).map_err(|e| errors::Input::from_message(e.to_string()))?;
        let inner = unsafe { CPXcreateprob(env.inner, &mut status, name.as_ptr()) };
        if inner.is_null() {
            Err(errors::Cplex::from_code(env.inner, std::ptr::null(), status).into())
        } else {
            Ok(Problem {
                inner,
                env,
                variables: vec![],
                constraints: vec![],
            })
        }
    }

    /// Get a mutable reference to the environment of the problem.
    pub fn env_mut(&mut self) -> &mut Environment {
        &mut self.env
    }

    /// Get a reference to the environment of the problem.
    pub fn env(&self) -> &Environment {
        &self.env
    }

    /// Add a variable to the problem.
    ///
    /// The id for the Variable is returned.
    pub fn add_variable(&mut self, var: Variable) -> Result<VariableId> {
        let name = CString::new(var.name().as_bytes())
            .map_err(|e| errors::Input::from_message(e.to_string()))?;

        macros::cpx_lp_result!(unsafe {
            CPXnewcols(
                self.env.inner,
                self.inner,
                1,
                &var.weight(),
                &var.lower_bound(),
                &var.upper_bound(),
                &var.type_().into_raw() as *const u8 as *const i8,
                &mut (name.as_ptr() as *mut _),
            )
        })?;

        let index = self.variables.len();
        self.variables.push(var);
        Ok(VariableId(index))
    }

    /// Add an array of variables to the problem.
    ///
    /// The id for the variables are returned, in the same order they have been given in the input.
    pub fn add_variables(&mut self, vars: Vec<Variable>) -> Result<Vec<VariableId>> {
        let names = vars
            .iter()
            .map(|v| {
                CString::new(v.name().as_bytes())
                    .map_err(|e| errors::Input::from_message(e.to_string()).into())
            })
            .collect::<Result<Vec<_>>>()?;

        let mut name_ptrs = names
            .iter()
            .map(|n| n.as_ptr() as *mut _)
            .collect::<Vec<_>>();

        let objs = vars.iter().map(|v| v.weight()).collect::<Vec<_>>();
        let lbs = vars.iter().map(|v| v.lower_bound()).collect::<Vec<_>>();
        let ubs = vars.iter().map(|v| v.upper_bound()).collect::<Vec<_>>();
        let types = vars
            .iter()
            .map(|v| v.type_().into_raw() as i8)
            .collect::<Vec<_>>();

        macros::cpx_lp_result!(unsafe {
            CPXnewcols(
                self.env.inner,
                self.inner,
                vars.len() as i32,
                objs.as_ptr(),
                lbs.as_ptr(),
                ubs.as_ptr(),
                types.as_ptr(),
                name_ptrs.as_mut_ptr(),
            )
        })?;

        let indices: Vec<VariableId> = vars
            .iter()
            .enumerate()
            .map(|(idx, _)| VariableId(idx + self.variables.len()))
            .collect();
        self.variables.extend(vars);
        Ok(indices)
    }

    /// Add a constraint to the problem.
    ///
    /// The id for the constraint is returned.
    pub fn add_constraint(&mut self, constraint: Constraint) -> Result<ConstraintId> {
        let (ind, val): (Vec<c_int>, Vec<f64>) = constraint
            .weights()
            .iter()
            .filter(|(_, weight)| *weight != 0.0)
            .map(|(var_id, weight)| (var_id.0 as c_int, weight))
            .unzip();
        let nz = val.len() as c_int;
        let name = constraint
            .name()
            .map(|n| {
                CString::new(n.as_bytes()).map_err(|e| errors::Input::from_message(e.to_string()))
            })
            .transpose()?;
        macros::cpx_lp_result!(unsafe {
            CPXaddrows(
                self.env.inner,
                self.inner,
                0,
                1,
                nz,
                &constraint.rhs(),
                &constraint.type_().into_raw(),
                &0,
                ind.as_ptr(),
                val.as_ptr(),
                std::ptr::null_mut(),
                &mut (name
                    .as_ref()
                    .map(|n| n.as_ptr())
                    .unwrap_or(std::ptr::null()) as *mut _),
            )
        })?;

        let index = self.constraints.len();
        self.constraints.push(constraint);
        Ok(ConstraintId(index))
    }

    /// Add an array of constraints to the problem.
    ///
    /// The id for the constraints are returned, in the same order they have been given in the input.
    pub fn add_constraints(&mut self, con: Vec<Constraint>) -> Result<Vec<ConstraintId>> {
        if con.is_empty() {
            return Err(errors::Input::from_message(
                "Called add_constraints with 0 constaints".to_owned(),
            )
            .into());
        }
        let beg = std::iter::once(0)
            .chain(con[..con.len() - 1].iter().map(|c| c.weights().len()))
            .scan(0, |state, x| {
                *state += x;
                Some(*state as i32)
            })
            .collect::<Vec<_>>();

        let (ind, val): (Vec<c_int>, Vec<f64>) = con
            .iter()
            .flat_map(|c| c.weights().iter())
            .filter(|(_, weight)| *weight != 0.0)
            .map(|(var_id, weight)| (var_id.0 as c_int, weight))
            .unzip();

        let nz = val.len() as c_int;
        let names = con
            .iter()
            .map(|c| {
                c.name()
                    .map(|n| {
                        CString::new(n.as_bytes())
                            .map_err(|e| errors::Input::from_message(e.to_string()).into())
                    })
                    .transpose()
            })
            .collect::<Result<Vec<_>>>()?;

        let mut name_ptrs = names
            .iter()
            .map(|n| {
                n.as_ref()
                    .map(|n| n.as_ptr())
                    .unwrap_or(std::ptr::null_mut()) as *mut _
            })
            .collect::<Vec<_>>();

        let rhss = con.iter().map(|c| c.rhs()).collect::<Vec<_>>();
        let senses = con.iter().map(|c| c.type_().into_raw()).collect::<Vec<_>>();

        macros::cpx_lp_result!(unsafe {
            CPXaddrows(
                self.env.inner,
                self.inner,
                0,
                con.len() as i32,
                nz,
                rhss.as_ptr(),
                senses.as_ptr(),
                beg.as_ptr(),
                ind.as_ptr(),
                val.as_ptr(),
                std::ptr::null_mut(),
                name_ptrs.as_mut_ptr(),
            )
        })?;

        let indices = con
            .iter()
            .enumerate()
            .map(|(idx, _)| ConstraintId(idx + self.constraints.len()))
            .collect();
        self.constraints.extend(con);
        Ok(indices)
    }

    /// Set the objective coefficients.
    pub fn set_objective(self, ty: ObjectiveType, obj: Vec<(VariableId, f64)>) -> Result<Self> {
        let (ind, val): (Vec<c_int>, Vec<f64>) = obj
            .into_iter()
            .map(|(var_id, weight)| (var_id.0 as c_int, weight))
            .unzip();

        macros::cpx_lp_result!(unsafe {
            CPXchgobj(
                self.env.inner,
                self.inner,
                ind.len() as c_int,
                ind.as_ptr(),
                val.as_ptr(),
            )
        })?;

        self.set_objective_type(ty)
    }

    /// Set quadratic objective coefficients, following the CPLEX convention.
    /// The objective is `c'x + 1/2 x'Qx`, and each entry `(i, j, v)` sets `Q[i][j]`.
    ///
    /// - `(i, i, v)` sets the diagonal entry `Q[i][i]`, contributing `1/2 * v * x_i^2`
    /// - `(i, j, v)` with `i != j` sets both `Q[i][j]` and `Q[j][i]`, contributing
    ///    `v * x_i * x_j`. `(i, j, v)` and `(j, i, v)` are equivalent
    ///
    /// E.g. to minimize `x^2`, pass `(x, x, 2.0)`. If the same unordered pair appears more than
    /// once, the last entry wins.
    ///
    /// Setting quadratic coefficients changes the CPLEX problem type to a quadratic one.
    /// Solve with [`ProblemType::Quadratic`] or [`ProblemType::MixedIntegerQuadratic`].
    ///
    /// For minimization the quadratic part must be convex (`Q` positive semidefinite), for
    /// maximization concave (`Q` negative semidefinite).
    pub fn set_quadratic_objective(self, obj: Vec<(VariableId, VariableId, f64)>) -> Result<Self> {
        if let Some((i, j, _)) = obj
            .iter()
            .find(|(i, j, _)| i.0 >= self.variables.len() || j.0 >= self.variables.len())
        {
            return Err(errors::Input::from_message(format!(
                "quadratic objective entry ({}, {}) out of bounds for problem with {} variables",
                i.0,
                j.0,
                self.variables.len()
            ))
            .into());
        }

        for (i, j, value) in obj {
            macros::cpx_lp_result!(unsafe {
                CPXchgqpcoef(
                    self.env.inner,
                    self.inner,
                    i.0 as c_int,
                    j.0 as c_int,
                    value,
                )
            })?;
        }
        Ok(self)
    }

    /// Change the objective type. Default: `ObjectiveType::Minimize`.
    pub fn set_objective_type(self, ty: ObjectiveType) -> Result<Self> {
        macros::cpx_lp_result!(unsafe { CPXchgobjsen(self.env.inner, self.inner, ty.into_raw()) })?;
        Ok(self)
    }

    /// Write the problem to a file named `name`.
    pub fn write<S>(&self, name: S) -> Result<()>
    where
        S: AsRef<str>,
    {
        let name =
            CString::new(name.as_ref()).map_err(|e| errors::Input::from_message(e.to_string()))?;

        macros::cpx_lp_result!(unsafe {
            CPXwriteprob(self.env.inner, self.inner, name.as_ptr(), std::ptr::null())
        })
    }

    /// Add an initial solution to the problem.
    ///
    /// `vars` is an array of indices (i.e. the result of `prob.add_variable`) and `values` are
    /// their values.
    pub fn add_initial_soln(&mut self, vars: &[VariableId], values: &[f64]) -> Result<()> {
        if values.len() != vars.len() {
            return Err(errors::Input::from_message(
                "number of solution variables and values does not match".to_string(),
            )
            .into());
        }
        let vars = vars.iter().map(|&u| u.0 as c_int).collect::<Vec<_>>();

        macros::cpx_lp_result!(unsafe {
            CPXaddmipstarts(
                self.env.inner,
                self.inner,
                1,
                vars.len() as c_int,
                &0,
                vars.as_ptr(),
                values.as_ptr(),
                &0,
                &mut std::ptr::null_mut(),
            )
        })
    }

    /// Solve the Problem, returning a `Solution` object with the
    /// result.
    pub fn solve_as(self, pt: ProblemType) -> Result<Solution> {
        macros::cpx_lp_result!(unsafe {
            CPXchgprobtype(self.env.inner, self.inner, pt.into_raw())
        })?;

        let start_optim = Instant::now();
        match pt {
            ProblemType::MixedInteger | ProblemType::MixedIntegerQuadratic => {
                macros::cpx_lp_result!(unsafe { CPXmipopt(self.env.inner, self.inner) })?
            }
            ProblemType::Linear => {
                macros::cpx_lp_result!(unsafe { CPXlpopt(self.env.inner, self.inner) })?
            }
            ProblemType::Quadratic => {
                macros::cpx_lp_result!(unsafe { CPXqpopt(self.env.inner, self.inner) })?
            }
        };
        let elapsed = start_optim.elapsed();
        debug!("CPLEX model solution took: {:?}", elapsed);

        let code = unsafe { CPXgetstat(self.env.inner, self.inner) };
        if code as u32 == CPX_STAT_INFEASIBLE || code as u32 == CPX_STAT_INForUNBD {
            return Err(crate::errors::Cplex::Unfeasible {
                code,
                message: "Unfeasible problem".to_string(),
            }
            .into());
        }

        if code as u32 == CPX_STAT_UNBOUNDED || code as u32 == CPXMIP_UNBOUNDED {
            return Err(crate::errors::Cplex::Unbounded {
                code,
                message: "Unbounded problem".to_string(),
            }
            .into());
        }

        let mut objective_value: f64 = 0.0;
        macros::cpx_lp_result!(unsafe {
            CPXgetobjval(self.env.inner, self.inner, &mut objective_value)
        })?;

        let mut variable_values = vec![0f64; self.variables.len()];
        macros::cpx_lp_result!(unsafe {
            CPXgetx(
                self.env.inner,
                self.inner,
                variable_values.as_mut_ptr(),
                0,
                self.variables.len() as c_int - 1,
            )
        })?;

        Ok(Solution::new(variable_values, objective_value))
    }
}

impl Drop for Problem {
    fn drop(&mut self) {
        unsafe {
            assert_eq!(CPXfreeprob(self.env.inner, &mut self.inner), 0);
        }
    }
}

#[cfg(test)]
mod test {
    use constants::INFINITY;
    use constraints::ConstraintType;

    use super::*;
    use variables::{Variable, VariableType};

    #[test]
    fn mipex1() {
        let env = Environment::new().unwrap();
        let mut problem = Problem::new(env, "mipex1").unwrap();

        let x0 = problem
            .add_variable(Variable::new(
                VariableType::Continuous,
                1.0,
                0.0,
                40.0,
                "x0",
            ))
            .unwrap();

        let x1 = problem
            .add_variable(Variable::new(
                VariableType::Continuous,
                2.0,
                0.0,
                INFINITY,
                "x1",
            ))
            .unwrap();

        let x2 = problem
            .add_variable(Variable::new(
                VariableType::Continuous,
                3.0,
                0.0,
                INFINITY,
                "x2",
            ))
            .unwrap();

        let x3 = problem
            .add_variable(Variable::new(VariableType::Integer, 1.0, 2.0, 3.0, "x3"))
            .unwrap();

        assert_eq!(x0, VariableId(0));
        assert_eq!(x1, VariableId(1));
        assert_eq!(x2, VariableId(2));
        assert_eq!(x3, VariableId(3));

        let c0 = problem
            .add_constraint(Constraint::new(
                ConstraintType::LessThanEq,
                20.0,
                None,
                vec![(x0, -1.0), (x1, 1.0), (x2, 1.0), (x3, 10.0)],
            ))
            .unwrap();

        let c1 = problem
            .add_constraint(Constraint::new(
                ConstraintType::LessThanEq,
                30.0,
                None,
                vec![(x0, 1.0), (x1, -3.0), (x2, 1.0)],
            ))
            .unwrap();

        let c2 = problem
            .add_constraint(Constraint::new(
                ConstraintType::Eq,
                0.0,
                None,
                vec![(x1, 1.0), (x3, -3.5)],
            ))
            .unwrap();

        assert_eq!(c0, ConstraintId(0));
        assert_eq!(c1, ConstraintId(1));
        assert_eq!(c2, ConstraintId(2));

        let problem = problem.set_objective_type(ObjectiveType::Maximize).unwrap();

        let solution = problem.solve_as(ProblemType::MixedInteger).unwrap();

        assert_eq!(solution.objective_value(), 122.5);
    }

    #[test]
    fn mipex1_batch() {
        let env = Environment::new().unwrap();
        let mut problem = Problem::new(env, "mipex1").unwrap();

        let vars = problem
            .add_variables(vec![
                Variable::new(VariableType::Continuous, 1.0, 0.0, 40.0, "x0"),
                Variable::new(VariableType::Continuous, 2.0, 0.0, INFINITY, "x1"),
                Variable::new(VariableType::Continuous, 3.0, 0.0, INFINITY, "x2"),
                Variable::new(VariableType::Integer, 1.0, 2.0, 3.0, "x3"),
            ])
            .unwrap();

        assert_eq!(
            vars,
            vec![VariableId(0), VariableId(1), VariableId(2), VariableId(3)]
        );

        let cons = problem
            .add_constraints(vec![
                Constraint::new(
                    ConstraintType::LessThanEq,
                    20.0,
                    None,
                    vec![
                        (vars[0], -1.0),
                        (vars[1], 1.0),
                        (vars[2], 1.0),
                        (vars[3], 10.0),
                    ],
                ),
                Constraint::new(
                    ConstraintType::LessThanEq,
                    30.0,
                    None,
                    vec![(vars[0], 1.0), (vars[1], -3.0), (vars[2], 1.0)],
                ),
                Constraint::new(
                    ConstraintType::Eq,
                    0.0,
                    None,
                    vec![(vars[1], 1.0), (vars[3], -3.5)],
                ),
            ])
            .unwrap();

        assert_eq!(
            cons,
            vec![ConstraintId(0), ConstraintId(1), ConstraintId(2)]
        );

        let problem = problem.set_objective_type(ObjectiveType::Maximize).unwrap();

        let solution = problem.solve_as(ProblemType::MixedInteger).unwrap();

        assert_eq!(solution.objective_value(), 122.5);
    }

    #[test]
    fn unfeasible() {
        let env = Environment::new().unwrap();
        let mut problem = Problem::new(env, "unfeasible").unwrap();

        let vars = problem
            .add_variables(vec![
                Variable::new(VariableType::Continuous, 1.0, 0.0, 1.0, "x0"),
                Variable::new(VariableType::Continuous, 1.0, 0.0, 1.0, "x1"),
            ])
            .unwrap();

        assert_eq!(vars, vec![VariableId(0), VariableId(1)]);

        let cons = problem
            .add_constraints(vec![
                Constraint::new(
                    ConstraintType::Eq,
                    0.0,
                    None,
                    vec![(vars[0], 1.0), (vars[1], 1.0)],
                ),
                Constraint::new(
                    ConstraintType::Eq,
                    1.0,
                    None,
                    vec![(vars[0], 1.0), (vars[1], 1.0)],
                ),
            ])
            .unwrap();

        assert_eq!(cons, vec![ConstraintId(0), ConstraintId(1)]);

        let problem = problem.set_objective_type(ObjectiveType::Maximize).unwrap();
        assert!(matches!(
            problem.solve_as(ProblemType::Linear),
            Err(errors::Error::Cplex(errors::Cplex::Unfeasible { .. }))
        ));
    }

    #[test]
    fn unbounded() {
        let env = Environment::new().unwrap();
        let mut problem = Problem::new(env, "unbounded").unwrap();

        problem
            .add_variable(Variable::new(
                VariableType::Integer,
                1.0,
                0.0,
                INFINITY,
                "x0",
            ))
            .unwrap();

        let problem = problem.set_objective_type(ObjectiveType::Maximize).unwrap();

        assert!(matches!(
            problem.solve_as(ProblemType::MixedInteger),
            Err(errors::Error::Cplex(errors::Cplex::Unbounded { .. }))
        ));
    }

    #[test]
    fn qpex1() {
        let env = Environment::new().unwrap();
        let mut problem = Problem::new(env, "qpex1").unwrap();

        let vars = problem
            .add_variables(vec![
                Variable::new(VariableType::Continuous, 1.0, 0.0, 40.0, "x0"),
                Variable::new(VariableType::Continuous, 2.0, 0.0, INFINITY, "x1"),
                Variable::new(VariableType::Continuous, 3.0, 0.0, INFINITY, "x2"),
            ])
            .unwrap();

        problem
            .add_constraints(vec![
                Constraint::new(
                    ConstraintType::LessThanEq,
                    20.0,
                    None,
                    vec![(vars[0], -1.0), (vars[1], 1.0), (vars[2], 1.0)],
                ),
                Constraint::new(
                    ConstraintType::LessThanEq,
                    30.0,
                    None,
                    vec![(vars[0], 1.0), (vars[1], -3.0), (vars[2], 1.0)],
                ),
            ])
            .unwrap();

        // maximize x0 + 2 x1 + 3 x2
        //          - 0.5 (33 x0^2 + 22 x1^2 + 11 x2^2 - 12 x0 x1 - 23 x1 x2)
        let solution = problem
            .set_objective_type(ObjectiveType::Maximize)
            .unwrap()
            .set_quadratic_objective(vec![
                (vars[0], vars[0], -33.0),
                (vars[1], vars[1], -22.0),
                (vars[2], vars[2], -11.0),
                (vars[0], vars[1], 6.0),
                (vars[1], vars[2], 11.5),
            ])
            .unwrap()
            .solve_as(ProblemType::Quadratic)
            .unwrap();

        assert!((solution.objective_value() - 2.015617).abs() < 1e-4);
    }

    #[test]
    fn miqp_simple() {
        let env = Environment::new().unwrap();
        let mut problem = Problem::new(env, "miqp_simple").unwrap();

        // minimize x^2 - 2.6 x, x integer in [0, 10] => x = 1, objective -1.6
        // x^2 = 1/2 * 2.0 * x^2, so Q[x][x] = 2.0
        let x = problem
            .add_variable(Variable::new(VariableType::Integer, -2.6, 0.0, 10.0, "x"))
            .unwrap();

        let solution = problem
            .set_objective_type(ObjectiveType::Minimize)
            .unwrap()
            .set_quadratic_objective(vec![(x, x, 2.0)])
            .unwrap()
            .solve_as(ProblemType::MixedIntegerQuadratic)
            .unwrap();

        assert!((solution.variable_value(x) - 1.0).abs() < 1e-6);
        assert!((solution.objective_value() + 1.6).abs() < 1e-6);
    }

    #[test]
    fn quadratic_objective_out_of_bounds() {
        let env = Environment::new().unwrap();
        let mut problem = Problem::new(env, "qp_oob").unwrap();

        let x = problem
            .add_variable(Variable::new(VariableType::Continuous, 1.0, 0.0, 1.0, "x"))
            .unwrap();

        assert!(matches!(
            problem.set_quadratic_objective(vec![(x, VariableId(5), 1.0)]),
            Err(errors::Error::Input(_))
        ));
    }
}
