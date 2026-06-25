import dill
import sympy as sp
from sympy import cse
from sympy.core.function import AppliedUndef
from pathlib import Path
from Config import RobotConfig

def load_equations(filepath):
    """Load legacy dill equations or the portable srepr-based schema."""
    with open(filepath, 'rb') as f:
        equations = dill.load(f)

    if equations.get('serialization') != 'sympy-srepr-v1':
        return equations

    decoded = {}
    for key, value in equations.items():
        if key in {'serialization', 'model_assumptions'}:
            decoded[key] = value
        elif key in {'symbols', 'functions'}:
            decoded[key] = {
                name: sp.sympify(expression)
                for name, expression in value.items()
            }
        else:
            decoded[key] = sp.sympify(value)
    return decoded

def validate_equations(M, C, G, D):
    """Fail before code generation when the serialized model is inconsistent."""
    if M.rows != M.cols:
        raise ValueError(f"M must be square, got {M.shape}")

    n = M.rows
    expected_matrix_shape = (n, n)
    if C.shape != expected_matrix_shape:
        raise ValueError(f"C must have shape {expected_matrix_shape}, got {C.shape}")
    if D.shape != expected_matrix_shape:
        raise ValueError(f"D must have shape {expected_matrix_shape}, got {D.shape}")
    if G.shape not in ((n, 1), (n,)):
        raise ValueError(f"G must have {n} elements, got {G.shape}")

    if sp.simplify(M - M.T) != sp.zeros(n, n):
        raise ValueError("Serialized mass matrix M is not symmetric")

    return n


def validate_substituted_expressions(expressions, allowed_symbols):
    """Ensure generated C++ cannot contain unresolved SymPy functions."""
    unresolved_functions = set()
    unresolved_derivatives = set()
    free_symbols = set()

    for expression in expressions:
        unresolved_functions.update(expression.atoms(AppliedUndef))
        unresolved_derivatives.update(expression.atoms(sp.Derivative))
        free_symbols.update(expression.free_symbols)

    unexpected_symbols = free_symbols - allowed_symbols
    if unresolved_functions or unresolved_derivatives or unexpected_symbols:
        raise ValueError(
            "Unresolved symbolic content after substitution: "
            f"functions={sorted(map(str, unresolved_functions))}, "
            f"derivatives={sorted(map(str, unresolved_derivatives))}, "
            f"symbols={sorted(map(str, unexpected_symbols))}"
        )


def generate_cpp_dynamics(equations_dict, output_dir='generated'):
    """
    Generate C++ code for quadruped dynamics based on symbolic equations.
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)
    
    # Extract symbolic equations
    M = equations_dict['M']
    C = equations_dict['C']
    G = equations_dict['G']
    # Runtime model intentionally omits the explicit inertia-rate force.
    # The pickle may retain D_exact for approximation-error analysis, but the
    # generated implementation always exports D = 0.
    D = sp.zeros(M.rows, M.cols)
    n = validate_equations(M, C, G, D)
    
    # Get symbols and functions
    symbols = equations_dict['symbols']
    functions = equations_dict['functions']
    t = symbols['t']
    
    # Define constant substitutions (consistent with numerical_cal.py)
    constants = {
        symbols['m_b']: RobotConfig.BASE_WEIGHT,
        symbols['m_l']: RobotConfig.LEG_WEIGHT,
        symbols['I_bxx']: RobotConfig.BODY_I_XX,
        symbols['I_byy']: RobotConfig.BODY_I_YY,
        symbols['g']: RobotConfig.GRAVITY,
        symbols['X_offset']: RobotConfig.X_OFFSET,
        symbols['Y_offset']: RobotConfig.Y_OFFSET
    }
    
    # Create substitutions for time-dependent functions -> simple symbols
    # Define simple symbols for all state variables
    x_sym, z_sym, phi_sym, psi_sym = sp.symbols('x z phi psi', real=True)
    beta_lf_sym, Rm_lf_sym = sp.symbols('beta_lf Rm_lf', real=True)
    beta_rf_sym, Rm_rf_sym = sp.symbols('beta_rf Rm_rf', real=True)
    beta_rh_sym, Rm_rh_sym = sp.symbols('beta_rh Rm_rh', real=True)
    beta_lh_sym, Rm_lh_sym = sp.symbols('beta_lh Rm_lh', real=True)
    
    # Velocities
    dx_sym, dz_sym, dphi_sym, dpsi_sym = sp.symbols('dx dz dphi dpsi', real=True)
    dbeta_lf_sym, dRm_lf_sym = sp.symbols('dbeta_lf dRm_lf', real=True)
    dbeta_rf_sym, dRm_rf_sym = sp.symbols('dbeta_rf dRm_rf', real=True)
    dbeta_rh_sym, dRm_rh_sym = sp.symbols('dbeta_rh dRm_rh', real=True)
    dbeta_lh_sym, dRm_lh_sym = sp.symbols('dbeta_lh dRm_lh', real=True)
    
    # Inertias as symbols (not functions)
    I_c_lf_sym, I_c_rf_sym, I_c_rh_sym, I_c_lh_sym = sp.symbols('I_c_lf I_c_rf I_c_rh I_c_lh', real=True, positive=True)
    
    # Substitution mapping
    func_to_sym = {
        functions['x']: x_sym,
        functions['z']: z_sym,
        functions['phi']: phi_sym,
        functions['psi']: psi_sym,
        functions['beta_lf']: beta_lf_sym,
        functions['Rm_lf']: Rm_lf_sym,
        functions['beta_rf']: beta_rf_sym,
        functions['Rm_rf']: Rm_rf_sym,
        functions['beta_rh']: beta_rh_sym,
        functions['Rm_rh']: Rm_rh_sym,
        functions['beta_lh']: beta_lh_sym,
        functions['Rm_lh']: Rm_lh_sym,
        functions['I_c_lf']: I_c_lf_sym,
        functions['I_c_rf']: I_c_rf_sym,
        functions['I_c_rh']: I_c_rh_sym,
        functions['I_c_lh']: I_c_lh_sym,
        # Derivatives (velocities)
        sp.Derivative(functions['x'], t): dx_sym,
        sp.Derivative(functions['z'], t): dz_sym,
        sp.Derivative(functions['phi'], t): dphi_sym,
        sp.Derivative(functions['psi'], t): dpsi_sym,
        sp.Derivative(functions['beta_lf'], t): dbeta_lf_sym,
        sp.Derivative(functions['Rm_lf'], t): dRm_lf_sym,
        sp.Derivative(functions['beta_rf'], t): dbeta_rf_sym,
        sp.Derivative(functions['Rm_rf'], t): dRm_rf_sym,
        sp.Derivative(functions['beta_rh'], t): dbeta_rh_sym,
        sp.Derivative(functions['Rm_rh'], t): dRm_rh_sym,
        sp.Derivative(functions['beta_lh'], t): dbeta_lh_sym,
        sp.Derivative(functions['Rm_lh'], t): dRm_lh_sym,
    }
    
    # Substitute constants first, then functions
    print("Substituting constants...")
    M_const = M.subs(constants)
    C_const = C.subs(constants)
    G_const = G.subs(constants)
    D_const = D.subs(constants)
    
    print("Substituting time-dependent functions with symbols...")
    M_const = M_const.subs(func_to_sym)
    C_const = C_const.subs(func_to_sym)
    G_const = G_const.subs(func_to_sym)
    D_const = D_const.subs(func_to_sym)

    allowed_symbols = {
        x_sym, z_sym, phi_sym, psi_sym,
        beta_lf_sym, Rm_lf_sym, beta_rf_sym, Rm_rf_sym,
        beta_rh_sym, Rm_rh_sym, beta_lh_sym, Rm_lh_sym,
        dx_sym, dz_sym, dphi_sym, dpsi_sym,
        dbeta_lf_sym, dRm_lf_sym, dbeta_rf_sym, dRm_rf_sym,
        dbeta_rh_sym, dRm_rh_sym, dbeta_lh_sym, dRm_lh_sym,
        I_c_lf_sym, I_c_rf_sym, I_c_rh_sym, I_c_lh_sym,
    }
    validate_substituted_expressions(
        list(M_const) + list(C_const) + list(G_const) + list(D_const),
        allowed_symbols,
    )
    
    # CSE optimization
    print("Performing CSE optimization...")
    
    # Combine all expressions that need to be calculated
    all_exprs = []
    
    # M matrix (only compute upper triangular part, since it's symmetric)
    for i in range(n):
        for j in range(i, n):
            all_exprs.append(M_const[i, j])
    
    # C is generally not symmetric: generate every element.
    for i in range(n):
        for j in range(n):
            all_exprs.append(C_const[i, j])
    
    # G vector
    for i in range(n):
        all_exprs.append(G_const[i])
    
    # D matrix. It is sparse in the current model, but including every element
    # keeps the serialized equations as the single source of truth.
    for i in range(n):
        for j in range(n):
            all_exprs.append(D_const[i, j])
    
    # Execute CSE
    replacements, reduced = cse(all_exprs, symbols=sp.numbered_symbols("tmp"))
    
    print(f"Found {len(replacements)} common subexpressions")
    
    # Generate C++ code
    generate_header(output_dir, n)
    generate_source(output_dir, replacements, reduced, n)
    
    print(f"C++ code generated to {output_dir}/")

def generate_header(output_dir, n):
    """Generate .hpp header file"""
    header_code = """#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace quadruped_dynamics {

/**
 * @brief Compute dynamics matrices for 12-DOF quadruped robot
 * 
 * State vector definition (12-dimensional):
 * q = [x, z, phi, psi, beta_lf, Rm_lf, beta_rf, Rm_rf, 
 *      beta_rh, Rm_rh, beta_lh, Rm_lh]
 * 
 * @param q Generalized coordinates (12,)
 * @param q_dot Generalized velocities (12,)
 * @param I_c Leg inertias [I_lf, I_rf, I_rh, I_lh] (4,)
 * @param M Output: Mass matrix (12x12)
 * @param C Output: Coriolis matrix (12x12)
 * @param G Output: Gravity vector (12,)
 * @param D Output: Inertia rate matrix (12x12)
 */
void compute_dynamics(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& q_dot,
    const Eigen::Ref<const Eigen::VectorXd>& I_c,
    Eigen::Ref<Eigen::MatrixXd> M,
    Eigen::Ref<Eigen::MatrixXd> C,
    Eigen::Ref<Eigen::VectorXd> G,
    Eigen::Ref<Eigen::MatrixXd> D
);

/**
 * @brief Compute mass matrix only (if only M is needed)
 */
void compute_mass_matrix(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& I_c,
    Eigen::Ref<Eigen::MatrixXd> M
);

} // namespace quadruped_dynamics
"""
    
    with open(output_dir / 'quadruped_dynamics.hpp', 'w') as f:
        f.write(header_code)

def generate_source(output_dir, replacements, reduced, n):
    """Generate .cpp implementation file"""
    
    cpp_code = ['#include "quadruped_dynamics.hpp"',
                '',
                'namespace quadruped_dynamics {',
                '',
                'void compute_dynamics(',
                '    const Eigen::Ref<const Eigen::VectorXd>& q,',
                '    const Eigen::Ref<const Eigen::VectorXd>& q_dot,',
                '    const Eigen::Ref<const Eigen::VectorXd>& I_c,',
                '    Eigen::Ref<Eigen::MatrixXd> M,',
                '    Eigen::Ref<Eigen::MatrixXd> C,',
                '    Eigen::Ref<Eigen::VectorXd> G,',
                '    Eigen::Ref<Eigen::MatrixXd> D',
                ') {',
                '    // Extract state variables',
                '    const double x = q(0);',
                '    const double z = q(1);',
                '    const double phi = q(2);',
                '    const double psi = q(3);',
                '    const double beta_lf = q(4);',
                '    const double Rm_lf = q(5);',
                '    const double beta_rf = q(6);',
                '    const double Rm_rf = q(7);',
                '    const double beta_rh = q(8);',
                '    const double Rm_rh = q(9);',
                '    const double beta_lh = q(10);',
                '    const double Rm_lh = q(11);',
                '',
                '    const double dx = q_dot(0);',
                '    const double dz = q_dot(1);',
                '    const double dphi = q_dot(2);',
                '    const double dpsi = q_dot(3);',
                '    const double dbeta_lf = q_dot(4);',
                '    const double dRm_lf = q_dot(5);',
                '    const double dbeta_rf = q_dot(6);',
                '    const double dRm_rf = q_dot(7);',
                '    const double dbeta_rh = q_dot(8);',
                '    const double dRm_rh = q_dot(9);',
                '    const double dbeta_lh = q_dot(10);',
                '    const double dRm_lh = q_dot(11);',
                '',
                '    const double I_c_lf = I_c(0);',
                '    const double I_c_rf = I_c(1);',
                '    const double I_c_rh = I_c(2);',
                '    const double I_c_lh = I_c(3);',
                '',
                '    // Precompute trigonometric functions',
                '    const double sin_phi = std::sin(phi);',
                '    const double cos_phi = std::cos(phi);',
                '    const double sin_psi = std::sin(psi);',
                '    const double cos_psi = std::cos(psi);',
                '']
    
    # Add CSE temporary variables
    cpp_code.append('    // CSE optimized temporary variables')
    for temp_var, expr in replacements:
        cpp_str = sp.cxxcode(expr)
        cpp_code.append(f'    const double {temp_var} = {cpp_str};')
    
    cpp_code.append('')
    cpp_code.append('    // Compute mass matrix M (symmetric, only compute upper triangular)')
    cpp_code.append('    M.setZero();')
    
    # Generate M matrix (utilizing symmetry)
    idx = 0
    for i in range(n):
        for j in range(i, n):
            if reduced[idx] != 0:
                cpp_str = sp.cxxcode(reduced[idx])
                cpp_code.append(f'    M({i},{j}) = {cpp_str};')
                if i != j:
                    cpp_code.append(f'    M({j},{i}) = M({i},{j});')
            idx += 1
    
    cpp_code.append('')
    cpp_code.append('    // Compute Coriolis matrix C')
    cpp_code.append('    C.setZero();')
    
    # Generate C matrix
    for i in range(n):
        for j in range(n):
            if reduced[idx] != 0:
                cpp_str = sp.cxxcode(reduced[idx])
                cpp_code.append(f'    C({i},{j}) = {cpp_str};')
            idx += 1
    
    cpp_code.append('')
    cpp_code.append('    // Compute gravity vector G')
    for i in range(n):
        cpp_str = sp.cxxcode(reduced[idx])
        cpp_code.append(f'    G({i}) = {cpp_str};')
        idx += 1
    
    cpp_code.append('')
    cpp_code.append('    // Compute inertia rate matrix D')
    cpp_code.append('    D.setZero();')
    for i in range(n):
        for j in range(n):
            if reduced[idx] != 0:
                cpp_str = sp.cxxcode(reduced[idx])
                cpp_code.append(f'    D({i},{j}) = {cpp_str};')
            idx += 1
    
    cpp_code.extend([
        '}',
        '',
        'void compute_mass_matrix(',
        '    const Eigen::Ref<const Eigen::VectorXd>& q,',
        '    const Eigen::Ref<const Eigen::VectorXd>& I_c,',
        '    Eigen::Ref<Eigen::MatrixXd> M',
        ') {',
        '    Eigen::MatrixXd C_dummy(12, 12);',
        '    Eigen::VectorXd G_dummy(12);',
        '    Eigen::MatrixXd D_dummy(12, 12);',
        '    Eigen::VectorXd q_dot_dummy = Eigen::VectorXd::Zero(12);',
        '    compute_dynamics(q, q_dot_dummy, I_c, M, C_dummy, G_dummy, D_dummy);',
        '}',
        '',
        '} // namespace quadruped_dynamics'
    ])
    
    with open(output_dir / 'quadruped_dynamics.cpp', 'w') as f:
        f.write('\n'.join(cpp_code))

if __name__ == '__main__':
    repo_root = Path(__file__).resolve().parent.parent
    file_path = repo_root / 'saved_equations' / 'equations_dill.pkl'
    equations = load_equations(file_path)
    
    # Generate C++ code
    generate_cpp_dynamics(equations, output_dir=repo_root / 'cpp')
    
    print("Done! Please check the cpp/ directory")
