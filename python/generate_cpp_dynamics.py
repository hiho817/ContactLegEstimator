import dill
import sympy as sp
from sympy import cse
from pathlib import Path
import numpy as np
from Config import RobotConfig

def load_equations(filepath):
    """Load equations from .dill file"""
    with open(filepath, 'rb') as f:
        return dill.load(f)

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
    D = equations_dict['D']
    
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
    
    # Inertia derivatives (set to zero for now, or keep as symbols)
    dI_c_lf_sym, dI_c_rf_sym, dI_c_rh_sym, dI_c_lh_sym = sp.symbols('dI_c_lf dI_c_rf dI_c_rh dI_c_lh', real=True)
    
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
        # Inertia derivatives
        sp.Derivative(functions['I_c_lf'], t): dI_c_lf_sym,
        sp.Derivative(functions['I_c_rf'], t): dI_c_rf_sym,
        sp.Derivative(functions['I_c_rh'], t): dI_c_rh_sym,
        sp.Derivative(functions['I_c_lh'], t): dI_c_lh_sym
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
    
    # CSE optimization
    print("Performing CSE optimization...")
    
    # Combine all expressions that need to be calculated
    all_exprs = []
    
    # M matrix (only compute upper triangular part, since it's symmetric)
    for i in range(12):
        for j in range(i, 12):
            all_exprs.append(M_const[i, j])
    
    # C matrix (only compute upper triangular part)
    for i in range(12):
        for j in range(i, 12):
            all_exprs.append(C_const[i, j])
    
    # G vector
    for i in range(12):
        all_exprs.append(G_const[i])
    
    # D matrix (sparse, only compute non-zero elements)
    # D only has a few non-zero elements on the diagonal
    
    # Execute CSE
    replacements, reduced = cse(all_exprs, symbols=sp.numbered_symbols("tmp"))
    
    print(f"Found {len(replacements)} common subexpressions")
    
    # Generate C++ code
    generate_header(output_dir)
    generate_source(output_dir, replacements, reduced, M_const, C_const, G_const, D_const)
    
    print(f"✓ C++ code generated to {output_dir}/")

def generate_header(output_dir):
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

def generate_source(output_dir, replacements, reduced, M, C, G, D):
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
    for i in range(12):
        for j in range(i, 12):
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
    for i in range(12):
        for j in range(i, 12):
            if reduced[idx] != 0:
                cpp_str = sp.cxxcode(reduced[idx])
                cpp_code.append(f'    C({i},{j}) = {cpp_str};')
                if i != j:
                    cpp_code.append(f'    C({j},{i}) = C({i},{j});')
            idx += 1
    
    cpp_code.append('')
    cpp_code.append('    // Compute gravity vector G')
    for i in range(12):
        cpp_str = sp.cxxcode(reduced[idx])
        cpp_code.append(f'    G({i}) = {cpp_str};')
        idx += 1
    
    cpp_code.append('')
    cpp_code.append('    // Compute inertia rate matrix D (sparse)')
    cpp_code.append('    D.setZero();')
    cpp_code.append('    // D only has specific non-zero diagonal elements')
    cpp_code.append('    // This part requires dI_c/dt to be computed externally and passed in')
    
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
    # Load equations
    file_path = 'saved_equations/equations_dill.pkl'
    equations = load_equations(file_path)
    
    # Generate C++ code
    generate_cpp_dynamics(equations, output_dir='cpp')
    
    print("Done! Please check the cpp/ directory")