/*
 * FILE:
 *   sequential_impulses.cpp
 *
 * AUTHOR:
 *   Stephen Thompson <stephen@solarflare.org.uk>
 *
 * CREATED:
 *   05-Aug-2010
 *
 * COPYRIGHT:
 *   Copyright (C) Stephen Thompson, 2011.
 * 
 *   This file is part of Stephen Thompson's Car Physics Demo.
 *
 *   The Car Physics Demo is free software: you can redistribute it
 *   and/or modify it under the terms of the GNU General Public
 *   License as published by the Free Software Foundation, either
 *   version 3 of the License, or (at your option) any later version.
 *
 *   The Car Physics Demo is distributed in the hope that it will be
 *   useful, but WITHOUT ANY WARRANTY; without even the implied
 *   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *   See the GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with the Car Physics Demo. If not, see
 *   <http://www.gnu.org/licenses/>.
 *
 */

#include "constraint.hpp"
#include "multibody.hpp"
#include "sequential_impulses.hpp"

#include <cmath>
#include <vector>

#define USE_WARM_STARTING
//#define DUMP_COMPLEMENTARITY_PROBLEM

#ifdef DUMP_COMPLEMENTARITY_PROBLEM
#include <fstream>
#endif

void CalculateBVector(const Constraint * const *constraints,
                      int num_constraints,
                      real_t dt,
                      std::vector<real_t> &Bvector)
{
    const real_t MIN_BOUNCE_VEL = real_t(0.3);

    using std::vector;

    int total_constraint_rows = 0;
    for (int i = 0; i < num_constraints; ++i) {
        const int nrows = constraints[i]->getNumRows();
        total_constraint_rows += nrows;
    }
    Bvector.resize(total_constraint_rows);
    
    
    // Compute vector b in the condition
    // J*v + b >= 0  (unilateral constraint)
    // J*v + b == 0  (bilateral constraint)
    //
    // Usually b = 0
    // but if restitution is present, b = J * e * v_old
    // and if Baumgarte is present,   b = (B/dt) * pos
    //
    // b has 1 column, total_constraint_rows rows.
    //
    real_t *ptr = &Bvector[0];
    const real_t one_over_dt = real_t(1) / dt;

    for (int con = 0; con < num_constraints; ++con) {
        Multibody *bod1 = constraints[con]->getBody1();
        Multibody *bod2 = constraints[con]->getBody2();
        const real_t baumgarte_coeff = constraints[con]->getBaumgarteCoeff();
        
        for (int num = 0; num < constraints[con]->getNumRows(); ++num, ++ptr) {
            if (num == 0 || !constraints[con]->isContact()) {
                // Non-friction row

                // Baumgarte
                const real_t pos = constraints[con]->getPosition(num);
                real_t minus_vnew = pos * baumgarte_coeff * one_over_dt;
                if (constraints[con]->isUnilateral()) {
                    const real_t env = constraints[con]->getBaumgarteEnvelope();
                    if (pos > env) {
                        minus_vnew = (pos - env * (1 - baumgarte_coeff)) * one_over_dt;
                    }
                }

                // Restitution
                if (constraints[con]->isUnilateral()) {
                    const real_t env = constraints[con]->getBaumgarteEnvelope();

                    // Calculate J * v_old
                    const real_t *jac1 = constraints[con]->jacobian1(num);
                    const real_t *jac2 = bod2 ? constraints[con]->jacobian2(num) : 0;

                    real_t j_vold = 0;
                    for (int i = 0; i < 6 + bod1->getNumLinks(); ++i) j_vold += bod1->getVelocityVector()[i] * jac1[i];
                    if (bod2) {
                        for (int i = 0; i < 6 + bod2->getNumLinks(); ++i) j_vold += bod2->getVelocityVector()[i] * jac2[i];
                    }

                    // Apply restitution only if 
                    //  1. Incoming velocity is above MIN_BOUNCE_VEL
                    //  2. Incoming velocity is enough to take us to pos<0 within two timesteps.
                    if (j_vold < -MIN_BOUNCE_VEL && pos + j_vold * 2 * dt < 0) {
                        minus_vnew = constraints[con]->getRestitutionCoeff() * j_vold;
                    }
                }

                // set result
                *ptr = minus_vnew;

            } else {
                // Friction row
                *ptr = 0;
            }
        }
    }
}


void SequentialImpulses(Constraint **constraints,
                        int num_constraints,
                        real_t dt,
                        const std::vector<real_t> &Bvector,
                        std::vector<real_t> &scratch_r,
                        std::vector<Vector3r> &scratch_v)
{
    // We just do a fixed number of iterations.
    // TODO: Perhaps try stopping early if the velocity constraints have been met already.
    const int NUM_SEQUENTIAL_IMPULSE_ITERATIONS = 10;

    const real_t MAXIMUM_IMPULSE = real_t(1e6) * dt;
    
    using std::vector;

#ifdef DUMP_COMPLEMENTARITY_PROBLEM
    std::ofstream str("E:/CompProblem.txt");
    str << "Num_Constraints = " << num_constraints << "\n";
#endif

    // Calculate the total number of constraint rows, and the number of lambda entries needed
    int total_constraint_rows = 0;
    int lambda_size = 0;
    for (int i = 0; i < num_constraints; ++i) {
        const int nrows = constraints[i]->getNumRows();
        total_constraint_rows += nrows;
        lambda_size += nrows * (constraints[i]->getBody1()->getNumLinks() + 7);
        if (constraints[i]->getBody2()) lambda_size += nrows * (constraints[i]->getBody2()->getNumLinks() + 6);
    }
    vector<real_t> lambda(lambda_size);
    vector<std::pair<real_t*, real_t*> > lambda_index(total_constraint_rows);


    // Lambda array holds "change in velocity for one unit of constraint impulse" for each constraint
    // It also holds "1/denom" for each constraint (used below)
    real_t *lambda_ptr = &lambda[0];
    std::pair<real_t*,real_t*> * li = &lambda_index[0];
    for (int con = 0; con < num_constraints; ++con) {
        for (int num = 0; num < constraints[con]->getNumRows(); ++num, ++li) {

            real_t *denom_ptr = lambda_ptr;
            ++lambda_ptr;

            Multibody *bod1 = constraints[con]->getBody1();
            Multibody *bod2 = constraints[con]->getBody2();

#ifdef DUMP_COMPLEMENTARITY_PROBLEM
            str << "con=" << con << ", num=" << num << ", bod1=" << bod1 << ", bod2=" << bod2 << "\n";
            str << "link1=" << constraints[con]->getLink1() << ", link2=" << constraints[con]->getLink2() << "\n";
            str << "initial_impulse = " << constraints[con]->getConstraintImpulse(num) << "\n";

            str << "normal:\n";
            for (int i = 0; i < 3; ++i) {
                str << constraints[con]->normal[i] << "\n";
            }

            str << "bod1 jacobian:\n";
            for (int i = 0; i < 6 + bod1->getNumLinks(); ++i) {
                str << constraints[con]->jacobian1(num)[i] << "\n";
            }

            if (bod2) {
                str << "bod2 jacobian:\n";
                for (int i = 0; i < 6 + bod2->getNumLinks(); ++i) {
                    str << constraints[con]->jacobian2(num)[i] << "\n";
                }
            }

            str << "bod1 lambda:\n";
#endif

            li->first = lambda_ptr;
            bod1->calcAccelerationDeltas(constraints[con]->jacobian1(num),
                                         lambda_ptr,
                                         scratch_r, scratch_v);
#ifdef DUMP_COMPLEMENTARITY_PROBLEM
            for (int i = 0; i < 6 + bod1->getNumLinks(); ++i) {
                str << lambda_ptr[i] << "\n";
            }
            str << "bod2 lambda:\n";
#endif

            lambda_ptr += (6 + bod1->getNumLinks());
            
            if (bod2) {
                li->second = lambda_ptr;
                bod2->calcAccelerationDeltas(constraints[con]->jacobian2(num),
                                             lambda_ptr,
                                             scratch_r, scratch_v);

#ifdef DUMP_COMPLEMENTARITY_PROBLEM
                for (int i = 0; i < 6 + bod2->getNumLinks(); ++i) {
                    str << lambda_ptr[i] << "\n";
                }
#endif

                lambda_ptr += (6 + bod2->getNumLinks());
            }

            // Compute "denom"
            const real_t *jac1 = constraints[con]->jacobian1(num);
            const real_t *lambda1 = li->first;
            const int ndof1 = 6 + bod1->getNumLinks();
            real_t denom = 0;
            for (int i = 0; i < ndof1; ++i) {
                denom += jac1[i] * lambda1[i];
            }
            if (bod2) {
                const real_t *jac2 = constraints[con]->jacobian2(num);
                const real_t *lambda2 = li->second;
                const int ndof2 = 6 + bod2->getNumLinks();
                for (int i = 0; i < ndof2; ++i) {
                    denom += jac2[i] * lambda2[i];
                }
                if (bod1 == bod2) {
                    // ndof1 == ndof2 in this case
                    for (int i = 0; i < ndof1; ++i) {
                        denom += jac2[i] * lambda1[i];
                        denom += jac1[i] * lambda2[i];
                    }
                }
            }
            // store 1/denom
            *denom_ptr = real_t(1) / denom;

        }
    }

    // Apply initial impulses (warm starting)
    lambda_ptr = &lambda[0];
    for (int con = 0; con < num_constraints; ++con) {
        for (int num = 0; num < constraints[con]->getNumRows(); ++num) {
            Multibody *bod1 = constraints[con]->getBody1();
            Multibody *bod2 = constraints[con]->getBody2();
            ++lambda_ptr;  // skip over stored "1/denom" value
#ifdef USE_WARM_STARTING
            const real_t initial_impulse = constraints[con]->getConstraintImpulse(num);
            bod1->applyDeltaVee(lambda_ptr, initial_impulse);
            lambda_ptr += 6 + bod1->getNumLinks();
            if (bod2) {
                bod2->applyDeltaVee(lambda_ptr, initial_impulse);
                lambda_ptr += 6 + bod2->getNumLinks();
            }
#else
            constraints[con]->setConstraintImpulse(num, 0);
#endif
        }
    }


#ifdef DUMP_COMPLEMENTARITY_PROBLEM

    str << "\nAmatrix:\n";

    for (int jac_con = 0; jac_con < num_constraints; ++jac_con) {

        Multibody * jac_bod1 = constraints[jac_con]->getBody1();
        Multibody * jac_bod2 = constraints[jac_con]->getBody2();
        const int jac_ndof1 = jac_bod1->getNumLinks() + 6;
        const int jac_ndof2 = jac_bod2 ? jac_bod2->getNumLinks() + 6 : 0;
        
        const int jac_num_rows = constraints[jac_con]->getNumRows();
        
        for (int jac_num = 0; jac_num < jac_num_rows; ++jac_num) {

            const real_t *jac1 = constraints[jac_con]->jacobian1(jac_num);
            const real_t *jac2 = jac_bod2 ? constraints[jac_con]->jacobian2(jac_num) : 0;

            li = &lambda_index[0];
            
            for (int lam_con = 0; lam_con < num_constraints; ++lam_con) {

                Multibody * lam_bod1 = constraints[lam_con]->getBody1();
                Multibody * lam_bod2 = constraints[lam_con]->getBody2();
                
                const int lam_num_rows = constraints[lam_con]->getNumRows();
                
                for (int lam_num = 0; lam_num < lam_num_rows; ++lam_num, ++li) {

                    const real_t * lambda1 = li->first;
                    const real_t * lambda2 = lam_bod2 ? li->second : 0;

                    // Work out Jac * Lam

                    real_t result = 0;
                    
                    if (jac_bod1 == lam_bod1) {
                        for (int i = 0; i < jac_ndof1; ++i) {
                            result += jac1[i] * lambda1[i];
                        }
                    }

                    if (jac_bod2 == lam_bod1) {
                        for (int i = 0; i < jac_ndof2; ++i) {
                            result += jac2[i] * lambda1[i];
                        }
                    }

                    if (jac_bod1 == lam_bod2) {
                        for (int i = 0; i < jac_ndof1; ++i) {
                            result += jac1[i] * lambda2[i];
                        }
                    }

                    if (jac_bod2 == lam_bod2) {
                        for (int i = 0; i < jac_ndof2; ++i) {
                            result += jac2[i] * lambda2[i];
                        }
                    }

                    str << result << ", ";
                }
            }

            str << "\n";
        }
    }

    str << "\nBvector:\n";
    for (int i = 0; i < total_constraint_rows; ++i) {
        str << Bvector[i] << "\n";
    }

    str << "\nInitial vel at each constraint:\n";
    for (int con = 0; con < num_constraints; ++con) {

        Multibody * bod1 = constraints[con]->getBody1();
        Multibody * bod2 = constraints[con]->getBody2();
        const int ndof1 = bod1->getNumLinks() + 6;
        const int ndof2 = bod2 ? bod2->getNumLinks() + 6 : 0;
        
        const int num_rows = constraints[con]->getNumRows();
        
        for (int num = 0; num < num_rows; ++num) {

            const real_t *jac1 = constraints[con]->jacobian1(num);
            const real_t *jac2 = bod2 ? constraints[con]->jacobian2(num) : 0;

            real_t result = 0;
            for (int i = 0; i < ndof1; ++i) result += jac1[i] * bod1->getVelocityVector()[i];
            for (int i = 0; i < ndof2; ++i) result += jac2[i] * bod2->getVelocityVector()[i];

            str << result << "\n";
        }
    }            
#endif

    // Now we can do our sequential impulses.
    // We are solving J*v + b >= 0 (or J*v + b = 0) by applying impulses which add Lambda * impulse to v.
    // (subject to suitable constraints on the total impulse.)
    for (int iterations = 0; iterations < NUM_SEQUENTIAL_IMPULSE_ITERATIONS; ++iterations) {

        // iterate through all constraints
        li = &lambda_index[0];
        const real_t *bptr = &Bvector[0];
        for (int con = 0; con < num_constraints; ++con) {

            Multibody *bod1 = constraints[con]->getBody1();
            Multibody *bod2 = constraints[con]->getBody2();

            const int ndof1 = bod1->getNumLinks() + 6;
            const int ndof2 = bod2 ? bod2->getNumLinks() + 6 : 0;

            for (int num = 0; num < constraints[con]->getNumRows(); ++num, ++li) {

                const real_t *jac1 = constraints[con]->jacobian1(num);
                const real_t *jac2 = bod2 ? constraints[con]->jacobian2(num) : 0;
                const real_t *lambda1 = li->first;
                const real_t *lambda2 = bod2 ? li->second : 0;

                real_t delta_impulse = - *bptr++;
                for (int i = 0; i < ndof1; ++i) delta_impulse -= jac1[i] * bod1->getVelocityVector()[i];
                for (int i = 0; i < ndof2; ++i) delta_impulse -= jac2[i] * bod2->getVelocityVector()[i];

                const real_t one_over_denom = *(lambda1 - 1);
                delta_impulse *= one_over_denom;
                const real_t old_impulse = constraints[con]->getConstraintImpulse(num);
                
                if (num > 0 && constraints[con]->isContact()) {
                    // This is a friction impulse so limit the total impulse to
                    // be between +/- mu times the normal reaction
                    const real_t max_friction = 
                        constraints[con]->getFrictionCoeff() * constraints[con]->getConstraintImpulse(0);
                    if (delta_impulse < -max_friction - old_impulse) {
                        delta_impulse = -max_friction - old_impulse;
                    } else if (delta_impulse > max_friction - old_impulse) {
                        delta_impulse = max_friction - old_impulse;
                    }
                } else if (constraints[con]->isUnilateral()) {
                    // This is a unilateral constraint so limit the total impulse to be >= 0
                    if (delta_impulse < -old_impulse) delta_impulse = -old_impulse;
                }

                // limit the maximum size of the impulse
                // (this is needed because e.g. a fixed base body might be touching a fixed object. neither
                // object can move, so denom will be zero, and a NaN impulse will be generated.)
                if (_isnan(delta_impulse)) delta_impulse = 0;
                if (delta_impulse + old_impulse > MAXIMUM_IMPULSE) delta_impulse = MAXIMUM_IMPULSE - old_impulse;
                if (delta_impulse + old_impulse < -MAXIMUM_IMPULSE) delta_impulse = -MAXIMUM_IMPULSE - old_impulse;

                // apply the impulse
                bod1->applyDeltaVee(lambda1, delta_impulse);
                if (bod2) bod2->applyDeltaVee(lambda2, delta_impulse);
                constraints[con]->addConstraintImpulse(num, delta_impulse);

            }   // loop over rows
        }     // loop over constraints
    }   // loop over iterations


#ifdef DUMP_COMPLEMENTARITY_PROBLEM
    str.close();
#endif

    // All done!
}
