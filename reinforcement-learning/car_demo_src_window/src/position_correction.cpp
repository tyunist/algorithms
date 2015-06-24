/*
 * FILE:
 *   position_correction.cpp
 *
 * AUTHOR:
 *   Stephen Thompson <stephen@solarflare.org.uk>
 *
 * CREATED:
 *   25-Jul-2011
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
#include "position_correction.hpp"

void PositionCorrection(Constraint **constraints,
                        int num_constraints,
                        std::vector<real_t> &scratch_r,
                        std::vector<Vector3r> &scratch_v,
                        std::vector<Matrix3r> &scratch_m)
{
    const int MAX_OUTER_ITERATIONS = 10;
    const int NUM_INNER_ITERATIONS = 10;

    const real_t MAXIMUM_IMPULSE = real_t(1e6);
    
    using std::vector;

    // Calculate the total number of constraint rows (excluding friction)
    // and the number of lambda entries needed.
    int total_constraint_rows = 0;
    int lambda_size = 0;
    for (int i = 0; i < num_constraints; ++i) {
        const int nrows = constraints[i]->isContact() ? 1 : constraints[i]->getNumRows();
        total_constraint_rows += nrows;
        lambda_size += nrows * (constraints[i]->getBody1()->getNumLinks() + 6);
        if (constraints[i]->getBody2()) lambda_size += nrows * (constraints[i]->getBody2()->getNumLinks() + 6);
    }
    vector<real_t> lambda(lambda_size);
    vector<std::pair<real_t*, real_t*> > lambda_index(total_constraint_rows);


    for (int outer_iteration = 0; outer_iteration < MAX_OUTER_ITERATIONS; ++outer_iteration) {

        // Update each constraint. Also check whether the constraints are satisfied.
        bool okay = true;
        for (int con = 0; con < num_constraints; ++con) {
            // Update
            constraints[con]->update(false, scratch_r, scratch_v, scratch_m);

            // Check position is valid
            const real_t tol = constraints[con]->getPositionTolerance();
            const int nrows = constraints[con]->isContact() ? 1 : constraints[con]->getNumRows();
            for (int num = 0; num < nrows; ++num) {
                const real_t curr_pos = constraints[con]->getPosition(num);
                if (constraints[con]->isUnilateral()) {
                    // Unilateral constraint: curr_pos must be >= 0
                    okay = okay && (curr_pos > -tol);
                } else {
                    // Bilateral constraint: curr_pos must be equal to zero
                    okay = okay && (curr_pos > -tol && curr_pos < tol);
                }
            }
        }

        // If all positions are within tolerance, there is no (more) position correction to do. Exit.
        if (okay) break;
        
        // Calculate lambda (change in position for one unit of constraint "impulse") for each constraint.
        // Also zero out initial delta_x's.
        real_t * lambda_ptr = &lambda[0];
        std::pair<real_t*, real_t*> * li = &lambda_index[0];
        for (int con = 0; con < num_constraints; ++con) {

            Multibody *bod1 = constraints[con]->getBody1();
            Multibody *bod2 = constraints[con]->getBody2();

            const int nrows = constraints[con]->isContact() ? 1 : constraints[con]->getNumRows();
            for (int num = 0; num < nrows; ++num, ++li) {
                
                li->first = lambda_ptr;
                bod1->calcAccelerationDeltas(constraints[con]->getLink1(),
                                             constraints[con]->force1(num),
                                             lambda_ptr, scratch_r, scratch_v);
                lambda_ptr += (6 + bod1->getNumLinks());
                
                if (bod2) {
                    li->second = lambda_ptr;
                    bod2->calcAccelerationDeltas(constraints[con]->getLink2(),
                                                 constraints[con]->force2(num),
                                                 lambda_ptr, scratch_r, scratch_v);
                    lambda_ptr += (6 + bod2->getNumLinks());
                }

                // Zero out impulse for this row
                constraints[con]->setPositionImpulse(num, 0);
            }

            // Zero out delta_x for this body
            bod1->clearDeltaX();
            if (bod2) bod2->clearDeltaX();
        }

        // Now do sequential "impulses" to calculate delta_x.
        // We are solving pos + J*delta_x >= 0 for unilateral constraints, or == 0 for bilateral ones.
        // This is done by applying "impulses" which create a delta_x of (Lambda*impulse + O(impulse^2)).
        for (int inner_iterations = 0; inner_iterations < NUM_INNER_ITERATIONS; ++inner_iterations) {

            // iterate through all constraints
            li = &lambda_index[0];
            for (int con = 0; con < num_constraints; ++con) {

                Multibody *bod1 = constraints[con]->getBody1();
                Multibody *bod2 = constraints[con]->getBody2();

                const int ndof1 = bod1->getNumLinks() + 6;
                const int ndof2 = bod2 ? bod2->getNumLinks() + 6 : 0;

                const int nrows = constraints[con]->isContact() ? 1 : constraints[con]->getNumRows();
                for (int num = 0; num < nrows; ++num, ++li) {

                    const real_t *jac1 = constraints[con]->jacobian1(num);
                    const real_t *jac2 = bod2 ? constraints[con]->jacobian2(num) : 0;
                    const real_t *lambda1 = li->first;
                    const real_t *lambda2 = bod2 ? li->second : 0;

                    real_t current_pos = constraints[con]->getPosition(num);
                    for (int i = 0; i < ndof1; ++i) current_pos += jac1[i] * bod1->getDeltaX()[i];
                    for (int i = 0; i < ndof2; ++i) current_pos += jac2[i] * bod2->getDeltaX()[i];
                    
                    // TODO: we could precompute denom for each row
                    real_t denom = 0;
                    for (int i = 0; i < ndof1; ++i) {
                        denom += jac1[i] * lambda1[i];
                    }
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

                    real_t delta_impulse = -current_pos / denom;

                    if (constraints[con]->isUnilateral()) {
                        // clamp total impulse
                        const real_t old_impulse = constraints[con]->getPositionImpulse(num);
                        if (delta_impulse < -old_impulse) delta_impulse = -old_impulse;
                    }

                    // limit the maximum size of the impulse
                    // (this is needed because e.g. a fixed base body might be touching a fixed object. since neither
                    // object can move, denom will be zero and a NaN impulse will be generated.)
                    if (_isnan(delta_impulse)) delta_impulse = 0;
                    if (delta_impulse > MAXIMUM_IMPULSE) delta_impulse = MAXIMUM_IMPULSE;
                    if (delta_impulse < -MAXIMUM_IMPULSE) delta_impulse = -MAXIMUM_IMPULSE;

                    // apply the impulse
                    bod1->addDeltaX(lambda1, delta_impulse);
                    if (bod2) bod2->addDeltaX(lambda2, delta_impulse);
                    constraints[con]->addPositionImpulse(num, delta_impulse);

                }  // loop over rows
            } // loop over constraints
        }  // loop over inner iterations

        // now we have to apply the delta_x to each body.
        for (int con = 0; con < num_constraints; ++con) {
            Multibody *bod1 = constraints[con]->getBody1();
            Multibody *bod2 = constraints[con]->getBody2();

            bod1->applyDeltaX();  // idempotent
            if (bod2) bod2->applyDeltaX();  // idempotent
        }

    }  // loop over outer iterations.                                             
}
