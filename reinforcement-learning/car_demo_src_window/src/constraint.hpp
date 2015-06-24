/*
 * FILE:
 *   constraint.hpp
 *
 * PURPOSE:
 *   Base class for rigid body constraints. Used for all types of
 *   constraints including contacts, joints, etc.
 *   
 * AUTHOR:
 *   Stephen Thompson <stephen@solarflare.org.uk>
 *
 * CREATED:
 *   04-Aug-2010
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

#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

#include "real.hpp"

#include <Eigen/StdVector>
#include <vector>

class Multibody;

#ifdef CONTACT_FORCE_DEBUGGING
#include <Eigen/Core>
#endif

class Constraint {
public:

    Constraint(bool uni, bool con,
               Multibody *bod1, Multibody *bod2,
               int lnk1, int lnk2,
               int num_rows,
               real_t friction_coeff_,      // only relevant if con == true
               real_t restitution_coeff_);  // only relevant if con == true

    virtual ~Constraint() { }
    
    // update(). recomputes constraint positions and jacobians, given 
    // current body positions. (This should NOT do anything velocity-dependent.)
    virtual void update(std::vector<real_t> &scratch_r,
                        std::vector<Vector3r> &scratch_v,
                        std::vector<Matrix3r> &scratch_m) = 0;
    
    // get the two bodies, and link numbers
    // (link num = -1 for the base, or 0 to nlinks-1 for the links)
    // note: no 'set' functions yet (these might involve reallocating the arrays).
    Multibody * getBody1() const { return body1; }
    Multibody * getBody2() const { return body2; }
    int getLink1() const { return link1; }
    int getLink2() const { return link2; }
    
    // Get the number of rows (each row is essentially a "sub-constraint")
    int getNumRows() const { return num_rows; }

    // unilateral ==> constraint is of form J' v >= 0, pos >= 0
    // bilateral ==> constraint is of form J' v = 0, pos = 0
    void setUnilateral(bool u) { unilateral = u; }
    bool isUnilateral() const { return unilateral; }

    // contact constraints have friction applied. (in this case,
    // number of rows must be 3. rows 1 and 2 are assumed to be the
    // friction rows.)
    bool isContact() const { return contact; }
    real_t getFrictionCoeff() const { return friction_coeff; }
    real_t getRestitutionCoeff() const { return restitution_coeff; }
    void setFrictionCoeff(real_t mu) { friction_coeff = mu; }
    void setRestitutionCoeff(real_t e) { restitution_coeff = e; }
    
    // current constraint position
    // constraint is pos >= 0 for unilateral, or pos = 0 for bilateral
    // NOTE: position ignored for friction rows.
    real_t getPosition(int row) const { return dat[pos_offset + row]; }
    void setPosition(int row, real_t pos) { dat[pos_offset + row] = pos; }

    // baumgarte parameters
    real_t getBaumgarteCoeff() const { return baumgarte_coeff; }
    real_t getBaumgarteEnvelope() const { return baumgarte_envelope; }
    void setBaumgarteParams(real_t b, real_t e) { baumgarte_coeff = b; baumgarte_envelope = e; }
    
    // constraint impulse. (used by the velocity correction / sequential impulses, for warm starting.)
    real_t getConstraintImpulse(int row) const { return dat[row]; }
    void addConstraintImpulse(int row, real_t f) { dat[row] += f; }
    void setConstraintImpulse(int row, real_t f) { dat[row] = f; }
    
    // jacobian blocks.
    // each of size 6 + num_links. (jacobian2 is null if no body2.)
    // format: 3 'omega' coefficients, 3 'v' coefficients, then the 'qdot' coefficients.
    real_t * jacobian1(int row) { return &dat[num_rows + row * jac_size_both]; }
    const real_t * jacobian1(int row) const { return &dat[num_rows + (row * jac_size_both)]; }
    real_t * jacobian2(int row) { return &dat[num_rows + (row * jac_size_both) + jac_size_1]; }
    const real_t * jacobian2(int row) const { return &dat[num_rows + (row * jac_size_both) + jac_size_1]; }

#ifdef CONTACT_FORCE_DEBUGGING
    virtual bool getDebugGfx(Vector3r &cp, Vector3r &n) const { return false; }
#endif
    
private:
    Multibody *body1;
    Multibody *body2;
    int link1;
    int link2;

    int num_rows;
    int jac_size_1;
    int jac_size_both;
    int pos_offset;

    // data block laid out as follows:
    // cached impulses. (one per row.)
    // jacobians. (interleaved, row1 body1 then row1 body2 then row2 body 1 etc)
    // positions. (one per row.)
    std::vector<real_t> dat;

    bool unilateral;
    bool contact;

    real_t friction_coeff;
    real_t restitution_coeff;
    real_t baumgarte_coeff;
    real_t baumgarte_envelope;
};

#endif
