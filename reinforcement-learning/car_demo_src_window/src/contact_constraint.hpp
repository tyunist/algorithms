/*
 * FILE:
 *   contact_constraint.hpp
 *
 * PURPOSE:
 *   Subclass of Constraint representing a contact constraint
 *   (i.e. a physical contact between two Multibodies).
 *   
 * AUTHOR:
 *   Stephen Thompson <stephen@solarflare.org.uk>
 *
 * CREATED:
 *   10-Aug-2010
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

#ifndef CONTACT_CONSTRAINT_HPP
#define CONTACT_CONSTRAINT_HPP

#include "constraint.hpp"


// contact constraint

// pos1 is given in local coords of body1/link1 and assumed constant
// in that frame. Similarly for pos2.
// 
// normal points FROM body1 TOWARDS body2 (if bodies are separated),
// and is assumed to be fixed in the frame of body2/link2.
//
// If there is no body2 then pos2, normal are fixed in world space.

class ContactConstraint : public Constraint {
public:
    ContactConstraint(Multibody *bod1, Multibody *bod2,
                      int lnk1, int lnk2,
                      const Vector3r &contact_point_1_,
                      const Vector3r &contact_point_2_,
                      const Vector3r &normal_,
                      real_t friction_coeff,
                      real_t restitution_coeff);

    virtual void update(std::vector<real_t> &scratch_r,
                        std::vector<Vector3r> &scratch_v,
                        std::vector<Matrix3r> &scratch_m);

    void setContactPoints(const Vector3r &cp1,
                          const Vector3r &cp2,
                          const Vector3r &n)
    {
        contact_point_1 = cp1;
        contact_point_2 = cp2;
        normal = n;
    }

#ifdef CONTACT_FORCE_DEBUGGING
    virtual bool getDebugGfx(Vector3r &cp, Vector3r &n) const {
        cp = gfx_contact_point;
        n = gfx_normal;
        return true;
    }
#endif

    // Class contains Vector3r's so need this:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
protected:
    Vector3r contact_point_1;
    Vector3r contact_point_2;
    Vector3r normal;

private:
#ifdef CONTACT_FORCE_DEBUGGING
    Vector3r gfx_contact_point;
    Vector3r gfx_normal;
#endif
};

#endif
