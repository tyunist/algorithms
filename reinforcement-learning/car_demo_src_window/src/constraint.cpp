/*
 * FILE:
 *   constraint.cpp
 *
 * AUTHOR:
 *   Stephen Thompson <stephen@solarflare.org.uk>
 *
 * CREATED:
 *   09-Aug-2010
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

#include <limits>

namespace {
    const real_t DEFAULT_BAUMGARTE_COEFF = real_t(0.02);
    const real_t DEFAULT_BAUMGARTE_ENVELOPE = real_t(0.01);
}

Constraint::Constraint(bool uni, bool con, Multibody *bod1, Multibody *bod2, int lnk1, int lnk2, int nrow,
                       real_t fric_coeff, real_t rest_coeff)
    : body1(bod1), body2(bod2), link1(lnk1), link2(lnk2), 
      num_rows(nrow),
      jac_size_1(6 + bod1->getNumLinks()),
      jac_size_both(jac_size_1 + (bod2 ? 6 + bod2->getNumLinks() : 0)),
      pos_offset((1 + jac_size_both)*num_rows),
      dat((2 + jac_size_both) * num_rows),
      unilateral(uni), contact(con),
      friction_coeff(fric_coeff),
      restitution_coeff(rest_coeff),
      baumgarte_coeff(DEFAULT_BAUMGARTE_COEFF),
      baumgarte_envelope(DEFAULT_BAUMGARTE_ENVELOPE)
{
    // note: dat is initialized to zero by the vector ctor.
}
