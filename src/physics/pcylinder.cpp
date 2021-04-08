/*
grSim - RoboCup Small Size Soccer Robots Simulator
Copyright (C) 2011, Parsian Robotic Center (eew.aut.ac.ir/~parsian/grsim)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "pcylinder.h"

PCylinder::PCylinder(dReal x, dReal y, dReal z, dReal radius, dReal length, dReal mass)
    : PObject(x, y, z, mass)
{
  this->m_radius = radius;
  this->m_length = length;
}

PCylinder::~PCylinder() = default;

void PCylinder::setMass(dReal mass)
{
  this->m_mass = mass;
  dMass m;
  dMassSetCylinderTotal(&m, this->m_mass, 1, this->m_radius, -this->m_length/2.);
  dBodySetMass(this->body, &m);
}

void PCylinder::init()
{
  this->m_z = this->m_z - this->m_length/4.; // offset body center of gravity
  this->body = dBodyCreate(this->world);
  initPosBody();
  this->m_z = this->m_z + this->m_length/4.; //offset
  setMass(this->m_mass);
  this->geom = dCreateCylinder(nullptr, this->m_radius, this->m_length);
  dGeomSetBody(this->geom, this->body);
  dGeomSetOffsetPosition(geom, 0.0f, 0.0f, -m_length/4.); // offset
  dSpaceAdd(this->space, this->geom);
}
