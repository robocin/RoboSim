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

#include "pobject.h"

PObject::PObject(dReal x, dReal y, dReal z, dReal mass)
{
    this->geom = nullptr;
    this->body = nullptr;
    this->world = nullptr;
    this->space = nullptr;
    this->m_x = x;
    this->m_y = y;
    this->m_z = z;
    this->m_mass = mass;
    this->isQSet = false;
    this->tag = 0;
}

PObject::~PObject()
{
    if (this->geom != nullptr)
        dGeomDestroy(this->geom);
    if (this->body != nullptr)
        dBodyDestroy(this->body);
}

void PObject::setRotation(dReal x_axis, dReal y_axis, dReal z_axis, dReal ang)
{
    dQFromAxisAndAngle(this->q, x_axis, y_axis, z_axis, ang);
    this->isQSet = true;
}

void PObject::setBodyPosition(dReal x, dReal y, dReal z, bool local)
{
    if (!local)
        dBodySetPosition(this->body, x, y, z);
    else
    {
        this->local_Pos[0] = x;
        this->local_Pos[1] = y;
        this->local_Pos[2] = z;
    }
}

void PObject::setBodyRotation(dReal x_axis, dReal y_axis, dReal z_axis, dReal ang, bool local)
{
    if (!local)
    {
        dQFromAxisAndAngle(this->q, x_axis, y_axis, z_axis, ang);
        dBodySetQuaternion(this->body, this->q);
    }
    else
    {
        dRFromAxisAndAngle(this->local_Rot, x_axis, y_axis, z_axis, ang);
    }
}

void PObject::getBodyPosition(dReal &x, dReal &y, dReal &z, bool local)
{
    if (local)
    {
        x = this->local_Pos[0];
        y = this->local_Pos[1];
        z = this->local_Pos[2];
        return;
    }
    const dReal *r = dBodyGetPosition(this->body);
    x = r[0];
    y = r[1];
    z = r[2];
}

void PObject::getBodyDirection(dReal &x, dReal &y, dReal &z)
{
    const dReal *r = dBodyGetRotation(this->body);
    dVector3 v = {1, 0, 0};
    dVector3 axis;
    dMultiply0(axis, r, v, 4, 3, 1);
    x = axis[0];
    y = axis[1];
    z = axis[2];
}

void PObject::getBodyDirection(dReal &x, dReal &y, dReal &z, dReal &k)
{
    const dReal *r = dBodyGetRotation(this->body);
    dVector3 v = {1, 0, 0};
    dVector3 axis;
    dMultiply0(axis, r, v, 4, 3, 1);
    x = axis[0];
    y = axis[1];
    z = axis[2];
    k = r[10];
}

void PObject::getBodyRotation(dMatrix3 r, bool local)
{
    if (local)
    {
        for (int k = 0; k < 12; k++)
            r[k] = this->local_Rot[k];
    }
    else
    {
        const dReal *rr = dBodyGetRotation(this->body);
        for (int k = 0; k < 12; k++)
            r[k] = rr[k];
    }
}

void PObject::initPosBody()
{
    dBodySetPosition(this->body, this->m_x, this->m_y, this->m_z);
    if (this->isQSet)
        dBodySetQuaternion(this->body, this->q);
}

void PObject::initPosGeom()
{
    dGeomSetPosition(this->geom, this->m_x, this->m_y, this->m_z);
    if (this->isQSet)
        dGeomSetQuaternion(this->geom, this->q);
}

void PObject::setMass(dReal mass)
{
    this->m_mass = mass;
}
