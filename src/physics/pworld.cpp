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

#include "pworld.h"
#include <iostream>
#include <chrono>

PSurface::PSurface()
{
    callback = nullptr;
    usefdir1 = false;
    surface.mode = dContactApprox1;
    surface.mu = 0.5;
}
bool PSurface::isIt(dGeomID i1, dGeomID i2)
{
    return ((i1 == id1) && (i2 == id2)) || ((i1 == id2) && (i2 == id1));
}

void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
    
    ((PWorld *)data)->nearcallbacks_count++;
    std::cout << "----nearCallback-----" << ((PWorld *)data)->nearcallbacks_count << std::endl;
    SSLConfig::GeometriesIDs().printGeomID(*((int *)(dGeomGetData(o1))));
    std::cout <<" x ";
    SSLConfig::GeometriesIDs().printGeomID(*((int *)(dGeomGetData(o2))));
    std::cout << std::endl;

    ((PWorld *)data)->handleCollisions(o1, o2);
}

PWorld::PWorld(dReal dt, dReal gravity, int _robot_count)
{
    this->robot_count = _robot_count;
    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);
    this->world = dWorldCreate();
    // this->space = dHashSpaceCreate(nullptr);
    this->space = dSimpleSpaceCreate(nullptr);
    this->spaceChassis = dSimpleSpaceCreate(nullptr);
    this->spaceKicker= dSimpleSpaceCreate(nullptr);
    this->spaceWall= dSimpleSpaceCreate(nullptr);
    this->spaceWheel= dSimpleSpaceCreate(nullptr);
    this->contactgroup = dJointGroupCreate(0);
    dWorldSetGravity(this->world, 0, 0, -gravity);
    this->objects_count = 0;
    this->sur_matrix = nullptr;
    //dAllocateODEDataForThread(dAllocateMaskAll);
    this->delta_time = dt;
}

PWorld::~PWorld()
{
    dJointGroupDestroy(this->contactgroup);
    dSpaceDestroy(this->space);
    dWorldDestroy(this->world);
    dCloseODE();
}

void PWorld::setGravity(dReal gravity)
{
    dWorldSetGravity(world, 0, 0, -gravity);
}

void PWorld::handleCollisions(dGeomID o1, dGeomID o2)
{
    PSurface *sur;
    int j = sur_matrix[*((int *)(dGeomGetData(o1)))][*((int *)(dGeomGetData(o2)))];
    if (j != -1)
    {
        const int N = 10;
        dContact contact[N];
        int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
        if (n > 0)
        {
            sur = surfaces[j];
            sur->contactPos[0] = contact[0].geom.pos[0];
            sur->contactPos[1] = contact[0].geom.pos[1];
            sur->contactPos[2] = contact[0].geom.pos[2];
            sur->contactNormal[0] = contact[0].geom.normal[0];
            sur->contactNormal[1] = contact[0].geom.normal[1];
            sur->contactNormal[2] = contact[0].geom.normal[2];
            bool flag = true;
            if (sur->callback != nullptr)
                flag = sur->callback(o1, o2, sur, robot_count);
            if (flag)
                for (int i = 0; i < n; i++)
                {
                    contact[i].surface = sur->surface;
                    if (sur->usefdir1)
                    {
                        contact[i].fdir1[0] = sur->fdir1[0];
                        contact[i].fdir1[1] = sur->fdir1[1];
                        contact[i].fdir1[2] = sur->fdir1[2];
                        contact[i].fdir1[3] = sur->fdir1[3];
                    }
                    dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);

                    dJointAttach(c,
                                 dGeomGetBody(contact[i].geom.g1),
                                 dGeomGetBody(contact[i].geom.g2));
                }
        }
    }
}

int PWorld::addObject(PObject* o)
{
    int id = this->objects.count();
    o->id = id;
    if (o->world == nullptr)
        o->world = this->world;
    if (o->space == nullptr)
        o->space = this->space;
    o->init();
    dGeomSetData(o->geom, (void *)(&(o->id)));
    this->objects.append(o);
    this->ball = o;
    return id;
}

int PWorld::addBallObject(PObject* o)
{
    int id = this->objects.count();
    o->id = id;
    if (o->world == nullptr)
        o->world = this->world;
    if (o->space == nullptr)
        o->space = this->space;
    o->init();
    dGeomSetData(o->geom, (void *)(&(o->id)));
    this->objects.append(o);
    this->ball = o;
    return id;
}

int PWorld::addGroundObject(PObject* o)
{
    int id = this->objects.count();
    o->id = id;
    if (o->world == nullptr)
        o->world = this->world;
    if (o->space == nullptr)
        o->space = this->space;
    o->init();
    dGeomSetData(o->geom, (void *)(&(o->id)));
    this->objects.append(o);
    this->ground = o;
    return id;
}

int PWorld::addWallObject(PObject* o)
{
    int id = this->objects.count();
    o->id = id;
    if (o->world == nullptr)
        o->world = this->world;
    if (o->space == nullptr)
        o->space = this->spaceWall;
    o->init();
    dGeomSetData(o->geom, (void *)(&(o->id)));
    this->objects.append(o);
    return id;
}

int PWorld::addWheelObject(PObject* o)
{
    int id = this->objects.count();
    o->id = id;
    if (o->world == nullptr)
        o->world = this->world;
    if (o->space == nullptr)
        o->space = this->spaceWheel;
    o->init();
    dGeomSetData(o->geom, (void *)(&(o->id)));
    this->objects.append(o);
    return id;
}

int PWorld::addChassisObject(PObject* o)
{
    int id = this->objects.count();
    o->id = id;
    if (o->world == nullptr)
        o->world = this->world;
    if (o->space == nullptr)
        o->space = this->spaceChassis;
    o->init();
    dGeomSetData(o->geom, (void *)(&(o->id)));
    this->objects.append(o);
    return id;
}

int PWorld::addKickerObject(PObject* o)
{
    int id = this->objects.count();
    o->id = id;
    if (o->world == nullptr)
        o->world = this->world;
    if (o->space == nullptr)
        o->space = this->spaceKicker;
    o->init();
    dGeomSetData(o->geom, (void *)(&(o->id)));
    this->objects.append(o);
    return id;
}

void PWorld::initAllObjects()
{
    objects_count = this->objects.count();
    int c = objects_count;
    bool flag = false;
    if (sur_matrix != nullptr)
    {
        for (int i = 0; i < c; i++)
            delete sur_matrix[i];
        delete sur_matrix;
        flag = true;
    }
    sur_matrix = new int *[c];
    for (int i = 0; i < c; i++)
    {
        sur_matrix[i] = new int[c];
        for (int j = 0; j < c; j++)
            sur_matrix[i][j] = -1;
    }
    if (flag)
    {
        for (int i = 0; i < surfaces.count(); i++)
            sur_matrix[(*(int *)(dGeomGetData(surfaces[i]->id1)))][*((int *)(dGeomGetData(surfaces[i]->id2)))] =
                sur_matrix[(*(int *)(dGeomGetData(surfaces[i]->id2)))][*((int *)(dGeomGetData(surfaces[i]->id1)))] = i;
    }
}

PSurface *PWorld::createSurface(PObject *o1, PObject *o2)
{
    auto *s = new PSurface();
    s->id1 = o1->geom;
    s->id2 = o2->geom;
    this->surfaces.append(s);
    this->sur_matrix[o1->id][o2->id] =
        this->sur_matrix[o2->id][o1->id] = this->surfaces.count() - 1;
    return s;
}

PSurface* PWorld::createOneWaySurface(PObject* o1,PObject* o2)
{
    PSurface *s = new PSurface();
    s->id1 = o1->geom;
    s->id2 = o2->geom;
    this->surfaces.append(s);
    this->sur_matrix[o1->id][o2->id] = this->surfaces.count() - 1;
    return s;
}

PSurface *PWorld::findSurface(PObject *o1, PObject *o2)
{
    for (int i = 0; i < surfaces.count(); i++)
    {
        if (surfaces[i]->isIt(o1->geom, o2->geom))
            return (surfaces[i]);
    }
    return nullptr;
}

void PWorld::step(dReal dt, bool sync)
{
    try
    {
        this->nearcallbacks_count = 0;
        
        // Collide wheels with ground
        std::chrono::steady_clock::time_point begin1 = std::chrono::steady_clock::now();
        dSpaceCollide2((dGeomID)this->spaceWheel, this->ground->geom, this, &nearCallback);
        std::chrono::steady_clock::time_point end1 = std::chrono::steady_clock::now();

        std::cout << "time wheels with ground: " << std::chrono::duration_cast<std::chrono::milliseconds>(end1 - begin1).count() << std::endl;
        
        // Collide Ball with ground
        std::chrono::steady_clock::time_point begin2 = std::chrono::steady_clock::now();
        dSpaceCollide2(this->ball->geom, this->ground->geom, this, &nearCallback);
        std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
        std::cout << "time Ball with ground: " << std::chrono::duration_cast<std::chrono::milliseconds>(end2 - begin2).count() << std::endl;

        // // Collide ball with kicker

        std::chrono::steady_clock::time_point begin3 = std::chrono::steady_clock::now(); 
        dSpaceCollide2(this->ball->geom, (dGeomID)this->spaceKicker, this, &nearCallback);
        std::chrono::steady_clock::time_point end3 = std::chrono::steady_clock::now();
        std::cout << "time ball with kicker: " << std::chrono::duration_cast<std::chrono::milliseconds>(end3 - begin3).count() << std::endl;

        // Collide ball with chassis
        std::chrono::steady_clock::time_point begin4 = std::chrono::steady_clock::now(); 
        dSpaceCollide2(this->ball->geom, (dGeomID)this->spaceChassis, this, &nearCallback);
        std::chrono::steady_clock::time_point end4 = std::chrono::steady_clock::now();
        std::cout << "time ball with chassis: " << std::chrono::duration_cast<std::chrono::milliseconds>(end4 - begin4).count() << std::endl;

        // Collide ball with wall
        std::chrono::steady_clock::time_point begin5 = std::chrono::steady_clock::now(); 
        dSpaceCollide2(this->ball->geom, (dGeomID)this->spaceWall, this, &nearCallback);
        std::chrono::steady_clock::time_point end5 = std::chrono::steady_clock::now();
        std::cout << "time ball with wall: " << std::chrono::duration_cast<std::chrono::milliseconds>(end5 - begin5).count() << std::endl;

        // Collide chassis with wall
        std::chrono::steady_clock::time_point begin6 = std::chrono::steady_clock::now(); 
        dSpaceCollide2((dGeomID)this->spaceChassis, (dGeomID)this->spaceWall, this, &nearCallback);
        std::chrono::steady_clock::time_point end6 = std::chrono::steady_clock::now();
        std::cout << "time chassis with wall: " << std::chrono::duration_cast<std::chrono::milliseconds>(end6 - begin6).count() << std::endl;

        // Collide chassis with chassis
        std::chrono::steady_clock::time_point begin7 = std::chrono::steady_clock::now(); 
        dSpaceCollide(this->spaceChassis, this, &nearCallback);
        std::chrono::steady_clock::time_point end7 = std::chrono::steady_clock::now();
        std::cout << "time chassis with chassis: " << std::chrono::duration_cast<std::chrono::milliseconds>(end7 - begin7).count() << std::endl;

        std::cout << "near callback count: " << this->nearcallbacks_count << std::endl;
        dWorldSetQuickStepNumIterations(world, 20);
        if (sync)
            dWorldQuickStep(world, (dt < 0) ? delta_time : dt);
        else
            dWorldStep(world, (dt < 0) ? delta_time : dt);
        dJointGroupEmpty(contactgroup);
    }
    catch (...)
    {
        //qDebug() << "Some Error Happened;";
    }
}
