#ifndef _DistanceKinematicsIMUIMU_h_
#define _DistanceKinematicsIMUIMU_h_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  DistanceKinematicsIMU.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth                                    *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


//=============================================================================
// INCLUDES
//=============================================================================
#include "osimAnalysesDLL.h"
#ifdef SWIG
    #ifdef OSIMANALYSES_API
        #undef OSIMANALYSES_API
        #define OSIMANALYSES_API
    #endif
#endif

#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Simulation/OpenSense/IMU.h>

const int DistanceKinematicsIMUNAME_LENGTH = 256;
const int DistanceKinematicsIMUBUFFER_LENGTH = 2048;

//=============================================================================
//=============================================================================
namespace OpenSim { 

class IMU;
class Model;
class Storage;

/**
 * A class for recording the kinematics of a point on a body
 * of a model during a simulation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMANALYSES_API DistanceKinematicsIMU : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(DistanceKinematicsIMU, Analysis);

//=============================================================================
// DATA
//=============================================================================
public:
    static const int NAME_LENGTH;
    static const int BUFFER_LENGTH;
private:
    //char _buffer[DistanceKinematicsIMUBUFFER_LENGTH];
    //char _tmp[DistanceKinematicsIMUBUFFER_LENGTH];
    const OpenSim::IMU *_body;
    const OpenSim::IMU *_relativeToBody;
protected:
    // Properties
    PropertyStr _bodyNameProp;
    PropertyDblVec3 _pointProp;
    PropertyStr _pointNameProp;
    PropertyStr _relativeToBodyNameProp;

    // References
    std::string &_bodyName;
    SimTK::Vec3 &_point;
    std::string &_pointName;
    std::string &_relativeToBodyName;

    double *_kin;
    Storage *_pStore;
    Storage *_vStore;
    Storage *_aStore;

//=============================================================================
// METHODS
//=============================================================================
public:
    DistanceKinematicsIMU(Model *aModel=0);
    DistanceKinematicsIMU(const std::string &aFileName);
    DistanceKinematicsIMU(const DistanceKinematicsIMU &aObject);
    virtual ~DistanceKinematicsIMU();

private:
    void setNull();
    void setupProperties();
    void constructDescription();
    void constructColumnLabels();
    void allocateStorage();
    void deleteStorage();
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
#ifndef SWIG
    DistanceKinematicsIMU& operator=(const DistanceKinematicsIMU &aDistanceKinematicsIMU);
#endif
public:
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    // BODY
    void setBodyPoint(const std::string& aBody, const SimTK::Vec3& aPoint);
    void setBody(const OpenSim::IMU* aBody);
    void setRelativeToBody(const OpenSim::IMU* aBody);
    const OpenSim::IMU* getBody() const;
    const OpenSim::IMU* getRelativeToBody() const;
    // POINT
    void setPoint(const SimTK::Vec3& aPoint);
    void getPoint(SimTK::Vec3& rPoint);
    // POINT NAME
    void setPointName(const std::string &aName);
    const std::string &getPointName();
    // MODEL
    void setModel(Model& aModel) override;
    
    // STORAGE
    [[deprecated("this method no longer does anything")]]
    void setStorageCapacityIncrements(int) {}
    Storage* getAccelerationStorage();
    Storage* getVelocityStorage();
    Storage* getPositionStorage();

    //--------------------------------------------------------------------------
    // ANALYSIS
    //--------------------------------------------------------------------------

    int begin(const SimTK::State& s) override;
    int step(const SimTK::State& s, int setNumber) override;
    int end(const SimTK::State& s) override;
protected:
    virtual int
        record(const SimTK::State& s );

    //--------------------------------------------------------------------------
    // IO
    //--------------------------------------------------------------------------
public:
    int
        printResults(const std::string &aBaseName,const std::string &aDir="",
        double aDT=-1.0,const std::string &aExtension=".sto") override;

//=============================================================================
};  // END of class DistanceKinematicsIMU

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __DistanceKinematicsIMU_h__
