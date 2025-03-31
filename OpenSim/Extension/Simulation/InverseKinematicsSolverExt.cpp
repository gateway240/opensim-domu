/* -------------------------------------------------------------------------- *
 *                   OpenSim:  InverseKinematicsSolverExt.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Alex Beattie, Ajay Seth                                                       *
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

#include "InverseKinematicsSolverExt.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>

#include "DistancesReference.h"
#include "simbody/internal/AssemblyCondition_Markers.h"
#include "simbody/internal/AssemblyCondition_OrientationSensors.h"
#include "simbody/internal/AssemblyCondition_DistanceSensors.h"

using namespace std;
using namespace SimTK;


template< template<typename,typename> class Container, typename Separator >
Container<std::string,std::allocator<std::string> > split( const std::string& line, Separator sep ) {
    std::size_t pos = 0;
    std::size_t next = 0;
    Container<std::string,std::allocator<std::string> > fields;
    while ( next != std::string::npos ) {
        next = line.find_first_of( sep, pos );
        std::string field = next == std::string::npos ? line.substr(pos) : line.substr(pos,next-pos);
        fields.push_back(  field );
        pos = next + 1;
    }
    return fields;
}

namespace OpenSim {

//______________________________________________________________________________
/*
 * An implementation of the InverseKinematicsSolverExt 
 *
 * @param model to assemble
 */
InverseKinematicsSolverExt::InverseKinematicsSolverExt(const Model& model,
        std::shared_ptr<MarkersReference> markersReference,
        SimTK::Array_<CoordinateReference>& coordinateReferences,
        double constraintWeight) :
        InverseKinematicsSolverExt(model, markersReference, nullptr, nullptr,
            coordinateReferences, constraintWeight = SimTK::Infinity)
{}

InverseKinematicsSolverExt::InverseKinematicsSolverExt(const Model& model,
        std::shared_ptr<DistancesReference> distancesReference,
        SimTK::Array_<CoordinateReference>& coordinateReferences,
        double constraintWeight) :
        InverseKinematicsSolverExt(model, nullptr , nullptr,distancesReference, 
            coordinateReferences, constraintWeight = SimTK::Infinity)
{}

InverseKinematicsSolverExt::InverseKinematicsSolverExt(const Model& model,
    std::shared_ptr<MarkersReference> markersReference,
    std::shared_ptr<OrientationsReference> orientationsReference,
    std::shared_ptr<DistancesReference> distancesReference,
    SimTK::Array_<CoordinateReference>& coordinateReferences,
        double constraintWeight):
          AssemblySolver(model, coordinateReferences, constraintWeight), 
          _markersReference(markersReference), 
          _orientationsReference(orientationsReference),
          _distancesReference(distancesReference) {

    setAuthors("Ajay Seth, Ayman Habib, Alexander Beattie");
    
    if (_markersReference && _markersReference->getNumRefs() > 0) {
        // Do some consistency checking for markers
        const MarkerSet &modelMarkerSet = getModel().getMarkerSet();

        if (modelMarkerSet.getSize() < 1) {
            log_error("InverseKinematicsSolverExt: Model has no markers!");
            throw Exception("InverseKinematicsSolverExt: Model has no markers!");
        }
        const SimTK::Array_<std::string>& markerNames
            = _markersReference->getNames(); // size and content as in trc file

        if (markerNames.size() < 1) {
            log_error("InverseKinematicsSolverExt: No markers available from data provided.");
            throw Exception("InverseKinematicsSolverExt: No markers available from data provided.");
        }
        int index = 0, cnt = 0;
        for (unsigned int i = 0; i < markerNames.size(); i++) {
            // Check if we have this marker in the model, else ignore it
            index = modelMarkerSet.getIndex(markerNames[i], index);
            if (index >= 0) //found corresponding model
                cnt++;
        }

        if (cnt < 1) {
            log_error("InverseKinematicsSolverExt: Marker data does not correspond to any model markers.");
            throw Exception("InverseKinematicsSolverExt: Marker data does not correspond to any model markers.");
        }
        if (cnt < 4)
            log_warn("WARNING: InverseKinematicsSolverExt found only {} markers to track.", cnt);

    }
    if (_orientationsReference && _orientationsReference->getNumRefs() > 0) {
        const SimTK::Array_<std::string>& sensorNames =
                _orientationsReference
                        ->getNames(); // size and content as in orientations file

        if (sensorNames.size() < 1) {
            log_error("InverseKinematicsSolverExt: No orientation data is available from provided "
                      "source.");
            throw Exception("InverseKinematicsSolverExt: No orientation data is available "
                            "from the provided source.");
        }
        int cnt = 0;
        const auto onFrames = getModel().getComponentList<PhysicalFrame>();

        for (const auto& modelFrame : onFrames) {
            const std::string& modelFrameName = modelFrame.getName();
            auto found = std::find(
                    sensorNames.begin(), sensorNames.end(), modelFrameName);
            if (found != sensorNames.end()) { cnt++; }
        }

        if (cnt < 1) {
            log_error("InverseKinematicsSolverExt: Orientation data does not "
                      "correspond to any model frames.");
            throw Exception("InverseKinematicsSolverExt: Orientation data does not "
                            "correspond to any model frames.");
        }
        log_info("InverseKinematicsSolverExt found {} model frames to track.",
                    cnt);

    }

    if (_distancesReference && _distancesReference->getNumRefs() > 0) {
        const SimTK::Array_<std::string>& sensorNames =
                _distancesReference
                        ->getNames(); // size and content as in distances file

        if (sensorNames.size() < 1) {
            log_error("InverseKinematicsSolverExt: No distance data is available from provided "
                      "source.");
            throw Exception("InverseKinematicsSolverExt: No distance data is available "
                            "from the provided source.");
        }
        int cnt = 0;
        const auto onFrames = getModel().getComponentList<PhysicalFrame>();

        for (const auto& modelFrame : onFrames) {
            const std::string& modelFrameName = modelFrame.getName();
            std::cout << modelFrameName << std::endl;
            auto found = std::find_if(sensorNames.begin(), sensorNames.end(), 
                [&modelFrameName](std::string const& elem) {
                auto v = split<std::vector>(elem, '-');
                return v[0] == modelFrameName || v[1] == modelFrameName;
            });
            if (found != sensorNames.end()) { cnt++; }
        }

        if (cnt < 1) {
            log_error("InverseKinematicsSolverExt: Distance data does not "
                      "correspond to any model frames.");
            throw Exception("InverseKinematicsSolverExt: Distance data does not "
                            "correspond to any model frames.");
        }
        log_info("InverseKinematicsSolverExt found {} model frames to track.",
                    cnt);

    }
}

int InverseKinematicsSolverExt::getNumMarkersInUse() const
{
    return _markerAssemblyCondition->getNumMarkers();
}

/* Change the weighting of a marker to take effect when assemble or track is called next. 
   Update a marker's weight by name. */
void InverseKinematicsSolverExt::updateMarkerWeight(const std::string& markerName, double value)
{
    const Array_<std::string> &names = _markersReference->getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), markerName);
    int index = (int)std::distance(names.begin(), p);
    updateMarkerWeight(index, value);
}

/* Update a marker's weight by its index. */
void InverseKinematicsSolverExt::updateMarkerWeight(int markerIndex, double value)
{
    if(markerIndex >=0 && markerIndex < _markersReference->getMarkerWeightSet().getSize()){
        // update the solver's copy of the reference
        _markersReference->updMarkerWeightSet()[markerIndex].setWeight(value);
        _markerAssemblyCondition->changeMarkerWeight(SimTK::Markers::MarkerIx(markerIndex), value);
    }
    else
        throw Exception("InverseKinematicsSolverExt::updateMarkerWeight: invalid markerIndex.");
}

/* Update all markers weights by order in the markersReference passed in to
   construct the solver. */
void InverseKinematicsSolverExt::updateMarkerWeights(const SimTK::Array_<double> &weights)
{
    if(static_cast<unsigned>(_markersReference->getMarkerWeightSet().getSize()) 
       == weights.size()){
        for(unsigned int i=0; i<weights.size(); i++){
            _markersReference->updMarkerWeightSet()[i].setWeight(weights[i]);
            _markerAssemblyCondition->changeMarkerWeight(SimTK::Markers::MarkerIx(i), weights[i]);
        }
    }
    else
        throw Exception("InverseKinematicsSolverExt::updateMarkerWeights: invalid size of weights.");
}

int InverseKinematicsSolverExt::getNumOrientationSensorsInUse() const
{
    return _orientationAssemblyCondition->getNumOSensors();
}

int InverseKinematicsSolverExt::getNumDistanceSensorsInUse() const
{
    return _distanceAssemblyCondition->getNumDSensors();
}


/* Change the weighting of an orientation sensor to take effect when assemble or
track is called next. Update an orientation sensor's weight by name. */
void InverseKinematicsSolverExt::updateOrientationWeight(const std::string& orientationName, double value)
{
    const Array_<std::string> &names = _orientationsReference->getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), orientationName);
    int index = (int)std::distance(names.begin(), p);
    updateOrientationWeight(index, value);
}

/* Update an orientation sensor's weight by its index. */
void InverseKinematicsSolverExt::updateOrientationWeight(int orientationIndex, double value)
{
    if (orientationIndex >= 0 && orientationIndex < _orientationsReference->updOrientationWeightSet().getSize()) {
        _orientationsReference->updOrientationWeightSet()[orientationIndex].setWeight(value);
        _orientationAssemblyCondition->changeOSensorWeight(
            SimTK::OrientationSensors::OSensorIx(orientationIndex), value );
    }
    else
        throw Exception("InverseKinematicsSolverExt::updateOrientationWeight: invalid orientationIndex.");
}

/* Update all orientation sensor weights by order in the orientationsReference passed in to
construct the solver. */
void InverseKinematicsSolverExt::updateOrientationWeights(const SimTK::Array_<double> &weights)
{
    if (static_cast<unsigned>(_orientationsReference->updOrientationWeightSet().getSize())
        == weights.size()) {
        for (unsigned int i = 0; i<weights.size(); i++) {
            _orientationsReference->updOrientationWeightSet()[i].setWeight(weights[i]);
            _orientationAssemblyCondition->changeOSensorWeight(
                SimTK::OrientationSensors::OSensorIx(i), weights[i] );
        }
    }
    else
        throw Exception("InverseKinematicsSolverExt::updateOrientationWeights: invalid size of weights.");
}

/* Compute and return the spatial location of a marker in ground. */
SimTK::Vec3 InverseKinematicsSolverExt::computeCurrentMarkerLocation(const std::string &markerName)
{
    const Array_<std::string> &names = _markersReference->getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), markerName);
    int index = (int)std::distance(names.begin(), p);
    return computeCurrentMarkerLocation(index);
}

SimTK::Vec3 InverseKinematicsSolverExt::computeCurrentMarkerLocation(int markerIndex)
{
    if(markerIndex >=0 && markerIndex < _markerAssemblyCondition->getNumMarkers()){
        return _markerAssemblyCondition->findCurrentMarkerLocation(SimTK::Markers::MarkerIx(markerIndex));
    }
    else
        throw Exception("InverseKinematicsSolverExt::computeCurrentMarkerLocation: invalid markerIndex.");
}

/* Compute and return the spatial locations of all markers in ground. */
void InverseKinematicsSolverExt::computeCurrentMarkerLocations(SimTK::Array_<SimTK::Vec3> &markerLocations)
{
    markerLocations.resize(_markerAssemblyCondition->getNumMarkers());
    for(unsigned int i=0; i<markerLocations.size(); i++)
        markerLocations[i] = _markerAssemblyCondition->findCurrentMarkerLocation(SimTK::Markers::MarkerIx(i));
}


/* Compute and return the distance error between model marker and observation. */
double InverseKinematicsSolverExt::computeCurrentMarkerError(const std::string &markerName)
{
    const Array_<std::string>& names = _markersReference->getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), markerName);
    int index = (int)std::distance(names.begin(), p);
    return computeCurrentMarkerError(index);
}

double InverseKinematicsSolverExt::computeCurrentMarkerError(int markerIndex)
{
    if(markerIndex >=0 && markerIndex < _markerAssemblyCondition->getNumMarkers()){
        return _markerAssemblyCondition->findCurrentMarkerError(SimTK::Markers::MarkerIx(markerIndex));
    }
    else
        throw Exception("InverseKinematicsSolverExt::computeCurrentMarkerError: invalid markerIndex.");
}

/* Compute and return the distance errors between all model markers and their observations. */
void InverseKinematicsSolverExt::computeCurrentMarkerErrors(SimTK::Array_<double> &markerErrors)
{
    markerErrors.resize(_markerAssemblyCondition->getNumMarkers());
    for(unsigned int i=0; i<markerErrors.size(); i++)
        markerErrors[i] = _markerAssemblyCondition->findCurrentMarkerError(SimTK::Markers::MarkerIx(i));
}


/* Compute and return the squared-distance error between model marker and observation. */
double InverseKinematicsSolverExt::computeCurrentSquaredMarkerError(const std::string &markerName)
{
    const Array_<std::string>& names = _markersReference->getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), markerName);
    int index = (int)std::distance(names.begin(), p);
    return computeCurrentSquaredMarkerError(index);
}

double InverseKinematicsSolverExt::computeCurrentSquaredMarkerError(int markerIndex)
{
    if(markerIndex >=0 && markerIndex < _markerAssemblyCondition->getNumMarkers()){
        return _markerAssemblyCondition->findCurrentMarkerErrorSquared(SimTK::Markers::MarkerIx(markerIndex));
    }
    else
        throw Exception("InverseKinematicsSolverExt::computeCurrentMarkerSquaredError: invalid markerIndex.");
}

/* Compute and return the distance errors between all model marker and observations. */
void InverseKinematicsSolverExt::
    computeCurrentSquaredMarkerErrors(SimTK::Array_<double> &markerErrors)
{
    markerErrors.resize(_markerAssemblyCondition->getNumMarkers());
    for(unsigned int i=0; i<markerErrors.size(); i++)
        markerErrors[i] = _markerAssemblyCondition->
                    findCurrentMarkerErrorSquared(SimTK::Markers::MarkerIx(i));
}

/* Marker errors are reported in order different from tasks file or model, find name corresponding to passed in index  */
std::string InverseKinematicsSolverExt::getMarkerNameForIndex(int markerIndex) const
{
    return _markerAssemblyCondition->getMarkerName(SimTK::Markers::MarkerIx(markerIndex));
}

/* Compute and return the spatial orientation of an o-sensor in ground. */
SimTK::Rotation InverseKinematicsSolverExt::
    computeCurrentSensorOrientation(const std::string& osensorName)
{
    const Array_<std::string>& names = _orientationsReference->getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), osensorName);
    int index = (int)std::distance(names.begin(), p);
    return computeCurrentSensorOrientation(index);
}

SimTK::Rotation InverseKinematicsSolverExt::computeCurrentSensorOrientation(int osensorIndex)
{
    if (osensorIndex >= 0 && osensorIndex < _orientationAssemblyCondition->getNumOSensors()) {
        return _orientationAssemblyCondition->findCurrentOSensorOrientation(SimTK::OrientationSensors::OSensorIx(osensorIndex));
    }
    else
        throw Exception("InverseKinematicsSolverExt::computeCurrentOSensorOrientation: invalid osensorIndex.");
}

/* Compute and return the spatial locations of all markers in ground. */
void InverseKinematicsSolverExt::computeCurrentSensorOrientations(
        SimTK::Array_<SimTK::Rotation>& osensorOrientations)
{
    osensorOrientations.resize(_orientationAssemblyCondition->getNumOSensors());
    for (unsigned int i = 0; i< osensorOrientations.size(); i++)
        osensorOrientations[i] =
            _orientationAssemblyCondition->findCurrentOSensorOrientation(SimTK::OrientationSensors::OSensorIx(i));
}


/* Compute and return the orientation error between model o-sensor and observation. */
double InverseKinematicsSolverExt::
    computeCurrentOrientationError(const std::string& osensorName)
{
    const Array_<std::string>& names = _orientationsReference->getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), osensorName);
    int index = (int)std::distance(names.begin(), p);
    return computeCurrentOrientationError(index);
}

double InverseKinematicsSolverExt::computeCurrentOrientationError(int osensorIndex)
{
    if (osensorIndex >= 0 && 
        osensorIndex < _orientationAssemblyCondition->getNumOSensors()) {
        return _orientationAssemblyCondition->
            findCurrentOSensorError(
                SimTK::OrientationSensors::OSensorIx(osensorIndex));
    }
    else
        throw Exception(
            "InverseKinematicsSolverExt::computeCurrentOrientationError: "
            "invalid sensor Index." );
}

/* Compute and return the distance between all model o-sensors and their observations. */
void InverseKinematicsSolverExt::computeCurrentOrientationErrors(
                                          SimTK::Array_<double>& osensorErrors)
{
    osensorErrors.resize(_orientationAssemblyCondition->getNumOSensors());
    for (unsigned int i = 0; i<osensorErrors.size(); i++)
        osensorErrors[i] = _orientationAssemblyCondition->
        findCurrentOSensorError(OrientationSensors::OSensorIx(i));
}

/* Orientation errors may be reported in an order that may be different from
   tasks file or model, find name corresponding to passed in index  */
std::string InverseKinematicsSolverExt::
    getOrientationSensorNameForIndex(int osensorIndex) const
{
    return _orientationAssemblyCondition->getOSensorName(SimTK::OrientationSensors::OSensorIx(osensorIndex));
}

// START DISTANCES
/* Compute and return the spatial orientation of an o-sensor in ground. */
SimTK::Real InverseKinematicsSolverExt::
    computeCurrentSensorDistance(const std::string& dsesnorName)
{
    const Array_<std::string>& names = _distancesReference->getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), dsesnorName);
    int index = (int)std::distance(names.begin(), p);
    return computeCurrentSensorDistance(index);
}

SimTK::Real InverseKinematicsSolverExt::computeCurrentSensorDistance(int dsensorIndex)
{
    if (dsensorIndex >= 0 && dsensorIndex < _distanceAssemblyCondition->getNumDSensors()) {
        return _distanceAssemblyCondition->findCurrentDSensorDistance(SimTK::DistanceSensors::DSensorIx(dsensorIndex));
    }
    else
        throw Exception("InverseKinematicsSolverExt::computeCurrentDSensorDistance: invalid dsensorIndex.");
}

/* Compute and return the spatial locations of all markers in ground. */
void InverseKinematicsSolverExt::computeCurrentSensorDistances(
        SimTK::Array_<SimTK::Real>& dsensorDistances)
{
    dsensorDistances.resize(_distanceAssemblyCondition->getNumDSensors());
    for (unsigned int i = 0; i< dsensorDistances.size(); i++)
        dsensorDistances[i] =
            _distanceAssemblyCondition->findCurrentDSensorDistance(SimTK::DistanceSensors::DSensorIx(i));
}


/* Compute and return the orientation error between model o-sensor and observation. */
double InverseKinematicsSolverExt::
    computeCurrentDistanceError(const std::string& dsensorName)
{
    const Array_<std::string>& names = _distancesReference->getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), dsensorName);
    int index = (int)std::distance(names.begin(), p);
    return computeCurrentDistanceError(index);
}

double InverseKinematicsSolverExt::computeCurrentDistanceError(int dsensorIndex)
{
    if (dsensorIndex >= 0 && 
        dsensorIndex < _distanceAssemblyCondition->getNumDSensors()) {
        return _distanceAssemblyCondition->
            findCurrentDSensorError(
                SimTK::DistanceSensors::DSensorIx(dsensorIndex));
    }
    else
        throw Exception(
            "InverseKinematicsSolverExt::computeCurrentDistanceError: "
            "invalid sensor Index." );
}

/* Compute and return the distance between all model o-sensors and their observations. */
void InverseKinematicsSolverExt::computeCurrentDistanceErrors(
                                          SimTK::Array_<double>& dsensorErrors)
{
    dsensorErrors.resize(_distanceAssemblyCondition->getNumDSensors());
    for (unsigned int i = 0; i<dsensorErrors.size(); i++)
        dsensorErrors[i] = _distanceAssemblyCondition->
        findCurrentDSensorError(DistanceSensors::DSensorIx(i));
}

/* Distance errors may be reported in an order that may be different from
   tasks file or model, find name corresponding to passed in index  */
std::string InverseKinematicsSolverExt::
    getDistanceSensorNameForIndex(int dsensorIndex) const
{
    return _distanceAssemblyCondition->getDSensorName(SimTK::DistanceSensors::DSensorIx(dsensorIndex));
}

// END DISTANCES

/* Internal method to convert the MarkerReferences into additional goals of the 
    of the base assembly solver, that is going to do the assembly.  */
void InverseKinematicsSolverExt::setupGoals(SimTK::State &s)
{
    // Setup coordinates performed by the base class
    AssemblySolver::setupGoals(s);

    setupMarkersGoal(s);

    setupOrientationsGoal(s);

    setupDistancesGoal(s);

    updateGoals(s);
}

void InverseKinematicsSolverExt::setupMarkersGoal(SimTK::State &s)
{
    // If we have no markers reference to track, then return.
    if (!_markersReference || _markersReference->getNumRefs() < 1) {
        return;
    }

    // Setup markers goals
    // Get lists of all markers by names and corresponding weights from the MarkersReference
    const SimTK::Array_<SimTK::String>& markerNames = _markersReference->getNames();
    SimTK::Array_<double> markerWeights;
    _markersReference->getWeights(s, markerWeights);
    // get markers defined by the model 
    const MarkerSet &modelMarkerSet = getModel().getMarkerSet();

    // now build the Goal (AsemblyCondition) for Markers
    std::unique_ptr<SimTK::Markers> condOwner(new SimTK::Markers());
    _markerAssemblyCondition.reset(condOwner.get());

    int index = -1;
    SimTK::Transform X_BF;
    //Loop through all markers in the reference
    for (unsigned int i = 0; i < markerNames.size(); ++i) {
        // Check if we have this marker in the model, else ignore it
        index = modelMarkerSet.getIndex(markerNames[i], index);
        if(index >= 0) {
            Marker &marker = modelMarkerSet[index];
            const SimTK::MobilizedBody& mobod =
                marker.getParentFrame().getMobilizedBody();

            X_BF = marker.getParentFrame().findTransformInBaseFrame();
            _markerAssemblyCondition->
                addMarker(marker.getName(), mobod, X_BF*marker.get_location(),
                    markerWeights[i]);
        }
    }

    // Add marker goal to the ik objective and transfer ownership of the 
    // goal (AssemblyCondition) to Assembler
    updAssembler().adoptAssemblyGoal(condOwner.release());
    // lock-in the order that the observations (markers) are in and this cannot
    // change from frame to frame. We can use an array of just the data for
    // updating.
    _markerAssemblyCondition->defineObservationOrder(markerNames);
}

void InverseKinematicsSolverExt::setupOrientationsGoal(SimTK::State &s)
{
    // If we have no orientations reference to track, then return.
    if (!_orientationsReference || _orientationsReference->getNumRefs() < 1) {
        return;
    }

    // Setup orientations tracking goal
    // Get list of orientations by name  
    const SimTK::Array_<SimTK::String> &osensorNames =
        _orientationsReference->getNames();

    std::unique_ptr<SimTK::OrientationSensors> 
        condOwner(new SimTK::OrientationSensors());
    _orientationAssemblyCondition.reset(condOwner.get());

    SimTK::Array_<double> orientationWeights;
    _orientationsReference->getWeights(s, orientationWeights);
    // get orientation sensors defined by the model 
    const auto onFrames = getModel().getComponentList<PhysicalFrame>();

    for (const auto& modelFrame : onFrames) {
        const std::string& modelFrameName = modelFrame.getName();
        auto found = std::find(osensorNames.begin(), osensorNames.end(), modelFrameName);
        if (found != osensorNames.end()) {
            int index = (int)std::distance(osensorNames.begin(), found);
            _orientationAssemblyCondition->addOSensor(modelFrameName,
                modelFrame.getMobilizedBodyIndex(),
                modelFrame.findTransformInBaseFrame().R(),
                orientationWeights[index]);
        }
    }

    // Add orientations goal to the ik objective and transfer ownership of the 
    // goal (AssemblyCondition) to Assembler
    updAssembler().adoptAssemblyGoal(condOwner.release());
    // lock-in the order that the observations (orientations) are in and this
    // cannot change from frame to frame and we can use an array of just the
    // data for updating
    _orientationAssemblyCondition->defineObservationOrder(osensorNames);
}

void InverseKinematicsSolverExt::setupDistancesGoal(SimTK::State &s)
{
    // If we have no distances reference to track, then return.
    if (!_distancesReference || _distancesReference->getNumRefs() < 1) {
        return;
    }

    // Setup distances tracking goal
    // Get list of distances by name  
    const SimTK::Array_<SimTK::String> &dsensorNames =
        _distancesReference->getNames();

    std::unique_ptr<SimTK::DistanceSensors> 
        condOwner(new SimTK::DistanceSensors());
    _distanceAssemblyCondition.reset(condOwner.get());

    SimTK::Array_<double> distanceWeights;
    _distancesReference->getWeights(s, distanceWeights);
    // get distance sensors defined by the model 
    const auto onFrames = getModel().getComponentList<PhysicalFrame>();

    for (const auto& modelFrame : onFrames) {
        const std::string& modelFrameName = modelFrame.getName();
        // auto found = std::find(dsensorNames.begin(), dsensorNames.end(), modelFrameName);
        auto found = std::find_if(dsensorNames.begin(), dsensorNames.end(), 
                [&modelFrameName](std::string const& elem) {
                auto v = split<std::vector>(elem, '-');
                return v[0] == modelFrameName;
            });
        if (found != dsensorNames.end()) {
            // const auto result = split<std::vector>(*found, '-');
            // const auto pair = std::make_pair(result[0],result[1]);
            // std::cout << "pair 1:" << pair.first << " second: " << pair.second << std::endl;
            for (const auto& modelFrame2 : onFrames) {
                const std::string& modelFrameName2 = modelFrame2.getName();
                auto found2 = std::find_if(dsensorNames.begin(), dsensorNames.end(), 
                    [&modelFrameName, &modelFrameName2](std::string const& elem) {
                    auto v = split<std::vector>(elem, '-');
                    return v[0] == modelFrameName && v[1] == modelFrameName2;
                });
                if (found2 != dsensorNames.end()) {
                    // std::cout << "frame1: " << *found << " frame2: " << *found2 <<  std::endl;
                    std::cout << "adding sensor: " << *found2 << std::endl;
                    int index = (int)std::distance(dsensorNames.begin(), found2);
                    _distanceAssemblyCondition->addDSensor(*found2,
                        modelFrame.getMobilizedBodyIndex(),
                        modelFrame2.getMobilizedBodyIndex(),
                        (modelFrame.findTransformInBaseFrame().p() - modelFrame2.findTransformInBaseFrame().p()).norm(),
                        distanceWeights[index]);
                }
            }
        }
    }

    // Add distances goal to the ik objective and transfer ownership of the 
    // goal (AssemblyCondition) to Assembler
    updAssembler().adoptAssemblyGoal(condOwner.release());
    // lock-in the order that the observations (distances) are in and this
    // cannot change from frame to frame and we can use an array of just the
    // data for updating
    _distanceAssemblyCondition->defineObservationOrder(dsensorNames);
}

/* Internal method to update the time, reference values and/or their weights based
    on the state */
void InverseKinematicsSolverExt::updateGoals(SimTK::State &s)
{
    // get time from References and update s time
    int x = 0;

    if (_advanceTimeFromReference) {
        double nextTime = NaN;
        if (_orientationsReference &&
                _orientationsReference->getNumRefs() > 0) {
            SimTK::Array_<SimTK::Rotation> orientationValues;
            nextTime = _orientationsReference->getNextValuesAndTime(
                    orientationValues);
            s.setTime(nextTime);
            _orientationAssemblyCondition->moveAllObservations(
                    orientationValues);
        }
        // update coordinates if any based on new time
        AssemblySolver::updateGoals(s);
        return;
    }
    // update coordinates performed by the base class
    AssemblySolver::updateGoals(s);

    double nextTime = s.getTime();
    // specify the marker observations to be matched
    if (_markersReference && _markersReference->getNumRefs() > 0) {
        SimTK::Array_<SimTK::Vec3> markerValues;
        _markersReference->getValuesAtTime(nextTime, markerValues);
        _markerAssemblyCondition->moveAllObservations(markerValues);
    }

    // specify the orientation observations to be matched
    if (_orientationsReference && _orientationsReference->getNumRefs() > 0) {
        SimTK::Array_<SimTK::Rotation> orientationValues;
        _orientationsReference->getValuesAtTime(nextTime, orientationValues);
        _orientationAssemblyCondition->moveAllObservations(orientationValues);
    }

        // specify the distance observations to be matched
    if (_distancesReference && _distancesReference->getNumRefs() > 0) {
        SimTK::Array_<double> distanceValues;
        _distancesReference->getValuesAtTime(nextTime, distanceValues);
        _distanceAssemblyCondition->moveAllObservations(distanceValues);
    }
}

} // end of namespace OpenSim
