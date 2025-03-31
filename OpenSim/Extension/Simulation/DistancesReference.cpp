/* -------------------------------------------------------------------------- *
 *                   OpenSim:  DistancesReference.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

#include "DistancesReference.h"
#include <OpenSim/Common/Units.h>
#include <OpenSim/Common/TRCFileAdapter.h>
#include <SimTKcommon/internal/State.h>

using namespace std;
using namespace SimTK;

namespace OpenSim {

DistancesReference::DistancesReference()
        : StreamableReference_<double>() {
    constructProperties();
    setAuthors("Alexander Beattie");
}

DistancesReference::DistancesReference(const std::string& distanceFile,
    Units modelUnits) : DistancesReference()
{
    loadDistancesFile(distanceFile, modelUnits);
}


DistancesReference::DistancesReference(
    const TimeSeriesTable_<double>& distanceData,
    const Set<DistanceWeight>* distanceWeightSet) 
        : DistancesReference()
{
    _distanceData = distanceData;
    if (distanceWeightSet!=nullptr)
        upd_distance_weights()= *distanceWeightSet;
    populateFromDistanceData();
}

void DistancesReference::loadDistancesFile(
                                    const std::string distanceFile,
                                    Units modelUnits)
{
    upd_distance_file() = distanceFile;

    _distanceData = TimeSeriesTable_<double>(distanceFile);

    populateFromDistanceData();
}

void DistancesReference::populateFromDistanceData()
{
    const std::vector<std::string>& tempNames = 
        _distanceData.getColumnLabels();
    unsigned int no = unsigned(tempNames.size());

    // empty any lingering names and weights
    _distanceNames.clear();
    _weights.clear();
    // pre-allocate arrays to the number of Distances in the file with default weightings
    _distanceNames.assign(no, "");
    _weights.assign(no, get_default_weight());

    int index = 0;
    // Build flat lists (arrays) of distance names and weights in the same order as the distance data
    for(unsigned int i=0; i<no; ++i){
        const std::string &name = tempNames[i];
        _distanceNames[i] = name;
        index = get_distance_weights().getIndex(name, index);
        //Assign user weighting for Distances that are user listed in the input set
        if(index >= 0)
            _weights[i] = get_distance_weights()[index].getWeight();
    }

    if(_distanceNames.size() != _weights.size())
        throw Exception("DistancesReference: Mismatch between the number "
            "of distance names and weights. Verify that distance names "
            "are unique.");
}

int DistancesReference::getNumRefs() const
{
    return int(_distanceData.getNumColumns());
}

double DistancesReference::getSamplingFrequency() const
{
    return std::atoi(_distanceData.getTableMetaData().getValueForKey("DataRate")
        .getValue<std::string>().c_str());
}

SimTK::Vec2 DistancesReference::getValidTimeRange() const
{
    auto& times = _distanceData.getIndependentColumn();
    return Vec2(*times.begin(), *(--times.end()));
}

const std::vector<double>& DistancesReference::getTimes() const
{
    return _distanceData.getIndependentColumn();
}

// utility to define object properties including their tags, comments and 
// default values.
void DistancesReference::constructProperties()
{
    constructProperty_distance_file("");
    Set<DistanceWeight> distanceWeights;
    constructProperty_distance_weights(distanceWeights);
    constructProperty_default_weight(1.0);
}

/** get the names of the Distances serving as references */
const SimTK::Array_<std::string>& DistancesReference::getNames() const
{
    return _distanceNames;
}

/** get the values of the DistancesReference */
void DistancesReference::getValuesAtTime(
        double time, SimTK::Array_<double> &values) const
{

    // get values for time
    SimTK::RowVector_<double> row = _distanceData.getNearestRow(time);

    int n = row.size();
    values.resize(n);

    for (int i = 0; i < n; ++i) {
        values[i] = row[i];
    }
}

/** get the weights of the Distances */
void  DistancesReference::getWeights(const SimTK::State &s, SimTK::Array_<double> &weights) const
{
    weights = _weights;
}

void DistancesReference::setDistanceWeightSet(const Set<DistanceWeight>& distanceWeights)
{
    upd_distance_weights() = distanceWeights;
}

} // end of namespace OpenSim