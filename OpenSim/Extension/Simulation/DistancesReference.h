#ifndef OPENSIM_DISTANCES_REFERENCE_H_
#define OPENSIM_DISTANCES_REFERENCE_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  DistancesReference.h                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
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

#include <OpenSim/Simulation/Reference.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Common/Units.h>

namespace OpenSim {

class OSIMSIMULATION_API DistanceWeight : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(DistanceWeight, Object);
private:
    OpenSim_DECLARE_PROPERTY(weight, double, "Distance reference weight.");

public:
    DistanceWeight() : Object() { constructProperties(); }

    DistanceWeight(std::string name, double weight) : DistanceWeight() {
        setName(name);
        upd_weight() = weight;
    }

    void setWeight(double weight) { upd_weight() = weight; }
    double getWeight() const {return get_weight(); }

private:
    void constructProperties() {
        constructProperty_weight(1.0);
    }

}; // end of DistanceWeight class


//=============================================================================
//=============================================================================
/**
 * Reference values for the Distances of model frames that will be used to
 * to compute tracking errors. An Distance is specified by a Rotation
 * matrix describing the frame distance with respect to Ground. The 
 * reference also contains weightings that identifies the relative importance
 * of achieving one distance's reference value over another.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API DistancesReference
        : public StreamableReference_<double> {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            DistancesReference, StreamableReference_<double>);
    //=============================================================================
// Properties
//=============================================================================
public:
    OpenSim_DECLARE_PROPERTY(distance_file, std::string, 
        "Distance file (.sto) containing the time history of observations "
        "of frame (sensor) distances.");

    OpenSim_DECLARE_PROPERTY(distance_weights, Set<DistanceWeight>,
        "Set of distance weights identified by distance name with weight "
        " being a positive scalar.");

    OpenSim_DECLARE_PROPERTY(default_weight, double,
        "Default weight for an distance.");
//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    DistancesReference();

    /** Convenience load Distances data from a file in the form of XYZ 
        body-fixed Euler angles. Units default to Radians.*/
    DistancesReference(const std::string& distanceFileName,
                     Units modelUnits=Units(Units::Radians));
    /** Form a Reference from TimeSeriesTable of Rotations and corresponding
    distance weights. The input orientatonWeightSet is used to initialize
    Reference weightings for individual Distances. Weights are associated
    to Distances by name.*/
    DistancesReference(const TimeSeriesTable_<double>& distanceData,
        const Set<DistanceWeight>* distanceWeightSet=nullptr);

    virtual ~DistancesReference() {}

    /** load the distance data for this DistancesReference from a file
    containing Euler-angles in body-fixed XYZ order.*/
    void loadDistancesFile(const std::string eulerAnglesXYZ,
        Units modelUnits=Units(Units::Radians));

    //--------------------------------------------------------------------------
    // Reference Interface
    //--------------------------------------------------------------------------
    int getNumRefs() const override;
    /** get the time range for which the DistancesReference values are valid,
        based on the loaded distance data.*/
    SimTK::Vec2 getValidTimeRange() const override;
    /** get the times at which the DistancesReference values are specified,
        based on the loaded distance data.*/
    const std::vector<double>& getTimes() const;
    /** get the names of the Distances serving as references */
    const SimTK::Array_<std::string>& getNames() const override;
    /** get the value of the DistancesReference */
    void getValuesAtTime(double time,
        SimTK::Array_<double>& values) const override;
    /** Default implementation does not support streaming */
    virtual double getNextValuesAndTime(
            SimTK::Array_<double>& values) override {
        throw Exception("getNextValuesAndTime method is not supported for this "
                        "reference {}.",
                this->getName());
    };
    virtual bool hasNext() const override { return false; };
    /** get the weighting (importance) of meeting this DistancesReference in the
        same order as names*/
    void getWeights(const SimTK::State& s,
                    SimTK::Array_<double>& weights) const override;

    //--------------------------------------------------------------------------
    // Convenience Access
    //--------------------------------------------------------------------------
    double getSamplingFrequency() const;
    Set<DistanceWeight>& updDistanceWeightSet()
        { return upd_distance_weights(); }
    /** %Set the distance weights from a set of DistanceWeights, which is
    const and a copy of the Set is used internally. Therefore, subsequent changes
    to the Set of DistanceWeights will have no effect on the distance weights
    associated with this Reference. You can, however, change the weightings on the
    InverseKinematicsSolver prior to solving at any instant in time. */
    void setDistanceWeightSet(const Set<DistanceWeight>& distanceWeights);
    void setDefaultWeight(double weight) { set_default_weight(weight); }

private:
    void constructProperties();
    void populateFromDistanceData();

protected:
    // Use a specialized data structure for holding the distance data
    TimeSeriesTable_<double> _distanceData;

private:
    // distance names inside the distance data
    SimTK::Array_<std::string> _distanceNames;
    // corresponding list of weights guaranteed to be in the same order as names above
    SimTK::Array_<double> _weights;

//=============================================================================
};  // END of class DistancesReference
//=============================================================================
} // namespace

#endif // OPENSIM_DISTANCES_REFERENCE_H_
