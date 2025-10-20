#ifndef SimTK_SIMBODY_ASSEMBLY_CONDITION_DISTANCE_SENSORS_H_
#define SimTK_SIMBODY_ASSEMBLY_CONDITION_DISTANCE_SENSORS_H_

/* -------------------------------------------------------------------------- *
 *                               Simbody(tm)                                  *
 * -------------------------------------------------------------------------- *
 * This is part of the SimTK biosimulation toolkit originating from           *
 * Simbios, the NIH National Center for Physics-Based Simulation of           *
 * Biological Structures at Stanford, funded under the NIH Roadmap for        *
 * Medical Research, grant U54 GM072970. See https://simtk.org/home/simbody.  *
 *                                                                            *
 * Portions copyright (c) 2014-24 Stanford University and the Authors.        *
 * Authors: Alexander Beattie, Michael Sherman                                *
 * Contributors:                                                              *
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

#include "SimTKcommon.h"
#include "simbody/internal/Assembler.h"
#include "simbody/internal/AssemblyCondition.h"
#include "simbody/internal/common.h"

#include <map>

namespace SimTK {

//------------------------------------------------------------------------------
//                           ORIENTATION SENSORS
//------------------------------------------------------------------------------
/** This AssemblyCondition specifies a correspondence between orientation
sensors fixed on mobilized bodies ("dsensors") and Ground-relative orientation
sensor readings ("observations"). Each dsensor represents the three orthogonal
axis directions of a frame fixed to a mobilized body; only orientation and not
location is represented. Distance sensing is commonly performed by
Inertial Measurements Units (IMUs). The behavior here is designed to be as
similar as possible to position sensors handled by the Markers
AssemblyCondition. You can combine DistanceSensors and Markers on the same
body.

The idea is to adjust the q's so that each dsensor is oriented similarly to its
corresponding observation. This is normally used as a goal since we don't expect
to be able to obtain a perfect match, but you can use these as a set of
assembly error conditions if there are enough degrees of freedom to achieve a
near-perfect solution.

Osensors are defined one at a time and assigned sequential index values
of type DistanceSensors::DSensorIx. They may optionally be given unique,
case-sensitive names, and we will keep a map from name to DSensorIx. A default
name will be assigned if none is given. A weight is assigned to every dsensor,
with default weight=1. We do not expect that all the dsensors will be used;
dsensors with weights of zero will not be included in the study, nor will
dsensors for which no observation is given.

Once specified, the dsensor definitions do not change during a series of inverse
kinematic (tracking) steps. The observations, on the other hand, are expected
to come from a time series of experimental measurements of dsensor orientations
and will be different at every step. They typically come from a file organized
by "frame", meaning an observation time and a set of observed orientations, one
per sensor, corresponding to that time. During initial setup, the number of
observations per frame and their correspondence to the defined dsensors is
specified. They can be in any order, may skip some dsensors, and may include
data for dsensors that are not defined. However, once initialized each frame
must supply the same information in the same order. Data for an unobserved
dsensor can be provided as NaN in which case it will be ignored in that frame.
The frame time is supplied to the track() method which initiates assembly for
a frame.

Observation-dsensor correspondence maps an ObservationIx to a unique DSensorIx.
By default, we'll expect to get an observation for each dsensor and that the
observation order and the dsensor order are the same, i.e.
ObservationIx==DSensorIx for every dsensor. However, you can instead define
observation/dsensor correspondence yourself, (\e after all dsensors have been
defined), via one of the defineObservationOrder() methods. This is done by
supplying an array of DSensorIx values, or an array of dsensor names, with the
array elements ordered by ObservationIx. Any invalid dsensor index or
unrecognized dsensor name means we will ignore values provide for that
observation; similarly, any dsensors whose index or name is not specified at
all will be ignored.
**/
class SimTK_SIMBODY_EXPORT DistanceSensors : public AssemblyCondition {

  // This is a private class used in the implementation below but not
  // accessible through the API.
  struct DSensor {
    DSensor(const String &name, MobilizedBodyIndex bodyA, const Vec3 &sensorInA,
            MobilizedBodyIndex bodyB, const Vec3 &sensorInB,
            const Real &distance, Real weight = 1)
        : name(name), bodyA(bodyA), sensorInA(sensorInA), bodyB(bodyB),
          sensorInB(sensorInB), distance(distance), weight(weight) {
      assert(weight >= 0);
    }

    DSensor(MobilizedBodyIndex bodyA, const Vec3 &sensorInA,
            MobilizedBodyIndex bodyB, const Vec3 &sensorInB,
            const Real &distance, Real weight = 1)
        : name(""), bodyA(bodyA), sensorInA(sensorInA), bodyB(bodyB),
          sensorInB(sensorInB), distance(distance), weight(weight) {
      assert(weight >= 0);
    }

    String name;
    MobilizedBodyIndex bodyA;
    Vec3 sensorInA;
    MobilizedBodyIndex bodyB;
    Vec3 sensorInB;
    Real distance;
    Real weight;
  };

public:
  /** Define the DSensorIx type which is just a uniquely-typed int. **/
  SimTK_DEFINE_UNIQUE_LOCAL_INDEX_TYPE(DistanceSensors, DSensorIx);
  /** Define the ObservationIx type which is just a uniquely-typed int. **/
  SimTK_DEFINE_UNIQUE_LOCAL_INDEX_TYPE(DistanceSensors, ObservationIx);

  //------------------------------------------------------------------------------
  /** @name                Construction and setup
  These methods are used as an extended construction phase for DistanceSensors
  objects, defining the dsensors and observations that will be used in the
  subsequent tracking steps. **/
  /*@{*/

  /** The default constructor creates an empty DistanceSensors
  AssemblyCondition object that should be filled in with calls fto addDSensor()
  and optionally defineObservationOrder(). **/
  DistanceSensors() : AssemblyCondition("DistanceSensors") {}

  /** Define a new orientation sensor (dsensor) attached to a particular
  MobilizedBody. Note that an dsensor will be ignored unless an observation is
  provided for it.
  @param[in]      name
      A unique name to be used to identify this dsensor. If the name is
      empty or blank, a default name will be supplied.
  @param[in]      bodyB
      The MobilizedBody to which this dsensor is fixed. DSensors on Ground
      are allowed but will be ignored.
  @param[in]      distance
      This is the orientation of the dsensor in \a bodyB's local frame.
  @param[in]      weight
      An optional weight for use in defining the objective function, which
      combines errors in this dsensor's orientation with errors in other
  dsensors' orientation. If the weight is zero this dsensor is ignored.
  @return The unique dsensor index number assigned to this dsensor. These are
  assigned sequentially as the dsensors are added.
  @note Adding an dsensor invalidates any observation/dsensor correspondence; be
  sure to call defineObservationOrder() \e after defining all your dsensors. **/
  DSensorIx addDSensor(const String &name, MobilizedBodyIndex bodyA,
                       const Vec3 &sensorInA, MobilizedBodyIndex bodyB,
                       const Vec3 &sensorInB, const Real &distance,
                       Real weight = 1) {
    SimTK_ERRCHK1_ALWAYS(isFinite(weight) && weight >= 0,
                         "DistanceSensors::addDSensor()",
                         "Illegal orientation sensor weight %g.", weight);
    uninitializeAssembler();
    // Forget any previously-established observation/dsensor correspondence.
    observation2dsensor.clear();
    dsensor2observation.clear();
    observations.clear();
    const DSensorIx ix(dsensors.size());
    String nm = String::trimWhiteSpace(name);
    if (nm.empty())
      nm = String("_UNNAMED_") + String(ix);

    std::pair<std::map<String, DSensorIx>::iterator, bool> found =
        dsensorsByName.insert(std::make_pair(nm, ix));
    SimTK_ERRCHK2_ALWAYS(found.second, // true if insertion was done
                         "DSensors::addDSensor()",
                         "DSensor name '%s' was already use for DSensor %d.",
                         nm.c_str(), (int)found.first->second);

    dsensors.push_back(
        DSensor(nm, bodyA, sensorInA, bodyB, sensorInB, distance, weight));
    return ix;
  }

  /** Define an unnamed dsensor. A default name will be assigned; that name
  will be "_UNNAMED_XX" where XX is the DSensorIx assigned to that dsensor
  (don't use names of that form yourself).
  @see addDSensor(name,...) for more information. **/
  DSensorIx addDSensor(MobilizedBodyIndex bodyA, const Vec3 &sensorInA,
                       MobilizedBodyIndex bodyB, const Vec3 &sensorInB,
                       const Real &distance, Real weight = 1) {
    return addDSensor("", bodyA, sensorInA, bodyB, sensorInB, distance, weight);
  }

  /** Define the meaning of the observation data by giving the DSensorIx
  associated with each observation. The length of the array of dsensor indices
  defines the expected number of observations to be provided for each
  observation frame. Any dsensor index that is supplied with an invalid value
  means that the corresponding observation will be present in the supplied data
  but should be ignored.
  @param[in]          observationOrder
      This is an array of dsensor index values, one per observation, that
  defines both the number of expected observations and the dsensor corresponding
      to each observation. DSensors can be in any order; an invalid dsensor
  index means that observation will be provided but should be ignored; dsensors
  whose indices are never listed are ignored. If \a observationOrder is supplied
  as a zero-length array, then we'll assume there are as many observations as
  dsensors and that their indices match.

  @note If you don't call this method at all, a default correspondence will
  be defined as described for a zero-length \a observationOrder array (that is,
  same number of observations and dsensors with matching indices). Whenever you
  add a new dsensor, any previously defined observation order is forgotten so
  the default correspondence will be used unless you call this again. **/
  void defineObservationOrder(const Array_<DSensorIx> &observationOrder) {
    uninitializeAssembler();
    if (observationOrder.empty()) {
      observation2dsensor.resize(dsensors.size());
      for (DSensorIx mx(0); mx < dsensors.size(); ++mx)
        observation2dsensor[ObservationIx(mx)] = mx;
    } else
      observation2dsensor = observationOrder;
    dsensor2observation.clear();
    // We might need to grow this more, but this is an OK starting guess.
    dsensor2observation.resize(observation2dsensor.size()); // all invalid
    for (ObservationIx ox(0); ox < observation2dsensor.size(); ++ox) {
      const DSensorIx mx = observation2dsensor[ox];
      if (!mx.isValid())
        continue;

      if (dsensor2observation.size() <= mx)
        dsensor2observation.resize(mx + 1);
      SimTK_ERRCHK4_ALWAYS(
          !dsensor2observation[mx].isValid(),
          "DSensors::defineObservationOrder()",
          "An attempt was made to associate DSensor %d (%s) with"
          " Observations %d and %d; only one Observation per DSensor"
          " is permitted.",
          (int)mx, getDSensorName(mx).c_str(), (int)dsensor2observation[mx],
          (int)ox);

      dsensor2observation[mx] = ox;
    }
    // Make room for dsensor observations.
    observations.clear();
    observations.resize(observation2dsensor.size(), NaN);
  }

  /** Define the meaning of the observations by giving the dsensor name
  corresponding to each observation, as a SimTK::Array_<String>. The length of
  the array of dsensor indices defines the expected number of observations. Any
  dsensor name that is unrecognized or empty means that the corresponding
  observation will be present in the supplied data but should be ignored. **/
  void defineObservationOrder(const Array_<String> &observationOrder) {
    Array_<DSensorIx> dsensorIxs(observationOrder.size());
    for (ObservationIx ox(0); ox < observationOrder.size(); ++ox)
      dsensorIxs[ox] = getDSensorIx(observationOrder[ox]);
    defineObservationOrder(dsensorIxs);
  }

  /** Define observation order using an std::vector of SimTK::String. **/
  // no copy required
  void defineObservationOrder(const std::vector<String> &observationOrder) {
    defineObservationOrder(ArrayViewConst_<String>(observationOrder));
  }

  /** Define observation order using an Array_ of std::string. **/
  // must copy
  void defineObservationOrder(const Array_<std::string> &observationOrder) {
    const Array_<String> observations(observationOrder); // copy
    defineObservationOrder(observations);
  }

  /** Define observation order using an std::vector of std::string. **/
  // must copy
  void
  defineObservationOrder(const std::vector<std::string> &observationOrder) {
    const Array_<String> observations(observationOrder); // copy
    defineObservationOrder(observations);
  }

  /** Define observation order using a C array of const char* names. **/
  void defineObservationOrder(int n, const char *const observationOrder[]) {
    Array_<DSensorIx> dsensorIxs(n);
    for (ObservationIx ox(0); ox < n; ++ox)
      dsensorIxs[ox] = getDSensorIx(String(observationOrder[ox]));
    defineObservationOrder(dsensorIxs);
  }
  /*@}*/

  //------------------------------------------------------------------------------
  /** @name               Retrieve setup information
  These methods are used to query information associated with the construction
  and setup of this DistanceSensors object. This information does not normally
  change during an dsensor-tracking study, although dsensor weights may be
  changed by some inverse kinematics methods. **/
  /*@{*/

  /** Return a count n of the number of currently-defined dsensors. Valid
  dsensor index values (of type DistanceSensors::DSensorIx) are 0..n-1. **/
  int getNumDSensors() const { return dsensors.size(); }

  /** Return the unique dsensor name assigned to the dsensor whose index
  is provided. If the dsensor was defined without a name, this will return
  the default name that was assigned to it. **/
  const String &getDSensorName(DSensorIx ix) { return dsensors[ix].name; }

  /** Return the dsensor index associated with the given dsensor name. If the
  name is not recognized the returned index will be invalid (test with
  index.isValid()). **/
  const DSensorIx getDSensorIx(const String &name) {
    std::map<String, DSensorIx>::const_iterator p = dsensorsByName.find(name);
    return p == dsensorsByName.end() ? DSensorIx() : p->second;
  }

  /** Get the weight currently in use for the specified dsensor; this can
  be changed dynamically via changeDSensorWeight(). **/
  Real getDSensorWeight(DSensorIx mx) { return dsensors[mx].weight; }

  /** Get the MobilizedBodyIndex of the body associated with this dsensor. **/
  MobilizedBodyIndex getDSensorBody(DSensorIx mx) const {
    return dsensors[mx].bodyB;
  }

  /** Get the orientation (coordinate axes fixed in its body frame) of the given
  dsensor. **/
  const Real &getDSensorStation(DSensorIx mx) const {
    return dsensors[mx].distance;
  }

  /** Return the number of observations that were defined via the last call to
  defineObservationOrder(). These are not necessarily all being used. If
  defineObservationOrder() was never called, we'll expect the same number of
  observations as dsensors although that won't be set up until the Assembler has
  been initialized. **/
  int getNumObservations() const { return observation2dsensor.size(); }

  /** Return the ObservationIx of the observation that is currently associated
  with the given dsensor, or an invalid index if the dsensor doesn't have any
  corresponding observation (in which case it is being ignored). An exception
  will be thrown if the given DSensorIx is not in the range
  0..getNumDSensors()-1. **/
  ObservationIx getObservationIxForDSensor(DSensorIx mx) const {
    return dsensor2observation[mx];
  }

  /** Return true if the supplied dsensor is currently associated with an
  observation. @see getObservationIxForDSensor() **/
  bool hasObservation(DSensorIx mx) const {
    return getObservationIxForDSensor(mx).isValid();
  }

  /** Return the DSensorIx of the dsensor that is associated with the
  given observation, or an invalid index if the observation doesn't correspond
  to any dsensor (in which case it is being ignored). An exception will be
  thrown if the given ObservationIx is not in the range
  0..getNumObservations()-1. **/
  DSensorIx getDSensorIxForObservation(ObservationIx ox) const {
    return observation2dsensor[ox];
  }

  /** Return true if the supplied observation is currently associated with a
  dsensor. @see getDSensorIxForObservation() **/
  bool hasDSensor(ObservationIx ox) const {
    return getDSensorIxForObservation(ox).isValid();
  }

  /** The DSensors assembly condition organizes the dsensors by body after
  initialization; call this to get the list of dsensors on any particular body.
  If necessary the Assembler will be initialized. It is an error if this
  assembly condition has not yet been adopted by an Assembler. **/
  const Array_<DSensorIx> &getDSensorsOnBody(MobilizedBodyIndex mbx) {
    static const Array_<DSensorIx> empty;
    SimTK_ERRCHK_ALWAYS(
        isInAssembler(), "DSensors::getDSensorsOnBody()",
        "This method can't be called until the DSensors object has been"
        " adopted by an Assembler.");
    initializeAssembler();
    PerBodyDSensors::const_iterator bodyp = bodiesWithDSensors.find(mbx);
    return bodyp == bodiesWithDSensors.end() ? empty : bodyp->second;
  }
  /*@}*/

  //------------------------------------------------------------------------------
  /** @name                Execution methods
  These methods can be called between tracking steps to make step-to-step
  changes without reinitialization, and to access the current values of
  step-to-step data including the resulting dsensor errors. **/
  /*@{*/

  /** Move a single dsensor's observed orientation without moving any of the
  others. If the value contains a NaN, this dsensor/observation pair will be
  ignored the next time the assembly goal cost function is calculated. **/
  void moveOneObservation(ObservationIx ox, const Real &observation) {
    SimTK_ERRCHK_ALWAYS(
        !observations.empty(), "Assembler::moveOneObservation()",
        "There are currently no observations defined. Either the Assembler"
        " needs to be initialized to get the default observation order, or you"
        " should call defineObservationOrder() explicitly.");
    SimTK_ERRCHK2_ALWAYS(
        ox.isValid() && ox < observations.size(),
        "Assembler::moveOneObservation()",
        "ObservationIx %d is invalid or"
        " out of range; there are %d observations currently defined. Use"
        " defineObservationOrder() to specify the set of observations and how"
        " they correspond to dsensors.",
        (int)ox, (int)observations.size());
    observations[ox] = observation;
  }

  /** Set the observed dsensor orientations for a new observation frame. These
  are the orientations to which we will next attempt to rotate all the
  corresponding dsensors. Note that not all observations necessarily have
  corresponding dsensors defined; orientations of those dsensors must still be
  provided here but they will be ignored. The length of the \a allObservations
  array must be the same as the number of defined observations; you can obtain
  that using getNumObservations(). Any observations that contain a NaN will be
  ignored; that dsensor/observation pair will not be used in the next
  calculation of the assembly goal cost function. **/
  void moveAllObservations(const Array_<Real> &observations) {
    SimTK_ERRCHK2_ALWAYS(
        (int)observations.size() == (int)observation2dsensor.size(),
        "DSensors::moveAllObservations()",
        "Number of observations provided (%d) differs from the number of"
        " observations (%d) last defined with defineObservationOrder().",
        observations.size(), observation2dsensor.size());
    this->observations = observations;
  }

  /** Change the weight associated with a particular dsensor. If this is just
  a quantitative change (e.g., weight was 0.3 now it is 0.4) then this does
  not require any reinitialization and will affect the goal calculation next
  time it is done. If the weight changes to or from zero (a qualitative change)
  then this will uninitialize the Assembler and all the internal data structures
  will be changed to remove or add this dsensor from the list of active
  dsensors. If you want to temporarily ignore an dsensor without reinitializing,
  you can set its corresponding observation to NaN in which case it will simply
  be skipped when the goal value is calculated. **/
  void changeDSensorWeight(DSensorIx mx, Real weight) {
    SimTK_ERRCHK1_ALWAYS(isFinite(weight) && weight >= 0,
                         "DSensors::changeDSensorWeight()",
                         "Illegal dsensor weight %g.", weight);

    DSensor &dsensor = dsensors[mx];
    if (dsensor.weight == weight)
      return;

    if (dsensor.weight == 0 || weight == 0)
      uninitializeAssembler(); // qualitative change

    dsensor.weight = weight;
  }

  /** Return the current value of the orientation for this observation. This
  is the orientation to which we will try to rotate the corresponding dsensor if
  there is one. The result might be NaN if there is no current value for this
  observation; you can check using Rotation's isFinite() method. **/
  const Real &getObservation(ObservationIx ox) const {
    return observations[ox];
  }

  /** Return the current values of all the observed orientations. These are the
  orientations to which we will try to rotate the corresponding dsensors, for
  those observations that have corresponding dsensors defined. Some of the
  values may be NaN if there is currently no corresponding observation. Note
  that these are indexed by ObservationIx; use getObservationIxForDSensor() to
  map a DSensorIx to its corresponding ObservationIx. **/
  const Array_<Real, ObservationIx> &getAllObservations() const {
    return observations;
  }

  /** Using the current value of the internal state, calculate the ground
  frame orientation of a particular dsensor. The difference between this
  orientation and the corresponding observation is the current error for this
  dsensor. **/
  Real findCurrentDSensorDistance(DSensorIx mx) const;

  /** Using the current value of the internal state, calculate the error
  between the given dsensor's current orientation and its corresponding observed
  orientation (unweighted), as a nonnegative angle in radians between 0 and Pi.
  If the dsensor is not associated with an observation, or if the observed
  location is missing (indicated by a NaN value), then the error is reported as
  zero. **/
  Real findCurrentDSensorError(DSensorIx mx) const {
    const ObservationIx ox = getObservationIxForDSensor(mx);
    // TODO: Fix this
    if (!ox.isValid())
      return 0; // no observation for this dsensor
    const Real &obs = getObservation(ox);
    if (isNaN(obs))
      return 0; // NaN in observation; error is ignored
    const Real calc = findCurrentDSensorDistance(mx);
    const Real error = calc - obs; // distance error, in S
    return std::abs(error);
    return 0;
  }
  /*@}*/

  //------------------------------------------------------------------------------
  /** @name              AssemblyCondition virtuals
  These methods are the implementations of the AssemblyCondition virtuals. **/
  /*@{*/
  int initializeCondition() const override;
  void uninitializeCondition() const override;
  int calcErrors(const State &state, Vector &err) const override;
  int calcErrorJacobian(const State &state, Matrix &jacobian) const override;
  int getNumErrors(const State &state) const override;
  int calcGoal(const State &state, Real &goal) const override;
  int calcGoalGradient(const State &state, Vector &grad) const override;
  /*@}*/

  //------------------------------------------------------------------------------
private:
  //------------------------------------------------------------------------------
  const DSensor &getDSensor(DSensorIx i) const { return dsensors[i]; }
  DSensor &updDSensor(DSensorIx i) {
    uninitializeAssembler();
    return dsensors[i];
  }

  // data members

  // DSensor definition. Any change here except a quantitative change to the
  // dsensor's weight uninitializes the Assembler.
  Array_<DSensor, DSensorIx> dsensors;
  std::map<String, DSensorIx> dsensorsByName;

  // Observation-dsensor corresondence specification. Any change here
  // uninitializes the Assembler.
  Array_<DSensorIx, ObservationIx> observation2dsensor;

  // For convience in mapping from an dsensor to its corresponding observation.
  // ObservationIx will be invalid if a particular dsensor has no associated
  // observation.
  Array_<ObservationIx, DSensorIx> dsensor2observation;

  // This is the current set of dsensor orientation observations, one per entry
  // in the observation2dsensor array. Changing the values here does not
  // uninitialize the Assembler.
  Array_<Real, ObservationIx> observations;

  // After initialize, this groups the dsensors by body and weeds out
  // any zero-weighted dsensors. TODO: skip low-weighted dsensors, at
  // least at the start of the assembly.
  typedef std::map<MobilizedBodyIndex, Array_<DSensorIx>> PerBodyDSensors;
  mutable PerBodyDSensors bodiesWithDSensors;
};

} // namespace SimTK

#endif // SimTK_SIMBODY_ASSEMBLY_CONDITION_ORIENTATION_SENSORS_H_
