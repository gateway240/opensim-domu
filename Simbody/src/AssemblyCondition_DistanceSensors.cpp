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

#include "simbody/internal/AssemblyCondition_DistanceSensors.h"
#include "simbody/internal/Assembler.h"
#include "simbody/internal/AssemblyCondition.h"
#include "simbody/internal/MobilizedBody.h"
#include "simbody/internal/MultibodySystem.h"
#include "simbody/internal/SimbodyMatterSubsystem.h"
#include <map>

using namespace SimTK;

//------------------------------------------------------------------------------
//                           DISTANCE SENSORS
//------------------------------------------------------------------------------

Real DistanceSensors::findCurrentDSensorDistance(DSensorIx mx) const {
  const SimbodyMatterSubsystem &matter = getMatterSubsystem();
  const DSensor &dsensor = getDSensor(mx);
  const MobilizedBody &mobodA = matter.getMobilizedBody(dsensor.bodyA);
  const MobilizedBody &mobodB = matter.getMobilizedBody(dsensor.bodyB);
  const State &state = getAssembler().getInternalState();
  const Transform &X_GB = mobodA.getBodyTransform(state);
  const Transform &Y_GB = mobodB.getBodyTransform(state);
  return (X_GB * dsensor.sensorInA - Y_GB * dsensor.sensorInB).norm();
}

// goal = 1/2 sum( wi * ai^2 ) / sum(wi) for WRMS
// ai == rotation angle between sensor and observation (-Pi:Pi)
int DistanceSensors::calcGoal(const State &state, Real &goal) const {
  const SimbodyMatterSubsystem &matter = getMatterSubsystem();
  goal = 0;
  // Loop over each body that has one or more active dsensors.
  Real wtot = 0;
  PerBodyDSensors::const_iterator bodyp = bodiesWithDSensors.begin();
  for (; bodyp != bodiesWithDSensors.end(); ++bodyp) {
    const MobilizedBodyIndex mobodIx = bodyp->first;
    const Array_<DSensorIx> &bodyDSensors = bodyp->second;
    const MobilizedBody &mobod = matter.getMobilizedBody(mobodIx);
    const Transform &X_GB = mobod.getBodyTransform(state);
    assert(bodyDSensors.size());
    // Loop over each dsensor on this body.
    for (unsigned m = 0; m < bodyDSensors.size(); ++m) {
      const DSensorIx mx = bodyDSensors[m];
      const DSensor &dsensor = dsensors[mx];
      // std::cout << "Name: " << dsensor.name  << " DSensorIx: " << mx <<
      // std::endl; std::cout << "observation" << ""
      assert(dsensor.bodyA == mobodIx); // better be on this body!
      const MobilizedBody &mobodB = matter.getMobilizedBody(dsensor.bodyB);
      const Real &obs = getObservation(getObservationIxForDSensor(mx));
      // std::cout << "obs: " << obs << "is nan: " << isNaN(obs) << std::endl;
      if (!isNaN(obs)) { // skip NaNs
        const Transform &Y_GB = mobodB.getBodyTransform(state);
        const Real &true_dist =
            (X_GB * dsensor.sensorInA - Y_GB * dsensor.sensorInB).norm();
        // std::cout << "Sensor in A: " << dsensor.sensorInA << " Sensor in B: "
        // << dsensor.sensorInB << std::endl;
        const Real &error = square(true_dist - obs);
        // std::cout << "Error: " << true_dist - obs << " True: " << true_dist
        // << " Obs: " << obs << std::endl;
        goal += dsensor.weight * error;
        wtot += dsensor.weight;
      }
    }
  }

  goal /= (2 * wtot);
  // std::cout << "Goal: " << goal << std::endl;

  return 0;
}
// dgoal/dq = sum( wi * ai * dai/dq ) / sum(wi)
// This calculation is modeled after Peter Eastman's gradient implementation
// in ObservedPointFitter. It treats each dsensor Distance error as a
// potential energy function whose negative spatial gradient would be a spatial
// force F. We can then use Simbody's spatial force-to-generalized force method
// (using -F instead of F) to obtain the gradient in internal coordinates.
int DistanceSensors::calcGoalGradient(const State &state,
                                      Vector &gradient) const {
  const int np = getNumFreeQs();
  assert(gradient.size() == np);
  const SimbodyMatterSubsystem &matter = getMatterSubsystem();

  Vector_<SpatialVec> dEdR(matter.getNumBodies());
  dEdR = SpatialVec(Vec3(0), Vec3(0));
  // Loop over each body that has one or more active dsensors.
  Real wtot = 0;
  PerBodyDSensors::const_iterator bodyp = bodiesWithDSensors.begin();
  for (; bodyp != bodiesWithDSensors.end(); ++bodyp) {
    const MobilizedBodyIndex mobodIx = bodyp->first;
    const Array_<DSensorIx> &bodyDSensors = bodyp->second;
    const MobilizedBody &mobod = matter.getMobilizedBody(mobodIx);
    const Transform &X_GB = mobod.getBodyTransform(state);
    assert(bodyDSensors.size());
    // Loop over each dsensor on this body.
    for (unsigned m = 0; m < bodyDSensors.size(); ++m) {
      const DSensorIx mx = bodyDSensors[m];
      const DSensor &dsensor = dsensors[mx];
      assert(dsensor.bodyA == mobodIx); // better be on this body!
      const MobilizedBody &mobodB = matter.getMobilizedBody(dsensor.bodyB);
      const Real &obs = getObservation(getObservationIxForDSensor(mx));
      const Real &R_GO = observations[getObservationIxForDSensor(mx)];
      if (!isNaN(obs)) { // skip NaNs
        const Transform &Y_GB = mobodB.getBodyTransform(state);
        const Vec3 error = X_GB * dsensor.sensorInA - Y_GB * dsensor.sensorInB;
        const Real weight = dsensor.weight;
        const Real distanceError = error.norm() - obs;
        const Vec3 force_S = weight * distanceError * error.normalize();
        mobod.applyForceToBodyPoint(state, dsensor.sensorInA, force_S, dEdR);
        mobodB.applyForceToBodyPoint(state, dsensor.sensorInB, -force_S, dEdR);
        wtot += weight;
      }
    }
  }
  // Convert spatial forces dEdR to generalized forces dEdU.
  Vector dEdU;
  matter.multiplyBySystemJacobianTranspose(state, dEdR, dEdU);

  dEdU /= wtot;

  const int nq = state.getNQ();
  if (np == nq) // gradient is full length
    matter.multiplyByNInv(state, true, dEdU, gradient);
  else { // calculate full gradient; extract the relevant parts
    Vector fullGradient(nq);
    matter.multiplyByNInv(state, true, dEdU, fullGradient);
    for (Assembler::FreeQIndex fx(0); fx < np; ++fx)
      gradient[fx] = fullGradient[getQIndexOfFreeQ(fx)];
  }

  return 0;
}

// TODO: We want the constraint version to minimize the same goal as above. But
// there can never be more than six independent constraints on the pose of
// a rigid body; this method should attempt to produce a minimal set so that
// the optimizer doesn't have to figure it out.
int DistanceSensors::calcErrors(const State &state, Vector &err) const {
  return AssemblyCondition::calcErrors(state, err);
} // TODO

int DistanceSensors::calcErrorJacobian(const State &state,
                                       Matrix &jacobian) const {
  return AssemblyCondition::calcErrorJacobian(state, jacobian);
} // TODO
int DistanceSensors::getNumErrors(const State &state) const {
  return AssemblyCondition::getNumErrors(state);
} // TODO

// Run through all the DSensors to find all the bodies that have at least one
// active dsensor. For each of those bodies, we collect all its dsensors so that
// we can process them all at once. Active dsensors are those whose weight is
// greater than zero. Also, if we haven't been given any observation<->dsensor
// correspondence, we're going to assume they map directly, with each
// ObservationIx the same as its DSensorIx.
int DistanceSensors::initializeCondition() const {
  // Fill in missing observation information if needed.
  if (observation2dsensor.empty()) {
    const Array_<DSensorIx> zeroLength; // gcc doesn't like this as a temp
    const_cast<DistanceSensors &>(*this).defineObservationOrder(zeroLength);
  }

  bodiesWithDSensors.clear();
  for (DSensorIx mx(0); mx < dsensors.size(); ++mx) {
    const DSensor &dsensor = dsensors[mx];
    if (hasObservation(mx) && dsensor.weight > 0)
      bodiesWithDSensors[dsensor.bodyA].push_back(mx);
    // bodiesWithDSensors[dsensor.bodyB].push_back(mx);
  }
  return 0;
}

// Throw away the bodiesWithDSensors map.
void DistanceSensors::uninitializeCondition() const {
  bodiesWithDSensors.clear();
}
