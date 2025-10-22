#ifndef OPENSIM_KF_COMBO_INVERSE_KINEMATICS_TOOL_H_
#define OPENSIM_KF_COMBO_INVERSE_KINEMATICS_TOOL_H_
/* -------------------------------------------------------------------------- *
 *                    OpenSim:  KFComboInverseKinematicsTool.h                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Distribution Statement A – Approved for Public Release; Distribution is    *
 * Unlimited.                                                                 *
 *                                                                            *
 * KFComboInverseKinematicsTool is written by University of Eastern Finland        *
 * and based on the IMUInverseKinematicsTool code written by Ajay Seth.       *
 *                                                                            *
 * The project was supported by the Research Council of Finland.              *
 *                                                                            *
 * Copyright (c) 2024 University of Eastern Finland                           *
 * Author(s): Alexander Beattie and Matti Kortelainen                         *
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

#include "osimToolsDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/ModelDisplayHints.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/TableSource.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/TRCFileAdapter.h>
#include <OpenSim/Common/Reporter.h>
#include <OpenSim/Simulation/Model/Point.h>
#include <OpenSim/Simulation/OrientationsReference.h>
#include <OpenSim/Simulation/OpenSense/OpenSenseUtilities.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include <OpenSim/Simulation/OrientationsReference.h>
#include <OpenSim/Tools/InverseKinematicsToolBase.h>
#include <OpenSim/Tools/IMUInverseKinematicsTool.h>
 //include to realize KF
#include <OpenSim/Extension/Simulation/DistancesReference.h>
#include <OpenSim/Extension/Tools/DistanceInverseKinematicsTool.h>

#include <iostream>
#include <fstream>
#include <functional>
#include <vector>

namespace OpenSim {

//class Model;
// Only reason for this class to exist is to have nicer name in XML
//class OSIMTOOLS_API OrientationWeightSet : public Set<OrientationWeight> {
//    OpenSim_DECLARE_CONCRETE_OBJECT(
//            OrientationWeightSet, Set<OrientationWeight>);

//public:
    /** Use Super's constructors. */
//    using Super::Super;
    // default copy, assignment operator, and destructor
//    OrientationWeightSet() = default;
//    OrientationWeightSet(const OrientationWeightSet&) = default;
    //=============================================================================
//}; 
        //=============================================================================
//=============================================================================
/**
 * A Study that performs an Inverse Kinematics analysis with a given model.
 * Inverse kinematics is the solution of internal coordinates that poses
 * the model such that the body rotations (as measured by IMUs) affixed to the 
 * model minimize the weighted least-squares error with observations of IMU 
 * orientations in their spatial coordinates. 
 *
 */



class OSIMTOOLS_API KFComboInverseKinematicsTool
        : public InverseKinematicsToolBase {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            KFComboInverseKinematicsTool, InverseKinematicsToolBase);

public:
    OpenSim_DECLARE_PROPERTY(
        distances_file, std::string,
        "Name/path to a .sto file of sensor frame distances as quaternions.");
    OpenSim_DECLARE_PROPERTY(orientations_file, std::string,
        "Name/path to a .sto file of sensor frame orientations as quaternions.");

    OpenSim_DECLARE_PROPERTY(sensor_to_opensim_rotations, SimTK::Vec3,
            "Space fixed Euler angles (XYZ order) from IMU Space to OpenSim."
            " Default to (0, 0, 0).");
    OpenSim_DECLARE_PROPERTY(orientation_weights, OrientationWeightSet,
            "Set of orientation weights identified by orientation name with "
            "weight being a positive scalar. If not provided, all IMU "
            "orientations are tracked with weight 1.0.");
    OpenSim_DECLARE_PROPERTY(
      distance_weights, DistanceWeightSet,
      "Set of distance weights identified by distance name with "
      "weight being a positive scalar. If not provided, all Distance "
      "distances are tracked with weight 1.0.");
    OpenSim_DECLARE_PROPERTY(write_KF, bool, 
    "Write the means and covariances of Kalman smoother estimates up to 2nd order.");
    OpenSim_DECLARE_PROPERTY(num_threads, int, 
    "Number of threads to use in KF forward filtering.");
    OpenSim_DECLARE_PROPERTY(alpha, double, 
    "KF hyperparameter alpha.");
    OpenSim_DECLARE_PROPERTY(beta, double, 
    "KF hyperparameter beta.");
    OpenSim_DECLARE_PROPERTY(kappa, double, 
    "KF hyperparameter kappa. Special values: "
    "-1.337 => Set kappa equal to (3 - length of state vector)."
    "-4.337 => Set kappa equal to length of state vector.");
    OpenSim_DECLARE_PROPERTY(sgma2w, double, 
    "Scaling factor for process noise covariance matrix");
    OpenSim_DECLARE_PROPERTY(update_threshold, double, 
    "How much (in stds) the weight needs to differ from the average weight to force adaptation of process noise covariances.");
    OpenSim_DECLARE_PROPERTY(order, int, 
    "Largest order of position time derivatives to use in process model.");
    OpenSim_DECLARE_PROPERTY(lag_length, int, 
    "Number of future samples to use in backward pass of Kalman smoother.");
    OpenSim_DECLARE_PROPERTY(num_adaptive_samples, int, 
    "Number of samples to use in estimation of state residual covariance matrix.");
    OpenSim_DECLARE_PROPERTY(num_adaptive_weights, int, 
    "Number of weights to use in calculation of weight statistics.");
    OpenSim_DECLARE_PROPERTY(missing_data_scale, double, 
    "Scaling factor for observation noise covariance matrix elements in case of missing observations.");
    OpenSim_DECLARE_PROPERTY(imu_RMS_in_deg, SimTK::Vec3, 
    "RMS errors for each of 3 axes of orientation sensors.");
    OpenSim_DECLARE_PROPERTY(enable_resampling, bool, 
    "Resample sigma points after propagation through process model.");
    OpenSim_DECLARE_PROPERTY(enable_clamping, bool, 
    "Enforce inequality constraints on clamped coordinates.")
    OpenSim_DECLARE_PROPERTY(process_covariance_method, int, 
    "Model for process noise covariance matrix. "
    "0 = classic white noise process (Fioretti and Jetto, 1989); "
    "1 = scaled remainders of Taylor series expansion.");


    //=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    virtual ~KFComboInverseKinematicsTool();
    KFComboInverseKinematicsTool();
    KFComboInverseKinematicsTool(const std::string &setupFile);
    //--------------------------------------------------------------------------
    // INTERFACE
    //--------------------------------------------------------------------------

    //The roll, heading, and pitch RMS default values as given for Xsens MTW2-3A7G6
    bool run(bool visualizeResults, SimTK::Vector_<double> processCovScales = SimTK::Vector_<double>()) SWIG_DECLARE_EXCEPTION;
    bool run() override SWIG_DECLARE_EXCEPTION { 
        return run(false);
    };

    static TimeSeriesTable_<SimTK::Vec3>
        loadMarkersFile(const std::string& markerFile);

    void runInverseKinematicsWithOrientationsFromFile(Model& model,
            const std::string& quaternionStoFileName, const std::string &distancesFileName, bool visualizeResults=false, SimTK::Vector_<double> processCovScales = SimTK::Vector_<double>());

    std::tuple<std::map<std::string, int>, std::map<int, std::string>> CreateYMaps(Model model);

    double computeFactorial(int input);

    double probWithinInterval(double x1, double x2);

private:
    void constructProperties();


    void cholesky(const SimTK::Matrix& A, SimTK::Matrix& L) {
        int n = A.nrow();
        L.resize(n, n);
        L = 0; // Initialize L to zero

        for (int i = 0; i < n; ++i) {
            for (int j = 0; j <= i; ++j) {
                double sum = 0.0;
                for (int k = 0; k < j; ++k) {
                    sum += L(i, k) * L(j, k);
                }
                if (i == j) {
                    // Diagonal elements
                    L(i, j) = std::sqrt(A(i, i) - sum);
                } else {
                    // Off-diagonal elements
                    L(i, j) = (A(i, j) - sum) / L(j, j);
                }
            }
        }
    }


    SimTK::Quaternion_<double> quaternionMultiply(const SimTK::Vec4& q1, const SimTK::Vec4& q2) {
        // Extract components of the quaternions
        double w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
        double w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];

        // Compute the product using the quaternion multiplication formula
        double w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
        double x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
        double y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
        double z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;

        // Return the resulting quaternion as a Vec4
        return SimTK::Quaternion_<double>(w, x, y, z);
    }
    

//=============================================================================
};  // END of class IMUInverseKinematicsTool
//=============================================================================
} // namespace

#endif // OPENSIM_IMU_INVERSE_KINEMATICS_TOOL_H_
