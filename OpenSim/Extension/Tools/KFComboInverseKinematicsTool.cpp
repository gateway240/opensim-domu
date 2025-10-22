#include <queue>
#include <numeric>

#include "KFComboInverseKinematicsTool.h"

#include <OpenSim/Extension/Simulation/InverseKinematicsSolverKF.h>
#include <OpenSim/Common/Logger.h>


// KFComboInverseKinematicsTool methods
OpenSim::KFComboInverseKinematicsTool::KFComboInverseKinematicsTool()
        : OpenSim::InverseKinematicsToolBase() {
    OpenSim::KFComboInverseKinematicsTool::constructProperties();
}

OpenSim::KFComboInverseKinematicsTool::KFComboInverseKinematicsTool(const std::string& setupFile)
        : OpenSim::InverseKinematicsToolBase(setupFile, true) {
    OpenSim::KFComboInverseKinematicsTool::constructProperties();
    updateFromXMLDocument();
}

OpenSim::KFComboInverseKinematicsTool::~KFComboInverseKinematicsTool()
{
}

void OpenSim::KFComboInverseKinematicsTool::constructProperties()
{
    OpenSim::KFComboInverseKinematicsTool::constructProperty_sensor_to_opensim_rotations(
            SimTK::Vec3(0));
    OpenSim::KFComboInverseKinematicsTool::constructProperty_orientations_file("");
    OpenSim::OrientationWeightSet orientationWeights = OpenSim::OrientationWeightSet();
    OpenSim::KFComboInverseKinematicsTool::constructProperty_orientation_weights(orientationWeights);
    OpenSim::KFComboInverseKinematicsTool::constructProperty_distances_file("");
    OpenSim::DistanceWeightSet distanceWeights;
    OpenSim::KFComboInverseKinematicsTool::constructProperty_distance_weights(distanceWeights);
    OpenSim::KFComboInverseKinematicsTool::constructProperty_alpha(1.0);
    OpenSim::KFComboInverseKinematicsTool::constructProperty_beta(2.0);
    OpenSim::KFComboInverseKinematicsTool::constructProperty_kappa(0);
    OpenSim::KFComboInverseKinematicsTool::constructProperty_sgma2w(16.0);
    OpenSim::KFComboInverseKinematicsTool::constructProperty_update_threshold(0.5);
    OpenSim::KFComboInverseKinematicsTool::constructProperty_order(2);
    OpenSim::KFComboInverseKinematicsTool::constructProperty_lag_length(5);
    OpenSim::KFComboInverseKinematicsTool::constructProperty_num_adaptive_samples(10);
    OpenSim::KFComboInverseKinematicsTool::constructProperty_num_adaptive_weights(30);
    OpenSim::KFComboInverseKinematicsTool::constructProperty_missing_data_scale(100.0);
    OpenSim::KFComboInverseKinematicsTool::constructProperty_num_threads(3);
    OpenSim::KFComboInverseKinematicsTool::constructProperty_write_KF(true);
    OpenSim::KFComboInverseKinematicsTool::constructProperty_enable_resampling(true);
    OpenSim::KFComboInverseKinematicsTool::constructProperty_enable_clamping(true);
    OpenSim::KFComboInverseKinematicsTool::constructProperty_process_covariance_method(0);
    OpenSim::KFComboInverseKinematicsTool::constructProperty_imu_RMS_in_deg(SimTK::Vec3(0));
        
}

void OpenSim::KFComboInverseKinematicsTool::runInverseKinematicsWithOrientationsFromFile(
        OpenSim::Model& model, const std::string& orientationsFileName, const std::string &distancesFileName, bool visualizeResults, SimTK::Vector_<double> processCovScales) {

    // Ideally if we add a Reporter, we also remove it at the end for good hygiene but 
    // at the moment there's no interface to remove Reporter so we'll reuse one if exists
    const auto reporterExists = model.findComponent<OpenSim::TableReporter>("ik_reporter");

    bool reuse_reporter = true;
    OpenSim::TableReporter* ikReporter = nullptr;
    if (reporterExists == nullptr) {
        // Add a reporter to get IK computed coordinate values out
        ikReporter = new OpenSim::TableReporter();
        ikReporter->setName("ik_reporter");
        reuse_reporter = false;
    } 
	else {
		ikReporter = &model.updComponent<OpenSim::TableReporter>("ik_reporter");
	}
	
    auto coordinates = model.updComponentList<OpenSim::Coordinate>();

    // Hookup reporter inputs to the individual coordinate outputs
    // and lock coordinates that are translational since they cannot be
    for (auto& coord : coordinates) {
        ikReporter->updInput("inputs").connect(
                coord.getOutput("value"), coord.getName());
        if (coord.getMotionType() == OpenSim::Coordinate::Translational) {
            coord.setDefaultLocked(true);
        }
    }

    if (!reuse_reporter) {
        model.addComponent(ikReporter);
    }
    // DISTANCES
    OpenSim::TimeSeriesTable_<double> distTable(distancesFileName);
    log_info("Loading distances from '{}'...", distancesFileName);
    // Will maintain only data in time range specified by the tool
    // If unspecified {-inf, inf} no trimming is done
    distTable.trim(getStartTime(), getEndTime());

    OpenSim::DistancesReference dRefs(distTable, &get_distance_weights());
    SimTK::Vec2 timeRange = dRefs.getValidTimeRange();
    // END DISTANCES
    OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable(orientationsFileName);
    log_info("Loading orientations as quaternions from '{}'...",
        orientationsFileName);
    // Will maintain only data in time range specified by the tool
    // If unspecified {-inf, inf} no trimming is done
    // quatTable.trim(getStartTime(), getEndTime());
    quatTable.trim(timeRange[0], timeRange[1]);
    // quatTable.removeColumn("femur_r_imu");
    // quatTable.removeColumn("femur_l_imu");
    // Convert to OpenSim Frame
    const SimTK::Vec3& rotations = OpenSim::KFComboInverseKinematicsTool::get_sensor_to_opensim_rotations();
    SimTK::Rotation sensorToOpenSim = SimTK::Rotation(
            SimTK::BodyOrSpaceType::SpaceRotationSequence, 
            rotations[0], SimTK::XAxis, rotations[1], SimTK::YAxis, 
            rotations[2], SimTK::ZAxis);

    // Rotate data so Y-Axis is up
    OpenSim::OpenSenseUtilities::rotateOrientationTable(quatTable, sensorToOpenSim);
    //Trim to time window required by Tool
    quatTable.trim(getStartTime(), getEndTime());

    OpenSim::TimeSeriesTable_<SimTK::Rotation> orientationsData =
        OpenSim::OpenSenseUtilities::convertQuaternionsToRotations(quatTable);

    OpenSim::OrientationsReference oRefs(orientationsData, &get_orientation_weights());



    SimTK::Array_<OpenSim::CoordinateReference> coordinateReferences;


    // visualize for debugging
    //if (visualizeResults)
    //    model.setUseVisualizer(true);
    SimTK::State& s0 = model.initSystem();

    OpenSim::AnalysisSet& analysisSet = model.updAnalysisSet();
    analysisSet.begin(s0);


    double t0 = s0.getTime();

    // create the solver given the input data
    const double accuracy = 1e-4;
    OpenSim::InverseKinematicsSolverKF ikSolver(model, nullptr,
            std::make_shared<OpenSim::OrientationsReference>(oRefs),
            std::make_shared<DistancesReference>(dRefs),
        coordinateReferences);
    ikSolver.setAccuracy(accuracy);

    auto& times = oRefs.getTimes();
    std::shared_ptr<OpenSim::TimeSeriesTable> modelOrientationErrors(
            get_report_errors() ? new OpenSim::TimeSeriesTable()
                                : nullptr);
    s0.updTime() = times[0];
    ikSolver.assemble(s0);
    // ikSolver.track(s0); // solve the initial state; assemble() was called before
	log_info("Solved at time: {} s", times[0]);
    // Create place holder for orientation errors, populate based on user pref.
    // according to report_errors property
    const int nOSensors = ikSolver.getNumOrientationSensorsInUse();
    const int nDSensors = ikSolver.getNumDistanceSensorsInUse();
    const int nTotalSensors = nOSensors + nDSensors;

    const int sizeOrientations = nOSensors*3;
    const int sizeDistances = nDSensors*1;
    const int sizeObs = sizeOrientations + sizeDistances;

    SimTK::Array_<double> weightsOrientations;
    SimTK::Array_<double> weightsDistances;

    dRefs.getWeights(s0, weightsDistances);
    oRefs.getWeights(s0, weightsOrientations);


    std::cout << "Observation Size: " << sizeObs << std::endl;


    SimTK::Array_<double> orientationErrors(nOSensors, 0.0);

    if (get_report_errors()) {
        SimTK::Array_<std::string> labels;
        for (int i = 0; i < nOSensors; ++i) {
            labels.push_back(ikSolver.getOrientationSensorNameForIndex(i));
        }
        modelOrientationErrors->setColumnLabels(labels);
        modelOrientationErrors->updTableMetaData().setValueForKey<std::string>(
                "name", "OrientationErrors");
        ikSolver.computeCurrentOrientationErrors(orientationErrors);
    }
    //if (visualizeResults) {
    //    model.getVisualizer().show(s0);
    //    model.getVisualizer().getSimbodyVisualizer().setShowSimTime(true);
    //}

    // Solve the states with Unscented Kalman Filter
	//int step = 0;	
	
	//log_info("Managed to get to KFTool.");


    // Make mapping between the IMUs in data file and in the model
    // int const ny = ikSolver.getNumOrientationSensorsInUse();    //number of sensors
    SimTK::Array_<std::string> oRefNames = oRefs.getNames();


    std::cout << "Orientation Names: " << oRefNames << std::endl;;
    std::cout << "Orientation Weights: " << weightsOrientations << std::endl;
    SimTK::Array_<std::string> oModelNames;
    for (int ii = 0; ii < nOSensors; ii++) { 
        oModelNames.push_back(ikSolver.getOrientationSensorNameForIndex(ii));
    }
    std::map<int, int> oMapFromDataToModel;
    std::map<int, int> oMapFromModelToData;
    for (int ii = 0; ii < oRefs.getNumRefs(); ii++) {
        for (int jj = 0; jj < nOSensors; jj++) {
            if (oRefNames[ii] == oModelNames[jj]) {
                oMapFromDataToModel.insert(std::pair<int, int>(ii, jj));
                oMapFromModelToData.insert(std::pair<int, int>(jj, ii));
                break;
            }
        }
    }

    SimTK::Array_<std::string> dRefNames = dRefs.getNames();
        
    std::cout << "Distance Names: " << dRefNames << std::endl;;
    std::cout << "Distance Weights: " << weightsDistances << std::endl;

    std::map<int, int> dMapFromDataToModel;
    std::map<int, int> dMapFromModelToData;
    SimTK::Array_<std::string> dModelNames;
    for (int ii = 0; ii < nDSensors; ii++) { 
        dModelNames.push_back(ikSolver.getDistanceSensorNameForIndex(ii));
    }
    for (int ii = 0; ii < dRefs.getNumRefs(); ii++) {
        for (int jj = 0; jj < nDSensors; jj++) {
            if (dRefNames[ii] == dModelNames[jj]) {
                dMapFromDataToModel.insert(std::pair<int, int>(ii, jj));
                dMapFromModelToData.insert(std::pair<int, int>(jj, ii));
                break;
            }
        }
    }


    // Create mappings between continuous state variables between OpenSim and Simbody
    std::tuple<std::map<std::string, int>, std::map<int, std::string>> mappings = OpenSim::KFComboInverseKinematicsTool::CreateYMaps(model);
    std::map<std::string, int> yMapFromOpenSimToSimbody = std::get<0>(mappings);
    std::map<int, std::string> yMapFromSimbodyToOpenSim = std::get<1>(mappings);

    // Find locked coordinates (we cannot change these in KF)

    std::map<int, int> simbodyIndexTypeMap;
    std::map<int, int> yMapFromSimbodyToEigen;
    std::map<int, int> yMapFromEigenToSimbody;

    int iqx;
    int nqf = 0;    //number of free and clamped q's
    int nuf = 0;
    double deltaTime = 1.0 / oRefs.getSamplingFrequency();
    for (const auto& coord : model.getComponentList<OpenSim::Coordinate>()) {
        iqx = yMapFromOpenSimToSimbody[coord.getStateVariableNames()[0]];
        if (coord.getMotionType() == OpenSim::Coordinate::MotionType::Translational) {
            simbodyIndexTypeMap.insert(std::pair<int, int>(iqx, 0));    //translational coordinate
            log_info("{} is translational coordinate, will be ignored in KF.", coord.getName());
        }
        else if (coord.getLocked(s0)) {
            simbodyIndexTypeMap.insert(std::pair<int, int>(iqx, 0));    //locked coordinate
            log_info("{} is locked, will be ignored in KF.", coord.getName());
        }
        else if (coord.isDependent(s0)) {
            simbodyIndexTypeMap.insert(std::pair<int, int>(iqx, 0));    //dependent coordinate
            log_info("{} is dependent coordinate, will be ignored in KF.", coord.getName());
        }
        else if (coord.getClamped(s0)) {
            simbodyIndexTypeMap.insert(std::pair<int, int>(iqx, 1));    //clamped coordinate
            nqf++;
        }
        else {
            simbodyIndexTypeMap.insert(std::pair<int, int>(iqx, 2));    //free coordinate
            nqf++;
        }
    }
    nuf = nqf;
    log_info("{} coordinates will be used in KF", nqf);

    int ie = 0;
    for (std::map<int, int>::iterator it = simbodyIndexTypeMap.begin(); it != simbodyIndexTypeMap.end(); ++it) {
        if (it->second != 0) {
            yMapFromSimbodyToEigen.insert(std::pair<int, int>(it->first, ie));
            yMapFromEigenToSimbody.insert(std::pair<int, int>(ie, it->first));
            ie++;
        }
    }


    SimTK::Matrix Q((nqf + get_order()*nuf), (nqf + get_order()*nuf));
    Q = 0.0;
    SimTK::Matrix F((nqf + get_order()*nuf), (nqf + get_order()*nuf));
    F = 0.0;
    // std::queue<SimTK::Matrix>* stateMeansBuffer = new std::queue<SimTK::Matrix>();

    // If the process covariance scale factors are not provided, use simply ones
    if (processCovScales.size() == 0) {
        processCovScales = SimTK::Vector_<double>(get_order(), 1.0);
        log_info("Process covariance scales not provided, using ones instead.");
    }
    
    log_info("Got inside KFTOOL");

    SimTK::State ss;
    ss = SimTK::State(s0);

    // {
    //     std::unique_lock<std::mutex> lock(*fwdBwdMutex);
    //     ss = SimTK::State(s);
    // }

    // auto times = oRefs.getTimes();
    double alpha = get_alpha();
    double beta = get_beta();
    double kappa = get_kappa();
    int order = get_order();
    double sgma2w = get_sgma2w();
    double missingDataScale = get_missing_data_scale();

    SimTK::Vec3 imuRMSinDeg = get_imu_RMS_in_deg();
    int num_cores = get_num_threads();
    
    // array for orientations computed by the model; REMOVE _<double> IF CAUSES ERROR
    SimTK::Array_<SimTK::Rotation_<double>> modelOrientations(nOSensors); 
    SimTK::Array_<SimTK::Rotation_<double>> observedOrientations(nOSensors); // array for observations (IMU orientations)
    SimTK::Array_<double> modelDistances(nDSensors);
    SimTK::Array_<double> observedDistances(nDSensors);

    int const nq = ss.getNQ(); // number of generalized positions (joint angles)
    int const nu = ss.getNU(); // number of generalized velocities (joint angular velocities)
    int const nr = std::min(nq, nu); // probably not needed, usually nq >= nu

    // RMS errors for observation noise covariance
    
    double xAxisRMS = imuRMSinDeg(0) * (SimTK::Pi / 180); // roll RMS error (x-axis)
    double yAxisRMS = imuRMSinDeg(1) * (SimTK::Pi / 180); // heading RMS error (y-axis)
    double zAxisRMS = imuRMSinDeg(2) * (SimTK::Pi / 180); // pitch RMS error (z-axis)

    // Coefficients for process model f()
    // double deltaTime = 1.0 / oRefs.getSamplingFrequency();
    SimTK::Matrix fCoeffs(order+1, order+1);
    fCoeffs = 0.0;
    for (int irow = 0; irow <= order; irow++) {
        for (int icol = irow; icol <= order; icol++) {
            double denum = OpenSim::KFComboInverseKinematicsTool::computeFactorial(icol-irow);
            fCoeffs(irow, icol) = std::pow(deltaTime, (icol-irow)) / denum;
        }
    }
    std::cout << "fCoeffs: " << fCoeffs << std::endl;

    // Coefficients for process noise covariance matrix Q
    SimTK::Matrix QCoeffs(order+1, order+1);
    
    // std::cout << QCoeffs << std::endl;
    
    if (get_process_covariance_method() == 0) {
    // classic approach of Fioretti and Jetto, 1989 (no scaling tricks by default; should use ones)    
        log_info("Using method of Fioretti and Jetto, 1989.");
        for (int irow = 0; irow <= order; irow++) {
            for (int icol = 0; icol <= order; icol++) {
                double denum1 = OpenSim::KFComboInverseKinematicsTool::computeFactorial(order-irow);
                double denum2 = OpenSim::KFComboInverseKinematicsTool::computeFactorial(order-icol);
                int deltaPower = (order-irow) + (order-icol);
                QCoeffs(irow, icol) = processCovScales(irow) * processCovScales(icol) * std::pow(deltaTime, (deltaPower+1)) / (denum1 * denum2 * (deltaPower+1));
            }
        }
    }

    else if (get_process_covariance_method() == 1) {
    // approach using the Taylor remainders as white noise multipliers (plus additional scale factors)        
        log_info("Using Taylor remainders.");
        for (int irow = 0; irow <= order; irow++) {
            for (int icol = 0; icol <= order; icol++) {
                double denum1 = OpenSim::KFComboInverseKinematicsTool::computeFactorial(order+1-irow);
                double denum2 = OpenSim::KFComboInverseKinematicsTool::computeFactorial(order+1-icol);
                int deltaPower = (order+1-irow) + (order+1-icol);
                QCoeffs(irow, icol) = processCovScales(irow) * processCovScales(icol) *
                                    std::pow(deltaTime, (deltaPower)) / (denum1 * denum2);
            }
        }
    }

    // Construct covariance matrix Q for process noise
    //Eigen::MatrixXd Q((nqf + order*nuf), (nqf + order*nuf));
    {
        // std::unique_lock<std::mutex> lock(*qMutex);
        Q.setToZero();
        log_info("Building Q");
        // This is syntax for identity matrix in simbody
        SimTK::Matrix IMat(nuf,nuf);
        IMat = 1;
        for (int irow = 0; irow <= order; irow++) {
            for (int icol = 0; icol <= order; icol++) {
                Q((irow*nuf), (icol*nuf), nuf, nuf) = sgma2w * QCoeffs(irow, icol) * IMat;
            }
        }
    }
    // std::cout << "Q: " << Q << std::endl;
        
    // Construct covariance matrix for observation errors
    SimTK::Matrix R(sizeObs, sizeObs);
    log_info("Building R");
    R.setToZero();

    const double distanceRMSinM = 0.01;

    for (int ii = 0; ii < sizeOrientations; ii++) {
        if (ii % 3 == 0) {      //x-axis
            R(ii, ii) = std::pow(xAxisRMS, 2);
        } 
		else if (ii % 3 == 1) { // y-axis
            R(ii, ii) = std::pow(yAxisRMS, 2);
        } 
		else {                //z-axis
            R(ii, ii) = std::pow(zAxisRMS, 2);        
        }
    }
    for (int ii = sizeOrientations; ii < sizeObs; ii++) {
        R(ii, ii) = std::pow(distanceRMSinM, 2);
    }

    log_info("Building R0");
    SimTK::Matrix R0(sizeObs, sizeObs);
    R0 = R;

    // Construct covariance matrix of state
    SimTK::Matrix P((nqf + (order*nuf)), (nqf + (order*nuf)));
    P = Q;
    // std::cout << "P: " << std::endl;
    // std::cout << P << std::endl;
    
    // Construct the state vector
    // Vector(size, initial value)
    SimTK::Vector x(nqf + (order*nuf), 0.0);
    SimTK::Vector xx(nqf + (order*nuf), 0.0);

    SimTK::Vector_<double> q(nq);
    q = ss.getQ();
    SimTK::Vector_<double> u(nu);
    u = ss.getU();

    for (std::map<int, int>::iterator it = yMapFromSimbodyToEigen.begin(); it != yMapFromSimbodyToEigen.end(); ++it) {
        x(it->second) = q(it->first);
    }
    for (std::map<int, int>::iterator it = yMapFromSimbodyToEigen.begin();
            it != yMapFromSimbodyToEigen.end(); ++it) {
        x((it->second)+nqf) = u(it->first);
    }

    // Construct the process model matrix
    //Eigen::MatrixXd F((nqf + order*nuf), (nqf + order*nuf));
    {
        // std::unique_lock<std::mutex> lock(*qMutex);
        F.setToZero();
        log_info("Building F");
        SimTK::Matrix IMat(nuf,nuf);
        IMat = 1;
        for (int irow = 0; irow <= order; irow++) {
            for (int icol = 0; icol <= order; icol++) {
                F((irow*nuf), (icol*nuf), nuf, nuf) = fCoeffs(irow, icol) * IMat;
            }
        }
    }
    // std::cout << "F: " << F << std::endl;

    // For data
    SimTK::Vector_<double> y(sizeObs); 

    // If the user wants kappa to be equal to state vector length (or 3-length)
    if (std::abs(kappa+1.337) < 1e-5 ) {
        kappa = -1.0 * ((double) nqf + (order*nuf) - 3);
    }
    else if (std::abs(kappa+4.337) < 1e-5) {        
        kappa = (double) nqf + (order*nuf);
    }
    log_info("kappa set to {}", kappa);

    double const lambda = (std::pow(alpha, 2) * (nqf + (order*nuf) + kappa)) - (nqf + (order*nuf));
    double const W0m = lambda / (nqf + (order*nuf) + lambda);
    double const W0c = W0m + (1 - std::pow(alpha, 2) + beta);
    double const Wi = 1 / (2 * (nqf + (order*nuf) + lambda));
    SimTK::Vector Wm(2 * (nqf + (order*nuf)) + 1);
    SimTK::Vector Wc(2 * (nqf + (order*nuf)) + 1);
    Wm = Wi;
    Wc = Wi;
    Wm(0) = W0m;
    Wc(0) = W0c;
    // std::cout << "Wm: " << Wm << std::endl;
    // std::cout << "Wc: " << Wc << std::endl;

    SimTK::Matrix SS(nqf + (order*nuf), nqf + (order*nuf));
    SimTK::Matrix Sigmas(nqf + (order*nuf), 2 * (nqf + (order*nuf)) + 1);
    SimTK::Matrix Sigmas2(nqf + (order*nuf), 2 * (nqf + (order*nuf)) + 1);
    SimTK::Matrix Sigmas2props(sizeObs, 2 * (nqf + (order*nuf)) + 1);

    SimTK::Vector_<double> qdot;
    SimTK::Vector xsave(nqf + (order*nuf));

    SimTK::Vector ysave(sizeObs);
    SimTK::Vector ydiff(sizeObs);

    int ycounter;
    SimTK::Matrix Py(sizeObs, sizeObs);
    SimTK::Matrix Pxy(nqf + (order*nuf), sizeObs);
    SimTK::Matrix K(nqf + (order*nuf), sizeObs);
    SimTK::Matrix C((nqf + (order*nuf)), (nqf + (order*nuf)));
    SimTK::Matrix stateMean(nqf+(order*nuf), 1);
    SimTK::Matrix A;        //matrix for holonomic constraints
    SimTK::Vector_<double> qerr;
	SimTK::Vec3 angle_vector;
    
    // Modifications to compute average orientations, comparisons between orientations, etc.
    SimTK::Quaternion_<double> simTKquat(1, 0, 0, 0);
    // std::vector<SimTK::Quaternion_<double>*> arr_simTKquat;
    SimTK::Quaternion_<double> dummyquat(1, 0, 0, 0);
    SimTK::Quaternion_<double> dataMinusMeanQuat(1, 0, 0, 0);
    std::vector<SimTK::Quaternion_<double>> dataOVector(nOSensors, dummyquat);
    std::vector<double> dataDVector(nDSensors, 0.0);
    std::vector<SimTK::Quaternion_<double>> expectedOVector(nOSensors, dummyquat);
    std::vector<double> expectedDVector(nDSensors, 0.0);
    std::vector<std::vector<SimTK::Quaternion_<double>>> Sigmas2Orientations( 2 * (nqf + (order*nuf)) + 1, dataOVector);
    std::vector<std::vector<double>> Sigmas2Distances( 2 * (nqf + (order*nuf)) + 1, dataDVector);
    std::vector<std::vector<SimTK::Quaternion_<double>>> Sigmas2OminusMean( 2 * (nqf + (order*nuf)) + 1, dataOVector);
    SimTK::Matrix M(4, 2 * (nqf + (order*nuf)) + 1);
    SimTK::Matrix MM(4, 4);

    SimTK::Vector_<std::complex<double>> eigenVals;
    SimTK::Matrix_<std::complex<double>> eigenVecs;

    int iMax = 0;
    double maxEigenVal = 0.0;
    SimTK::Vec4 dummy4vector = simTKquat.asVec4();

    
	int step = 0;

    
	log_info("Got to the beginning of for loop.");

    //log_info("before for loop, x(0, 0) = {}", (double)x(0, 0));

    for (auto time : times) { 
        // Step 0. Cholesky factorization of state covariance            
        ss.updTime() = time;
        ss.updQ() = q;
        ss.updU() = u;

        // Steps 0-4 for linear model with Gaussian noise
        x = F * x;
        C = P * F.transpose();
        P = (F * P * F.transpose()) + Q;

        // Resample the sigma points after propagating through process model (optional)
        // get_enable_resampling() == true
        // TODO: Reenable this
        if (true) {
            // SimTK::FactorLU lu((nqf + (order*nuf) + lambda) * P);  // perform LU factorization 
            // lu.getL(SS);
            SimTK::Matrix L;
            cholesky((nqf + (order*nuf) + lambda) * P, L);
            SS = L;
            Sigmas2.updCol(0) = x;                
            for (int ii = 1; ii < (nqf + (order*nuf) + 1); ii++) {
                Sigmas2.updCol(ii) = x + SS.col(ii - 1);
            }
            for (int ii = (nqf + (order*nuf) + 1); ii < (2 * (nqf + (order*nuf)) + 1); ii++) {
                Sigmas2.updCol(ii) = x - SS.col(ii - (nqf + (order*nuf) + 1));
            }
            // std::cout << "SS: " << SS << std::endl;
            // std::cout << "x: " << x << std::endl;
        }
        // std::cout << "Sigmas2: " << Sigmas2 << std::endl;

        // Step 5. Propagate a priori sigma points of current step through
        // observation model
        xsave = x;
        for (int icol = 0; icol < (2 * (nqf + (order*nuf)) + 1); icol++) {
            xx = Sigmas2.col(icol);
            for (std::map<int, int>::iterator it = yMapFromEigenToSimbody.begin(); it != yMapFromEigenToSimbody.end(); ++it) {
                q(it->second) = xx(it->first);
            }
            // std::cout << "xx: " << xx << std::endl;
            // std::cout << "q: " << q << std::endl;
            ss.updTime() = time;
            ss.updQ() = q;
            // arr_ss[ithr]->updTime() = time;
            // arr_ss[ithr]->updQ() = (*(arr_q[ithr]));
            if (order > 0) {
                 for (std::map<int, int>::iterator it = yMapFromEigenToSimbody.begin(); it != yMapFromEigenToSimbody.end(); ++it) {
                    u(it->second) = xx((it->first)+nqf);
                }
                ss.updU() = u;
            }
            //method added to AssemblySolver (like in Kalman Smoother)
            ikSolver.setState(ss);
            ikSolver.computeCurrentSensorOrientations(modelOrientations);  
            ikSolver.computeCurrentSensorDistances(modelDistances);           

            for (int yy = 0; yy < nOSensors; yy++) {
                Sigmas2Orientations[icol][yy] = modelOrientations[yy].convertRotationToQuaternion();
                // std::cout << "Sigmas2Orientations: " << Sigmas2Orientations[icol][yy] << std::endl;
             }
            //  std::cout << "Sigmas2Distances: " << std::endl;
            for (int yy = 0; yy < nDSensors; yy++) {
                Sigmas2Distances[icol][yy] = modelDistances[yy];
                // std::cout << Sigmas2Distances[icol][yy] << " ";
             }
            //  std::cout << std::endl;
        }
        // std::cout << "Sigmas2Orientations: " << Sigmas2Orientations << std::endl;

        // Step 6. Calculate expected observation of current step
        
        for (int yy = 0; yy < nOSensors; yy++) {
            for (int icol = 0; icol < (2 * (nqf + (order*nuf)) + 1); icol++) {
                M(0, icol) = Sigmas2Orientations[icol][yy][0];
                M(1, icol) = Sigmas2Orientations[icol][yy][1];
                M(2, icol) = Sigmas2Orientations[icol][yy][2];
                M(3, icol) = Sigmas2Orientations[icol][yy][3];
            }
            // Turn vector into diagonal identity
            SimTK::Matrix wmDiag(Wm.size(),Wm.size());
            wmDiag.setToZero();
            wmDiag.updDiag() = Wm;
            // std::cout << "Wm diag: " << wmDiag << std::endl;
            MM = M * wmDiag * ~M;
            // std::cout << "MM: " << MM << std::endl;
            SimTK::Eigen  es(MM);   // setup the eigen system 
            es.getAllEigenValuesAndVectors(eigenVals,eigenVecs);

            // std::cout << "eigenVals: " << eigenVals << std::endl;
            // std::cout << "eigenVecs: " << eigenVecs << std::endl;

            iMax = 0;
            double eVal = eigenVals(0).real();
            for(int i=0;i<eigenVals.size(); i++ ) {
                eVal = eigenVals(i).real();
                if (eVal > maxEigenVal){
                    maxEigenVal = eVal;
                    iMax = i;
                }
            } 
            // std::cout << "iMax: " << iMax << std::endl;
            expectedOVector[yy] = SimTK::Quaternion_<double>(eigenVecs(0, iMax).real(),
                                                    eigenVecs(1, iMax).real(),
                                                    eigenVecs(2, iMax).real(),
                                                    eigenVecs(3, iMax).real());
  
            // std::cout << "expectedOVector: " << expectedOVector[yy] << std::endl;
        }
        for (int yy = 0; yy < nDSensors; yy++) {
            double sum = 0;
            for (int icol = 0; icol < (2 * (nqf + (order*nuf)) + 1); icol++) {
                sum += Sigmas2Distances[icol][yy];
            }
            double mean = sum / (2 * (nqf + (order*nuf)) + 1);
            // double sum = std::accumulate(Sigmas2Distances[yy].begin(), Sigmas2Distances[yy].end(), 0.0);
            // double mean = sum / Sigmas2Distances[yy].size();
            // expectedDVector[yy] = mean;
            expectedDVector[yy] = mean;
            // std::cout << "Mean: " << mean << std::endl;
            // std::cout << "Sigmas2Distances[yy]: " << std::endl;
            // for (const auto& element : Sigmas2Distances[yy]) {
            //     std::cout << element << " ";
            // }
            // std::cout << std::endl; // Print a newline at the end
        }
        // log_info("Made it to step 7!");
        //Step 7. Subtract expected mean orientation from propagated sigma points
        for (int icol = 0; icol < (2 * (nqf + (order*nuf)) + 1); icol++) {
            for (int iy = 0; iy < nOSensors; iy++) {
                SimTK::Vec4 sigmas2OrientationsVec = Sigmas2Orientations[icol][iy].asVec4();
                SimTK::Vec4 conjugate = SimTK::Vec4(
                    sigmas2OrientationsVec[0],
                    -sigmas2OrientationsVec[1],
                    -sigmas2OrientationsVec[2],
                    -sigmas2OrientationsVec[3]
                );
                SimTK::Vec4 oVec = expectedOVector[iy].asVec4();
                // std::cout << "sigmas2OVec: " << sigmas2OrientationsVec << std::endl;
                // std::cout << "Exp O: " << oVec<< std::endl;
                // std::cout << "Conjugate: " << conjugate << std::endl;

                Sigmas2OminusMean[icol][iy] = quaternionMultiply(oVec,conjugate);
               
                simTKquat = SimTK::Quaternion_<double>(
                                    Sigmas2OminusMean[icol][iy][0],
                                    Sigmas2OminusMean[icol][iy][1],
                                    Sigmas2OminusMean[icol][iy][2],
                                    Sigmas2OminusMean[icol][iy][3]);
                // std::cout << "simTKquat: " << simTKquat << std::endl;
                angle_vector = SimTK::Rotation_<double>(simTKquat).convertThreeAxesRotationToThreeAngles(
                        SimTK::BodyOrSpaceType::SpaceRotationSequence, SimTK::XAxis, SimTK::YAxis, SimTK::ZAxis);

                for (int kk = 0; kk < 3; kk++) {
                    y(iy * 3 + kk) = (double) (angle_vector)(kk);
                }
            }
            int countDist = 0;
            for (int iy = sizeOrientations; iy < sizeObs; iy++) {
                y(iy) = (Sigmas2Distances[icol][countDist] - expectedDVector[countDist]);
                countDist++;
            }           
            // y(iy) = Sigmas2Distances[icol][iy] - expectedDVector[iy];
           
            // std::cout << "y: " << y << std::endl;
            Sigmas2props.updCol(icol) = y;
            // Sigmas2props.updCol(icol) = 
        }
        // std::cout << "Sigmas2props: " << Sigmas2props << std::endl;
  
        // log_info("Made it to step 8!");

        // Step 8. Subtract expected mean orientation from data
        R = R0; //restore covariance of observations; gets modified if missing data

        oRefs.getValuesAtTime(time, observedOrientations);

        for (std::map<int, int>::iterator it = oMapFromDataToModel.begin();
                it != oMapFromDataToModel.end(); ++it) {
            dummy4vector = observedOrientations[it->first].convertRotationToQuaternion().asVec4() * weightsOrientations[it->first];
            if (SimTK::isNaN(dummy4vector(0))) {
                dataOVector[it->second] = SimTK::Quaternion_<double>(1, 0, 0, 0);
            } 
            else {
                if (weightsOrientations[it->first] == 0) {
                    expectedOVector[it->second] = SimTK::Quaternion_<double>(1, 0, 0, 0);
                }
                dataOVector[it->second] = SimTK::Quaternion_<double>(
                        dummy4vector(0), dummy4vector(1), dummy4vector(2),
                        dummy4vector(3));
            }
        }

        dRefs.getValuesAtTime(time, observedDistances);
        for (std::map<int, int>::iterator it = dMapFromDataToModel.begin(); it != dMapFromDataToModel.end(); ++it) {
            if (SimTK::isNaN(observedDistances[it->first])) {
                dataDVector[it->second] = 0.0;
                R(sizeOrientations+(it->second), sizeOrientations+(it->second)) *= missingDataScale;
            }
            else if (observedDistances[it->first] <= 0.0) {
                dataDVector[it->second] = 0.0;
                R(sizeOrientations+(it->second), sizeOrientations+(it->second)) *= missingDataScale;
                log_info("Distance data at index {} is leq 0.0; this should not be possible!", it->first);
            }
            else {
                dataDVector[it->second] = observedDistances[it->first] * weightsDistances[it->first];
                expectedDVector[it->second] *= weightsDistances[it->first];
            }                
        }

        // log_info("Mid step 8!");
        for (int iy = 0; iy < nOSensors; iy++) {
            if (dataOVector[iy][0] == 0 && dataOVector[iy][1] == 0 &&
                    dataOVector[iy][2] == 0 && dataOVector[iy][3] == 0) {
                for (int kk = 0; kk < 3; kk++) {
                    ydiff(iy * 3 + kk) = 0.0;
                    R(iy * 3 + kk, iy * 3 + kk) *= missingDataScale;
                }
            } 
            else if (std::isnan(dataOVector[iy][0]) &&
                        std::isnan(dataOVector[iy][1]) &&
                        std::isnan(dataOVector[iy][2]) &&
                        std::isnan(dataOVector[iy][3])) {
                for (int kk = 0; kk < 3; kk++) {
                    ydiff(iy * 3 + kk) = 0.0;
                    R(iy * 3 + kk, iy * 3 + kk) *= missingDataScale;
                }                    
            }
            else {

                SimTK::Vec4 conjugate = SimTK::Vec4(
                     dataOVector[iy][0],
                    -dataOVector[iy][1],
                    -dataOVector[iy][2],
                    -dataOVector[iy][3]
                );
                SimTK::Vec4 oVec = expectedOVector[iy].asVec4();
                dataMinusMeanQuat = quaternionMultiply(oVec,conjugate);
                // dataMinusMeanQuat[0] = expectedOVector[iy][0] * conjugate[0];
                // dataMinusMeanQuat[1] = expectedOVector[iy][1] * conjugate[1];
                // dataMinusMeanQuat[2] = expectedOVector[iy][2] * conjugate[2];
                // dataMinusMeanQuat[3] = expectedOVector[iy][3] * conjugate[3];

                // dataMinusMeanQuat = dataOVector[iy].conjugate() *
                // expectedOVector[iy];
                simTKquat = SimTK::Quaternion_<double>(
                        dataMinusMeanQuat[0], dataMinusMeanQuat[1],
                        dataMinusMeanQuat[2], dataMinusMeanQuat[3]);
                angle_vector =
                        SimTK::Rotation_<double>(simTKquat)
                                .convertThreeAxesRotationToThreeAngles(
                                        SimTK::BodyOrSpaceType::
                                                SpaceRotationSequence,
                                        SimTK::XAxis, SimTK::YAxis,
                                        SimTK::ZAxis);
                for (int kk = 0; kk < 3; kk++) {
                    ydiff(iy * 3 + kk) = (double)angle_vector(kk);
                    // std::cout << iy;
                }
            }
        }
        // std::cout << ydiff << std::endl;
        int countDist = 0;
        for (int iy = sizeOrientations; iy < sizeObs; iy++) {
            ydiff(iy) = (dataDVector[countDist] - expectedDVector[countDist]);
            // std::cout << "data: " << dataDVector[countDist] << " expected: " << expectedDVector[countDist] << " diff: " << ydiff(iy) << std::endl;
            countDist++;
            // std::cout << iy;
        }

        // log_info("completed step 8");
        // std::cout << "Sigmas2props: " << std::endl;
        // std::cout << Sigmas2props << std::endl;

        // Step 9. Calculate covariance of expected orientations
        Py = R + W0c * (Sigmas2props.col(0) * Sigmas2props.col(0).transpose());
        for (int ii = 1; ii < (2 * (nqf + (order*nuf)) + 1); ii++) {
            Py += Wi * (Sigmas2props.col(ii) * Sigmas2props.col(ii).transpose());
        }
        // log_info("completed step 9");
        
        // Step 10. Calculate cross-covariance of expected orientations and a
        // priori state mean
        for (int jj = 0; jj < (2 * (nqf + (order*nuf)) + 1); jj++) {
            Sigmas2.updCol(jj) -= xsave;
        }
        Pxy = W0c * (Sigmas2.col(0) * Sigmas2props.col(0).transpose());
        for (int ii = 1; ii < (2 * (nqf + (order*nuf)) + 1); ii++) {
            // std::cout << "Sigmas2 col: " << Sigmas2.col(ii) << std::endl;
            Pxy += Wi * (Sigmas2.col(ii) * Sigmas2props.col(ii).transpose());
        }
        // std::cout << "Pxy" << Pxy << std::endl;
        // log_info("completed step 10");
        
        // Step 11. Calculate Kalman gain

        // QR 

        SimTK::FactorQTZ cqtz(Py);
        SimTK::Matrix invQTZ;
        cqtz.inverse(invQTZ);
        // std::cout << "invQTZ" << invQTZ << std::endl;
        // std::cout << "Sigmas2props: " << Sigmas2props.nrow() << "x" << Sigmas2props.ncol() << std::endl;
        // std::cout << "Sigmas2: " << Sigmas2.nrow() << "x" << Sigmas2.ncol() << std::endl;

        // SVD
        // SimTK::FactorSVD csvd(Py);
        // SimTK::Matrix invSVD;
        // csvd.inverse(invSVD);
        // std::cout << "Py: " << Py.nrow() << "x" << Py.ncol() << std::endl;
        // std::cout << "Pxy: " << Pxy.nrow() << "x" << Pxy.ncol() << " invQTZ: " << invQTZ.nrow() << "x" << invQTZ.ncol() << std::endl;
        // std::cout << "Pxy Matrix: " << std::endl;
        // std::cout << Pxy << std::endl;
        // std::cout << "Inv Matrix: " << std::endl;
        // std::cout << invQTZ << std::endl;
        K = Pxy * invQTZ; 
        // std::cout << K << std::endl;     

        // Step 12. Calculate a posteriori state mean of current step
        // log_info("Completed step 11");
        x = xsave + K * ydiff;
        std::cout << "ydiff: " << ydiff << std::endl;
        log_info("absolute difference in data, maxCoeff = {}", max(ydiff.abs()));
        // log_info("difference in data, minCoeff = {}", (double)ydiff.minCoeff());
        log_info("absolute difference in data, mean = {}", mean(ydiff.abs()));
        // Step 12. Calculate a posteriori state covariance of current step
        // log_info("Completed step 12");
        P -= K * Py * K.transpose();
        // std::cout << "P: " << P << std::endl;

        // Abort running inverse kinematics in case of significant numerical instabilities
        if (mean(ydiff.abs()) > 1.0) {
            log_info("Significant numerical instabilities encountered, aborting...");
            // break;
        }
        else if ((double)max(ydiff.abs()) > 3.0) {
            log_info("Significant numerical instabilities encountered, aborting...");
            // break;
        }
        log_info("Solved at time: {} s", time);
        analysisSet.step(ss, step++);
        model.realizeReport(ss);
        // break;
    }	//end of for

	log_info("Got out of for loop and end of KFTool.");

    auto report = ikReporter->getTable();
    // form resultsDir either from results_directory or output_motion_file
    auto resultsDir = get_results_directory();
    if (resultsDir.empty() && !get_output_motion_file().empty())
        resultsDir = OpenSim::IO::getParentDirectory(get_output_motion_file());
    if (!resultsDir.empty()) {
        OpenSim::IO::makeDir(resultsDir);
        // directory will be restored on block exit
        // by changing dir all other files are created in resultsDir
        auto cwd = OpenSim::IO::CwdChanger::changeTo(resultsDir);
        std::string outName = get_output_motion_file();
        outName = OpenSim::IO::GetFileNameFromURI(outName);
        if (outName.empty()) {
            bool isAbsolutePath;
            std::string directory, fileName, extension;
            SimTK::Pathname::deconstructPathname(orientationsFileName,
                    isAbsolutePath, directory, fileName, extension);
            outName = "ik_" + fileName;
        }
        std::string outputFile = outName;

        // Convert to degrees to compare with marker-based IK
        // but only for rotational coordinates
        model.getSimbodyEngine().convertRadiansToDegrees(report);
        report.updTableMetaData().setValueForKey<std::string>("name", outName);

        auto fullOutputFilename = outputFile;
        std::string::size_type extSep = fullOutputFilename.rfind(".");
        if (extSep == std::string::npos) { fullOutputFilename.append(".mot"); }
        OpenSim::STOFileAdapter_<double>::write(report, fullOutputFilename);

        log_info("Wrote IK with IMU tracking results to: '{}'.",
                fullOutputFilename);
        if (get_report_errors()) {
            OpenSim::STOFileAdapter_<double>::write(*modelOrientationErrors,
                    outName + "_orientationErrors.sto");
        }
    } 
    else
        log_info("KFComboInverseKinematicsTool: No output files were generated, "
            "set output_motion_file to generate output files.");
    // Results written to file, clear in case we run again
    ikReporter->clearTable();
}


// main driver
bool OpenSim::KFComboInverseKinematicsTool::run(bool visualizeResults, SimTK::Vector_<double> processCovScales)
{
    if (_model.empty()) {
        _model.reset(new Model(get_model_file()));
    }

    OpenSim::KFComboInverseKinematicsTool::runInverseKinematicsWithOrientationsFromFile(*_model,
            get_orientations_file(), get_distances_file(), visualizeResults, processCovScales);

    return true;
}

OpenSim::TimeSeriesTable_<SimTK::Vec3> OpenSim::KFComboInverseKinematicsTool::loadMarkersFile(const std::string& markerFile)
{
    OpenSim::TimeSeriesTable_<SimTK::Vec3> markers(markerFile);
    log_info("'{}' loaded {} markers and {} rows of data.", markerFile,
        markers.getNumColumns(), markers.getNumRows());

    if (markers.hasTableMetaDataKey("Units")) {
        auto& value = markers.getTableMetaData().getValueForKey("Units");
        log_info("'{}' has Units meta data. Units are {}.", markerFile,
                value.getValue<std::string>());
        if (value.getValue<std::string>() == "mm") {
            log_info("Marker data in mm, converting to m.");
            for (size_t i = 0; i < markers.getNumRows(); ++i) {
                markers.updRowAtIndex(i) *= 0.001;
            }
            markers.updTableMetaData().removeValueForKey("Units");
            markers.updTableMetaData().setValueForKey<std::string>("Units", "m");
        }
    }
    auto& value = markers.getTableMetaData().getValueForKey("Units");
    log_info("'{}' Units are {}.", markerFile, value.getValue<std::string>());

    return markers;
}

std::tuple<std::map<std::string, int>, std::map<int, std::string>> OpenSim::KFComboInverseKinematicsTool::CreateYMaps(OpenSim::Model model) {
	model.initSystem();
    SimTK::State ss = model.getWorkingState();
    OpenSim::Array<std::string> modelStateVariableNames = model.getStateVariableNames();
    int numY = model.getNumStateVariables();
    ss.updY() = 0;
    std::map<std::string, int> yMapFromOpenSimToSimbody;
    std::map<int, std::string> yMapFromSimbodyToOpenSim;
    SimTK::Vector modelStateVariableValues;
    for (int iy = 0; iy < numY; iy++) { //this y-index runs for Simbody
        ss.updY()[iy] = SimTK::NaN;
        modelStateVariableValues = model.getStateVariableValues(ss);
        for (int ii = 0; ii < modelStateVariableNames.size(); ii++) {   //this index runs for OpenSim
            if (SimTK::isNaN(modelStateVariableValues[ii])) {
                yMapFromOpenSimToSimbody.insert(std::pair<std::string, int>(modelStateVariableNames[ii], iy));
                yMapFromSimbodyToOpenSim.insert(std::pair<int, std::string>(iy, modelStateVariableNames[ii]));
                ss.updY()[iy] = 0;
                break;
            }
        }
        if (SimTK::isNaN(ss.updY()[iy])) {
            // If we reach here, this is an unused slot for a quaternion (from Antoine Felisse code)
            ss.updY()[iy] = 0;
        }
    }
    std::tuple<std::map<std::string, int>, std::map<int, std::string>> mappings(yMapFromOpenSimToSimbody, yMapFromSimbodyToOpenSim);
    if (numY != (int)yMapFromOpenSimToSimbody.size()) {
        log_info("There were {} state variables, but got {} mappings from OpenSim to Simbody!", numY,
            (int)yMapFromOpenSimToSimbody.size());
    }
    return mappings;

}   //end of KFComboInverseKinematicsTool::CreateYMaps


double OpenSim::KFComboInverseKinematicsTool::computeFactorial(int input) {
    double fact = 1.0;
    for (int ii = 1; ii <= input; ii++) {
        fact *= ii;
    }
    return fact;
}

double OpenSim::KFComboInverseKinematicsTool::probWithinInterval(double x1, double x2) {
    return (std::erf(x2 / std::sqrt(2)) - std::erf(x1 / std::sqrt(2))) / 2;
}

