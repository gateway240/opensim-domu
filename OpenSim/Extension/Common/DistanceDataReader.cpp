#include <fstream>
#include "Simbody.h"
#include <OpenSim/Common/DataAdapter.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/FileAdapter.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <memory>
#include "DistanceDataReader.h"

namespace OpenSim {

std::unique_ptr<DataAdapter> DistanceDataReader::clone() const {
    return std::make_unique<DistanceDataReader>(*this);
}

DataAdapter::OutputTables 
DistanceDataReader::extendRead(const std::string& folderName) const {

    double dataRate = SimTK::NaN;

    int n_sensors = _settings.getProperty_ExperimentalSensors().size();
    int last_size = 1024;
    
    std::string prefix = _settings.get_trial_prefix();

    std::shared_ptr<TimeSeriesTable_<double>> distancesTable;
    for (int index = 0; index < n_sensors; ++index) {
        std::string prefix = _settings.get_trial_prefix();
        const ExperimentalSensor& nextItem = _settings.get_ExperimentalSensors(index);
        auto fileName = folderName + prefix + nextItem.getName() +".sto";
        STOFileAdapter_<double> stofileadapter{};
        auto abstable1 = stofileadapter.read(fileName).at("table");
        OpenSim::TimeSeriesTable_<double>* table1 = static_cast<OpenSim::TimeSeriesTable_<double>*>(abstable1.get());
        std::cout << "Read sto table into memory" << std::endl;
                // Column labels of the table.
        auto& tlabels = table1->
                    getDependentsMetaData().
                    getValueArrayForKey("labels");
        for(size_t i = 0; i < tlabels.size(); ++i)
            std::cout << tlabels[i].getValue<std::string>() << "\t";
        std::cout << std::endl;

        // Dimensions of the table.
        std::cout << table1->getNumRows() << "\t"
                << table1->getNumColumns() << std::endl;

        // Individual rows of the table.
        auto& times = table1->getIndependentColumn();
        // for(size_t i = 0; i < table1->getNumRows(); ++i)
        //     std::cout << "time{" << times[i] << "}: " << table1->getRowAtIndex(i) << std::endl;
        if (index == 0) {
            distancesTable = std::make_shared<TimeSeriesTable>(times);
        }

        // Add imu name to labels
        distancesTable->appendColumn(nextItem.get_name_in_model(), table1->getDependentColumnAtIndex(0));
    }

    DataAdapter::OutputTables tables{};
    distancesTable->updTableMetaData().setValueForKey("DataRate", std::to_string(dataRate));
    tables.emplace("distances", distancesTable);

    return tables;
}

int DistanceDataReader::find_index(std::vector<std::string>& tokens, const std::string& keyToMatch) {
    int returnIndex = -1;
    std::vector<std::string>::iterator it = std::find(tokens.begin(), tokens.end(), keyToMatch);
    if (it != tokens.end())
        returnIndex = static_cast<int>(std::distance(tokens.begin(), it));
    return returnIndex;
}

}
