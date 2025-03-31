#include "DistanceBaseDataReader.h"

namespace OpenSim {

    const std::string DistanceBaseDataReader::Distances { "distances" };         // name of table for orientation data
 
    DataAdapter::OutputTables DistanceBaseDataReader::createTablesFromMatrices(double dataRate,
        const std::vector<std::string>& labels, const std::vector<double>& times,
        const SimTK::Matrix_<double>& distancesData) const {

        DataAdapter::OutputTables tables{};

        auto distancesTable = std::make_shared<TimeSeriesTable_<double>>(times, distancesData, labels);
        distancesTable->updTableMetaData()
            .setValueForKey("DataRate", std::to_string(dataRate));
        tables.emplace(Distances, distancesTable);

        return tables;

    }
}
