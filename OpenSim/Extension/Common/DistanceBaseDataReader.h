#ifndef OPENSIM_DISTANCE_BASE_DATA_READER_H_
#define OPENSIM_DISTANCE_BASE_DATA_READER_H_

/* -------------------------------------------------------------------------- *
 *                          OpenSim:  DistanceBaseDataReader.h                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2019 Stanford University and the Authors                *
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
#include "osimCommonDLL.h"
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Common/DataAdapter.h>

/** @file
* This file defines common base class for various DISTANCE DataReader
* classes that support different DISTANCE providers
*/

namespace OpenSim {


class OSIMCOMMON_API DistanceBaseDataReader : public DataAdapter {

public:
    
    DistanceBaseDataReader() = default;
    DistanceBaseDataReader(const DistanceBaseDataReader&)            = default;
    DistanceBaseDataReader(DistanceBaseDataReader&&)                 = default;
    DistanceBaseDataReader& operator=(const DistanceBaseDataReader&) = default;
    DistanceBaseDataReader& operator=(DistanceBaseDataReader&&)      = default;
    virtual ~DistanceBaseDataReader()                   = default;

    static const std::string Distances;         // name of table for orientation data
   /**
     * Custom accessors to retrieve tables of proper types without requiring users/scripters to cast.
     * Scripting friendly */
     /** get table of Orientations as TimeSeriesTableQuaternion */
    static const TimeSeriesTable_<double>& getDistancesTable(const DataAdapter::OutputTables& tables) {
        return dynamic_cast<const TimeSeriesTable_<double>&>(*tables.at(Distances));
    }

protected:
    /** create a map of names to TimeSeriesTables. MetaData contains dataRate.
     * The result can be passed to accessors above to get individual TimeSeriesTable(s)
     * If a matrix has nrows = 0 then an empty table is created.
     */
    DataAdapter::OutputTables createTablesFromMatrices(double dataRate, 
        const std::vector<std::string>& labels, const std::vector<double>& times,
        const SimTK::Matrix_<double>& distancesData) const;
};

} // OpenSim namespace

#endif // OPENSIM_DISTANCE_DATA_READER_H_
