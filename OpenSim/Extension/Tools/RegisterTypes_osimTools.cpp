/* -------------------------------------------------------------------------- *
 *                   OpenSim:  RegisterTypes_osimExtensionTools.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <OpenSim/Common/Object.h>

#include "osimTools.h"

#include <exception>
#include <iostream>
#include <string>

using namespace std;
using namespace OpenSim;

static osimExtensionToolsInstantiator instantiator;

//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the osimExtensionTools library.
 */
OSIMTOOLS_API void RegisterTypes_osimExtensionTools() {
  try {
    Object::registerType( DistanceInverseKinematicsTool());
    Object::registerType( DistanceWeightSet());
  } catch (const std::exception &e) {
    log_error("Error during osimExtensionTools Object registration: {}.", e.what());
    log_error("");
  }
}

osimExtensionToolsInstantiator::osimExtensionToolsInstantiator() { registerDllClasses(); }

void osimExtensionToolsInstantiator::registerDllClasses() { RegisterTypes_osimExtensionTools(); }