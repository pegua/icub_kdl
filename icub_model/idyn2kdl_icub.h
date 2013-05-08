/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#ifndef IDYN2KDL_ICUB_H 
#define IDYN2KDL_ICUB_H 

#include <iCub/iDyn/iDynBody.h>
#include <kdl/tree.h>

/**
 * Get a KDL::Tree model from a iCub::iDyn::iCubWholeBody object
 * 
 * @param icub_idyn the iCub::iDyn::iCubWholeBody iCub object
 * @param icub_kdl the iCub KDL::Tree object
 * @return false in case of error, true otherwise
 */
bool toKDL(const iCub::iDyn::iCubWholeBody & icub_idyn, KDL::Tree & icub_kdl);

#endif
