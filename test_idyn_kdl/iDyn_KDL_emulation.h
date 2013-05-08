/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#ifndef IDYN_KDL_EMULATION_H
#defined IDYN_KDL_EMULATION_H


//for now, for this methods, assuming dw0,w0,ddp0 = 0.0, q,dq,ddq already setted in the chain, and with kinematic information (position) updated based on q
//And no joint blocked!
//Same functions, but no more assuming ddp0 = 0.0
//if ddp0 = 0.0, assuming H0 = eye()
/**
 * Same as iCub::iDyn::iDynChain::getForces() method, but using KDL for calculation
 */
yarp::sig::Matrix idynChainGetForces_usingKDL(iCub::iDyn::iDynChain & idynChain,yarp::sig::Vector & ddp0);

/**
 * Same as iCub::iDyn::iDynChain::getMoments() method, but using KDL for calculation
 */
yarp::sig::Matrix idynChainGetMoments_usingKDL(iCub::iDyn::iDynChain & idynChain,yarp::sig::Vector & ddp0);

/**
 * Same as iCub::iDyn::iDynChain::getTorques() method, but using KDL for calculation
 */
yarp::sig::Vector idynChainGetTorques_usingKDL(iCub::iDyn::iDynChain & idynChain,yarp::sig::Vector & ddp0);

/**
 * Function for internal use
 */
KDL::Wrenches idynChainGet_usingKDL_aux(iCub::iDyn::iDynChain & idynChain,yarp::sig::Vector & ddp0);


yarp::sig::Matrix idynChainGetForces_usingKDL(iCub::iDyn::iDynChain & idynChain,iCub::iDyn::iDynInvSensor & idynSensor,yarp::sig::Vector & ddp0);
yarp::sig::Matrix idynChainGetMoments_usingKDL(iCub::iDyn::iDynChain & idynChain,iCub::iDyn::iDynInvSensor & idynSensor,yarp::sig::Vector & ddp0);
yarp::sig::Vector idynChainGetTorques_usingKDL(iCub::iDyn::iDynChain & idynChain,iCub::iDyn::iDynInvSensor & idynSensor,yarp::sig::Vector & ddp0);

KDL::Wrenches idynChainGet_usingKDL_aux(iCub::iDyn::iDynChain & idynChain,iCub::iDyn::iDynInvSensor & idynSensor,yarp::sig::Vector & ddp0);


yarp::sig::Vector idynSensorGetSensorForce_usingKDL(iCub::iDyn::iDynChain & idynChain,iCub::iDyn::iDynInvSensor & idynSensor,yarp::sig::Vector & ddp0);
yarp::sig::Vector idynSensorGetSensorMoment_usingKDL(iCub::iDyn::iDynChain & idynChain,iCub::iDyn::iDynInvSensor & idynSensor,yarp::sig::Vector & ddp0);


#endif
