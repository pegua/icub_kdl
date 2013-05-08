/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#ifndef KDL2IDYN_H
#define KDL2IDYN_H
/**
 * Convert a KDL::Vector to a yarp::sig::Vector 
 * @param kdlVector KDL::Vector input
 * @param idynVector yarp::sig::Vector output
 * @return true if conversion was successful, false otherwise
 */
bool kdlVector2idynVector(const KDL::Vector & kdlVector,yarp::sig::Vector & idynVector);

/**
 * 
 * @return true if conversion was successful, false otherwise
 */
bool kdlJntArray2idynVector(const KDL::JntArray & kdlJntArray,yarp::sig::Vector & idynVector);


#endif
