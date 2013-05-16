#include <iostream>

#include <iCub/iDyn/iDyn.h>
#include <yarp/sig/all.h>
#include <yarp/math/api.h>
#include <yarp/os/Log.h>
#include <yarp/os/all.h>
#include <yarp/os/Time.h>

#include "../iDyn_KDL_conversion/iDyn2KDL.h"
#include "../iDyn_KDL_conversion/KDL2iDyn.h"
#include "../icub_model/idyn2kdl_icub.h"

#include "iDyn_KDL_emulation.h"


#include <kdl/chainfksolver.hpp>
#include "custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

#include <kdl_codyco/treecomsolver.hpp>
#include <kdl_codyco/utils.hpp>

#include <cassert>

using namespace std;
using namespace iCub::iDyn;
using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::os;

double delta = 1e-10;
#define EQUALISH(x,y) norm(x-y) < delta



int main(int argc, char * argv[])
{
    iCub::iDyn::version_tag icub_type;
    
    icub_type.head_version = 2;
    if( argc == 1 ) {
        icub_type.legs_version = 1;    
    } else {
        icub_type.legs_version = atoi(argv[1]);
    }
    
    
    //Creating iDyn iCub
    iCubWholeBody icub_idyn(icub_type);
    
    

    KDL::Tree icub_kdl;
    KDL::Tree icub_kdl_urdf;
    
    int N_TRIALS = 1000;
    int N = 32;
    
    double time_kdl = 0.0;
    double time_idyn = 0.0;
    double tic = 0.0;
    double toc = 0.0;
    
    
    //Creating KDL iCub
    bool ret = toKDL(icub_idyn,icub_kdl);
    assert(ret);
    
    //Creating solver
    KDL::TreeCOMSolver com_solver(icub_kdl);
    
    Random rng;
    rng.seed(yarp::os::Time::now());
    
    Vector q(N);
    KDL::JntArray q_kdl(N);
    
    for(int trial=0; trial < N_TRIALS; trial++ ) {
        KDL::Vector COM_kdl;
        Vector COM_kdl_yarp(3);

        for(int i=0;i<N;i++) 
        {
            q[i] = CTRL_DEG2RAD*360*rng.uniform();
        }
        
        q = icub_idyn.setAllPositions(q);
        
        for(int i=0;i<N; i++ ) {
            q_kdl(i) = q[i];
        }
        
        //Compute COM with iDyn
        tic = yarp::os::Time::now();
        icub_idyn.computeCOM();
        toc = yarp::os::Time::now();
        time_idyn += (toc-tic);
        
        
        //Compute COM with KDL
        tic = yarp::os::Time::now();
        int ret = com_solver.JntToCOM(q_kdl,COM_kdl);
        toc = yarp::os::Time::now();
        assert(ret == 0);
        time_kdl += (toc-tic);
        
        //Compare: 
        /*
        cout << "iDyn mass: " << endl;
        cout << icub_idyn.whole_mass << endl;
        cout << "iDyn COM: " << icub_idyn.whole_COM.toString() << endl;
        
        cout << "KDL mass: " << endl;
        cout << KDL::computeMass(icub_kdl) << endl;
        cout << "KDL COM: " << endl;
        cout << COM_kdl << endl;
        */
        bool retb = to_iDyn(COM_kdl,COM_kdl_yarp);
        assert(retb);
        assert(EQUALISH(icub_idyn.whole_COM,COM_kdl_yarp));
    }
    
    
    cout << "Total time for KDL:" << time_kdl/N_TRIALS << endl;
    cout << "Total time for iDyn:" << time_idyn/N_TRIALS << endl;
    cout << "Improvement factor:" << time_idyn/time_kdl << endl;
    
    
}


