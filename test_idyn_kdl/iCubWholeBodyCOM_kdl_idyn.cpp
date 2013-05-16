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

#include <cassert>

using namespace std;
using namespace iCub::iDyn;
using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::os;

double delta = 1e-10;
#define EQUALISH(x,y) norm(x-y) < delta

void printMatrix(string s,const Matrix &m)
{
	cout<<s<<endl;
	for(int i=0;i<m.rows();i++)
	{
		for(int j=0;j<m.cols();j++) {
            if( abs(m(i,j)) < 1e-15 ) {
                cout << " " << 0.0;
            } else {
                cout << " " <<m(i,j);
            }
        }
		cout<<endl;
	}
}

void getPartialCOM(iDynLimb * limb, Matrix* orig, Vector    * total_COM, double * total_mass, int link_limit = 1000) {
    Vector COM;
    *total_mass = 0;
    //printMatrix("origin:",*orig);
    for (int i=0; i<limb->getN() && i < link_limit; i++)
        {
            (*total_mass)=(*total_mass)+limb->getMass(i);
            COM=limb->getCOM(i).getCol(3);
                yarp::sig::Matrix m = limb->getH(i, true);  
            printMatrix("H_b",  m );
            printVector("COM",  COM);   
            
            (*total_COM) = (*total_COM) + ((*orig) * m * COM) * limb->getMass(i); 
        }
        if (fabs(*total_mass) > 0.00001)
            {(*total_COM)=(*total_COM)/(*total_mass);  }
        else
            {(*total_COM).zero();}
}




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
    
    int N_TRIALS = 1;
    int N = 32;
    
    double time_kdl = 0.0;
    double time_idyn = 0.0;
    double tic = 0.0;
    double toc = 0.0;
    
    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
    /*
    printMatrix("low HLeft",icub_idyn.lowerTorso->HLeft);
    printMatrix("low HRight",icub_idyn.lowerTorso->HRight);
    printMatrix("low Hup",icub_idyn.lowerTorso->HUp);
    printMatrix("up HLeft",icub_idyn.upperTorso->HLeft);
    printMatrix("up HRight",icub_idyn.upperTorso->HRight);
    printMatrix("up Hup",icub_idyn.upperTorso->HUp);
    */
    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";

    
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

        for(int i=0;i<N;i++) 
        {
            q[i] = 0*CTRL_DEG2RAD*360*rng.uniform();
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
        assert( com_solver.JntToCOM(q_kdl,COM_kdl) == 0);
        toc = yarp::os::Time::now();
        time_kdl += (toc-tic);
        
        //Compare: 
        cout << "iDyn COM: " << endl;
        printVector("",icub_idyn.whole_COM);
        cout << "iDyn ll MCOM" << endl;
        printVector("",icub_idyn.lowerTorso->total_COM_LF);
        cout << "iDyn rl MCOM" << endl;
        printVector("",icub_idyn.lowerTorso->total_COM_RT);
        cout.precision(10);
        cout << "iDyn ll+rl MCOM" << endl;
        printVector("",icub_idyn.lowerTorso->total_COM_LF+icub_idyn.lowerTorso->total_COM_RT);
        cout << (icub_idyn.lowerTorso->total_COM_LF+icub_idyn.lowerTorso->total_COM_RT)[1] << endl;
        cout << "iDyn torso COM" << endl;
        printVector("",icub_idyn.lowerTorso->total_COM_UP);
        cout << setprecision(10) << (icub_idyn.lowerTorso->total_COM_LF+icub_idyn.lowerTorso->total_COM_RT)[1] << endl;
        cout << "iDyn la COM" << endl;
        printVector("",icub_idyn.upperTorso->total_COM_LF);
        cout << "iDyn ra COM" << endl;
        printVector("",icub_idyn.upperTorso->total_COM_RT);
        cout << (icub_idyn.upperTorso->total_COM_LF+icub_idyn.upperTorso->total_COM_RT)[2] << endl;
        cout << "iDyn hd MCOM" << endl;
        printVector("",icub_idyn.upperTorso->total_mass_UP*icub_idyn.upperTorso->total_COM_UP);

        
        /*
        printMatrix("low HLeft",icub_idyn.lowerTorso->HLeft);
        printMatrix("low HRight",icub_idyn.lowerTorso->HRight);
        printMatrix("low Hup",icub_idyn.lowerTorso->HUp);
        printMatrix("up HLeft",icub_idyn.upperTorso->HLeft);
        printMatrix("up HRight",icub_idyn.upperTorso->HRight);
        printMatrix("up Hup",icub_idyn.upperTorso->HUp);
        */
        /*
        cout << "Test iDyn legs" << endl;
        for(int l=0; l < 2; l++ ) {
            cout << "Com of first " << l << " links " << endl;
            double l_mass,r_mass;
            Vector l_com(4), r_com(4);
            l_com.zero();
            r_com.zero();
            getPartialCOM(icub_idyn.lowerTorso->left,&(icub_idyn.lowerTorso->HLeft),&l_com,&l_mass,l);
            getPartialCOM(icub_idyn.lowerTorso->right,&(icub_idyn.lowerTorso->HRight),&r_com,&r_mass,l);
            printVector("left:",l_com);
            printVector("right:",r_com);
            
        }*/

        //printVector("q",q);
        
        cout << "KDL COM: " << endl;
        cout << COM_kdl << endl;
        
    }
    
    
    cout << "Total time for KDL:" << time_kdl/N_TRIALS << endl;
    cout << "Total time for iDyn:" << time_idyn/N_TRIALS << endl;
    
    
}


