/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include "idyn2kdl_icub.h"

bool names2links_joints(const std::vector<std::string> names,std::vector<std::string> & names_links,std::vector<std::string> & names_joint)
{
    names_links = names;
    names_joints = names;
    for(int i=0;i<names.size();i++) {
        names_links[i] = names_links[i]+"_link";
        name_joints[i] = names_joints[i]+"_joint";
    }
}

bool toKDL(const iCub::iDyn::iCubWholeBody & icub_idyn, KDL::Tree & icub_kdl)
{
    //Joint names extracted from http://eris.liralab.it/wiki/ICub_joints
    //Serialization: left leg (6), right leg (6), torso (3), left arm (7), right arm (7), head (3). 
    
    //Default "fake" base link for humanoids URDF
    icub_kdl = Tree("base_link");
    
    std::vector<string> joints,links;
    
    //Creating left leg
    KDL::Chain ll;
    const char *ll_joints_cstr[] = {"left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll"};    
    std::vector<std::string> ll_joints(ll_joints_cstr,end(ll_joints_cstr));
    names2links_joints(ll_joints,links,joints);
    idynSensorChain2kdlChain(*(icub_idyn.lowerTorso->left),*(icub_idyn.lowerTorso->leftSensor),ll,links,joints)
    
    //Creating right leg
    KDL::Chain rl;
    const char *rl_joints_cstr[] = {"right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll"};    
    std::vector<std::string> rl_joints(rl_joints_cstr,end(rl_joints_cstr));
    names2links_joints(rl_joints,links,joints);
    idynSensorChain2kdlChain(*(icub_idyn.lowerTorso->right),*(icub_idyn.lowerTorso->rightSensor),rl,links,joints)

    
    //Creating torso
    KDL::Chain torso;
    const char *torso_joints_cstr[] = {"torso_yaw","torso_roll","torso_pitch"};
    std::vector<std::string> torso_joints(torso_joints_cstr,end(torso_joints_cstr));
    names2links_joints(torso_joints,links,joints);

    
    //Creating left arm
    KDL::Chain la;
    const char *la_joints_cstr[] = {"left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow", "left_wrist_prosup", "left_wrist_pitch","left_wrist_yaw",};    
    std::vector<std::string> la_joints(la_joints_cstr,end(la_joints_cstr));
    names2links_joints(la_joints,links,joints);
    idynSensorChain2kdlChain(*(icub_idyn.upperTorso->left),*(icub_idyn.upperTorso->leftSensor),la,links,joints)


    //Creating right arm
    KDL::Chain ra;
    const char *ra_joints_cstr[] = {"left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll"};    
    std::vector<std::string> ra_joints(ra_joints_cstr,end(ra_joints_cstr));
    names2links_joints(ra_joints,links,joints);
    idynSensorChain2kdlChain(*(icub_idyn.upperTorso->right),*(icub_idyn.upperTorso->rightSensor),ra,links,joints)

    //Creating head
    KDL::Chain head;
    const char *head_joints_cstr[] = {"neck_pitch","neck_roll","neck_yaw","eyes_tilt","eyes_version","eyes_vergence"}
    std::vector<std::string> head_joints(head_joints_cstr,end(head_joints_cstr));
    names2links_joints(head_joints,links,joints);
    idynChain2kdlChain(*(icub_idyn.upperTorso->up),head,links,joints);

}
