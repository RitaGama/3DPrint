// =============================================================================
// Name        : print_node.cpp
// Author      : Rita Gama
// Description :
// =============================================================================

#include <franka_spacenav/spacenav.h>
#include <tf/transform_broadcaster.h>

// STATE MACHINE --------------------------------------------------------------
#define FIRST_MOVE 0
#define PRINT 1
// -----------------------------------------------------------------------------

int main(int argc, char **argv){
    
    geometry_msgs::PoseStamped marker_pose;

    ros::init(argc, argv, "print_node");

    ros::NodeHandle nh;
    franka_spacenav::Spacenav panda(nh);

    // ---------------------------------------------------------------------------
    // GET FILE
    // ---------------------------------------------------------------------------
    Eigen::MatrixXd position;  // matrix to save the robot positions in Base-frame
    Eigen::MatrixXd orientation;  // matrix to save the robot orientations in Base-frame
    std::ifstream pattern_file;
    std::string pattern_path;
    pattern_path = "/home/ritagama/catkin_ws/src/3DPrint/Files/Manequim/Ponto1/AreaManequim25_306";
    pattern_file.open(pattern_path);

    //p_x p_y p_z Qx Qy Qz Qw
    double px, py, pz, qx, qy, qz, qw;
    std::string line;
    getline(pattern_file, line);  // first line
    int n_pattern = 0;
    position.resize(3, n_pattern + 1);
    orientation.resize(4, n_pattern + 1);
    if(pattern_file.is_open()){
        while(pattern_file >> px >> py >> pz >> qx >> qy >> qz >> qw){
            // save the values in the matrix
            position.conservativeResize(3, n_pattern + 1);
            orientation.conservativeResize(4, n_pattern + 1);
            position(0, n_pattern) = px;
            position(1, n_pattern) = py;
            position(2, n_pattern) = pz;
            orientation(0, n_pattern) = qx;
            orientation(1, n_pattern) = qy;
            orientation(2, n_pattern) = qz;
            orientation(3, n_pattern) = qw;
            n_pattern++;
        }
    }
    else{
        std::cout << "\nError open the file!" << std::endl;
        return(0);
    }
    pattern_file.close();

    // ---------------------------------------------------------------------------
    // COMPUTE OFFSETS BETWEEN POSITIONS
    // ---------------------------------------------------------------------------
    Eigen::MatrixXd OFFSET;
    OFFSET.resize(3, n_pattern-1);
    for(int i = 0; i < n_pattern-1; i++){
        OFFSET(0, i) = position(0, i+1) - position(0, i);
        OFFSET(1, i) = position(1, i+1) - position(1, i);
        OFFSET(2, i) = position(2, i+1) - position(2, i);
    }

    // ---------------------------------------------------------------------------
    // GET INITIAL POSE
    // ---------------------------------------------------------------------------
    Eigen::Matrix4d O_T_EE_i;
    Eigen::VectorXd pose_i(7,1);
    O_T_EE_i = panda.FK;
    pose_i = panda.robot_pose(O_T_EE_i);

    // ---------------------------------------------------------------------------
    // TRAJECTORY TO FIRST POINT
    // ---------------------------------------------------------------------------
    Eigen::Vector3d Pos;
    Pos << 0.585994, 0.1, 0.2477107;

    Eigen::Vector3d p2, pi, pf;
    pi << pose_i[0], pose_i[1], pose_i[2];
    p2 << position(0, 0), position(1, 0), position(2, 0);
    pf << Pos + p2;
    double ti = 1.0;
    double tf = 6.0;
    double t = 0.0;
    double delta_t = 0.001;

    // ---------------------------------------------------------------------------
    // GET INITIAL ORIENTATION
    // ---------------------------------------------------------------------------
    Eigen::Quaterniond oi, of;
    oi.coeffs() << pose_i[3], pose_i[4], pose_i[5], pose_i[6];
    // compute the desired rotation in the first point of the mold
    Eigen::Quaterniond Qmold_temp;
    Qmold_temp.coeffs() << orientation(0, 0), orientation(1, 0), orientation(2, 0), orientation(3, 0);
    Qmold_temp.normalize();
    Eigen::Matrix3d Rmould(Qmold_temp);
    Eigen:: Matrix3d Rpanda(oi);
    Eigen:: Quaterniond Rpoints(Rmould*Rpanda); // --
    Eigen::Quaterniond Qpoints(Rpoints);
    of.coeffs() << Qpoints.vec()[0], Qpoints.vec()[1], Qpoints.vec()[2], Qpoints.w();
    double t1 = 0.0;
    double delta_t1 = delta_t/(tf-ti);

    // ---------------------------------------------------------------------------
    // MAIN LOOP
    // ---------------------------------------------------------------------------
    Eigen::Vector3d pd(pi);
    Eigen::Quaterniond od(oi);
    Eigen::Vector3d new_pd(pd);

    Eigen::Quaterniond Qmold_aux;
    Eigen::Quaterniond new_od;

    Eigen::Vector3d aux_offset;
    aux_offset.setZero();

    int flag_pattern = 0;
    int flag_print = 0;
    int count = 0;

    ros::Rate loop_rate(1000);
    while (ros::ok()){

        switch (flag_pattern) {

            case FIRST_MOVE:

                if(flag_print == 0){
                    std::cout << CLEANWINDOW << "ROBOT IS MOVING TO A MOLD POINT..." << std::endl;
                    flag_print = 1;
                }

                // --> MOVE TO A MOLD POINT <--
                if( (t >= ti) && (t <= tf) ){
                    pd = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
                    if ( t1 <= 1.0 ){
                        od = oi.slerp(t1, of);
                        od.normalize();
                    }
                    t1 = t1 + delta_t1;
                }
                else if(t > tf){
                    flag_pattern = PRINT;
                    count = 0;
                    t = 0;
                }
                t = t + delta_t;

                new_pd = pd;


            break;

            case PRINT:
                if(flag_print == 1){
                    std::cout << CLEANWINDOW << "START PRINTING!" << std::endl;
                }

                if (count < n_pattern-1){

                    aux_offset << OFFSET(0, count), OFFSET(1, count), OFFSET(2, count);
                    new_pd = new_pd + Rmould * aux_offset;

                    pd = new_pd;

                    //Set new orientation
                    oi.coeffs() << pose_i[3], pose_i[4], pose_i[5], pose_i[6];
                    Qmold_aux.coeffs() << orientation(0, count+1), orientation(1, count+1), orientation(2, count+1), orientation(3, count+1);
                    Qmold_aux.normalize();
                    Eigen::Matrix3d Rmould_new(Qmold_aux);
                    Eigen::Matrix3d Rpanda(oi);
                    Eigen::Matrix3d Rpoints(Rmould_new*Rpanda);
                    Eigen::Quaterniond Qpoints(Rpoints);
                    new_od.coeffs() << Qpoints.vec()[0], Qpoints.vec()[1], Qpoints.vec()[2], Qpoints.w();

                    od = new_od;

                    t = t + delta_t; //time

                }// --------------------------------------------------------------------
                else{

                    std::cout << CLEANWINDOW << "FINISHED PRINT. TIME: " << t << "s "<< std::endl;

                    return 0;
                }
                count++;

            break;
        }

        panda.publisherCallback(marker_pose, pd, od);

        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) break;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}