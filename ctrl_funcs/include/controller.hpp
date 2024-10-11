// #include <Eigen/Core>
// #include <Eigen/Dense>
// #include "filter.hpp"
// #include <stdio.h>
// #include <iostream>
// #include <vector>
// #include "msg_hanlder.hpp"

// class Controller : public Filter
// {
// public:
//     explicit Controller(double sampling_time, MsgHandler *msg_handler);
//     virtual ~Controller();
//     // 내부에서 PID게인 세팅하기    
//     void update();

// private:
//     vector<double> Kp_rw[NUMOFLEG]{0}; vector<double> Ki_rw[NUMOFLEG]{0};
//     vector<double> Kd_rw[NUMOFLEG]{0}; vector<double> Qd_rw[NUMOFLEG]{0.1};

//     vector<double> Kp_rw_sim[NUMOFLEG]{0}; vector<double> Ki_rw_sim[NUMOFLEG]{0}; 
//     vector<double> Kd_rw_sim[NUMOFLEG]{0}; vector<double> Qd_rw_sim[NUMOFLEG]{0.1};

//     vector<double> Kp_j[RNUMOFSLAVE]{0}; vector<double> Ki_j[RNUMOFSLAVE]{0};
//     vector<double> Kd_j[RNUMOFSLAVE]{0}; vector<double> Qd_j[NUMOFLEG]{0.1};

//     vector<double> Kp_j_sim[SNUMOFSLAVE]{0}; vector<double> Ki_j_sim[SNUMOFSLAVE]{0}; 
//     vector<double> Kd_j_sim[SNUMOFSLAVE]{0}; vector<double> Qd_j_sim[NUMOFLEG]{0.1};
    
//     vector<double> Q_jDOB[RNUMOFSLAVE]{0.1}; vector<double> Q_jDOB_sim[SNUMOFSLAVE]{0.1};
//     vector<double> Q_rwDOB[NUMOFLEG]{0.1}; vector<double> Q_rwDOB_sim[NUMOFLEG]{0.1}; 
//     vector<double> Q_oriDOB[NUMOFLEG]{0.1}; vector<double> Q_oriDOB_sim[NUMOFLEG]{0.1};
    


//     std::make_shared<MsgHandler> message_handler();
    

// };