#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "graph.h"

namespace rrt
{
    class Node_test : public ::testing::Test
    {
        protected:
        Node* under_test_;
    };

    class Graph_test : public ::testing::Test
    {

    };
};

int main(int argc, char **argv) {
    return 0;
}

/*
namespace controls 
{

    class PID_Test : public ::testing::Test 
    {
        protected:
        /// Pointer for Kalman Filter object
        std::shared_ptr<PID_Controller> pid_ptr;
        double kp, ki, kd;
        double max, min;
        double imax, imin;
        double init_val;
        double fltr_coef;

        virtual void SetUp()
        {
            kp = 0.0F;
            ki = 0.0F;
            kd = 0.0F;
            max = 10.0F;
            min = -max;
            imax = 3.0F;
            imin = -imax;
            init_val = 0.0F;
            fltr_coef = 1.0F;
            pid_ptr = std::make_shared<PID_Controller>(kp, ki, kd, max, min, imax, imin, init_val, fltr_coef);
        }

        virtual void TearDown()
        {
        }
    };

    TEST_F(PID_Test, PTEST)
    {
        double kp = 3.0F;
        double ki = 0.0F;
        double kd = 0.0F;
        double dT = 0.1F;
        bool result = false;

        Eigen::VectorXd error(63);          /// Control Error
        Eigen::VectorXd pid_expected(63);   /// Expected 
        Eigen::VectorXd pid_out(63);        /// Controller Output
        double cmp_error = 0.0F;            /// Compare Error
        double mae = 0.0F;                  /// Mean Abs Error
        
        pid_ptr->Init();
        pid_ptr->Update_Gains(kp, ki, kd);
        pid_ptr->Update_dT(dT);

        /// Run Controler with 5.17 deg/100ms error signal
        for(int i=0; i<63; i++)
        {      
            error(i) = sin(i*dT);      
            pid_expected(i) = kp*error(i);
            pid_out(i) = pid_ptr->Step(error(i));
            cmp_error = pid_expected(i) - pid_out(i);
            mae += cmp_error;
        }
        
        /// Mean Absolute Error
        mae /= 63;
        if(fabs(mae) < 1e-6)
        {
            result = true;
        }
        else 
        {
            result = false;
            std::cerr << "PTest Mean Absolute Arror: " << mae << std::endl;
        }
        EXPECT_TRUE(result);
    }

    TEST_F(PID_Test, ITEST)
    {
        double kp = 0.0F;
        double ki = 1.0F;
        double kd = 0.0F;
        double dT = 0.01F;
        double imax = 5.0F;
        double imin = -5.0F;
        double initial_val = 0.0F;
        bool result = false;

        Eigen::VectorXd error(630);          /// Control Error
        Eigen::VectorXd pid_expected(630);   /// Expected 
        Eigen::VectorXd pid_out(630);        /// Controller Output
        double cmp_error = 0.0F;            /// Compare Error
        double mae = 0.0F;                  /// Mean Abs Error

        /// Initialize Controller
        pid_ptr->Init();
        pid_ptr->Update_I_Sat_Limit(imax, imin);
        pid_ptr->Update_InitVal(initial_val);
        pid_ptr->Update_Gains(kp, ki, kd);
        pid_ptr->Update_dT(dT);

        /// Run Controler with 0.57 deg/10ms error signal
        for(int i=0; i<630; i++)
        {
            error(i) = sin(i*dT);

            pid_expected(i) = ki*1.0F-cos(i*dT);
            pid_out(i) = pid_ptr->Step(error(i));
            cmp_error = pid_expected(i) - pid_out(i);
            mae += cmp_error;
        }

        /// Mean Absolute Error
        mae /= 630;
        if(fabs(mae) < 0.002)
        {
            result = true;
        }
        else 
        {
            result = false;
            std::cerr << "ITest Mean Absolute Error: " << mae << std::endl;
        }
        EXPECT_TRUE(result);
    }

    TEST_F(PID_Test, ISAT_TEST)
    {
        double kp = 0.0F;
        double ki = 1.0F;
        double kd = 0.0F;
        double dT = 0.01F;
        double max = 1.0F;
        double min = -1.0F;
        double imax = 0.5F;
        double imin = -0.5F;
        double initial_val = 0.0F;
        bool result = false;

        double pid_out;        /// Controller Output
        double error;          /// Controller Error

        /// Initialize Controller
        pid_ptr->Init();
        pid_ptr->Update_Sat_Limit(max, min);
        pid_ptr->Update_I_Sat_Limit(imax, imin);
        pid_ptr->Update_InitVal(initial_val);
        pid_ptr->Update_Gains(kp, ki, kd);
        pid_ptr->Update_dT(dT);

        /// Run Controler with 0.57 deg/10ms error signal
        for(int i=0; i<1260; i++)
        {
            error = sin(i*dT);
            pid_out = pid_ptr->Step(error);
            if(pid_out > imax || pid_out < imin)
            {
                result = false;
                std::cerr << "PID_OUT: " << pid_out << "   I LIM +/-: " 
                          << imax << std::endl;
            }
        }
        EXPECT_TRUE(result);
    }

    TEST_F(PID_Test, PIDSAT_DYN_UPDATE_TEST)
    {
        double kp = 1.0F;
        double ki = 2.0F;
        double kd = 0.0F;
        double dT = 0.01F;
        double max = 1.6F;
        double min = 1.6F;
        double imax = 0.5F;
        double imin = 0.5F;
        double initial_val = 0.0F;
        bool result = false;

        double pid_out;        /// Controller Output
        double error;          /// Controller Error

        /// Initialize Controller
        pid_ptr->Init();
        pid_ptr->Update_Sat_Limit(max, min);
        pid_ptr->Update_I_Sat_Limit(imax, imin);
        pid_ptr->Update_InitVal(initial_val);
        pid_ptr->Update_Gains(kp, ki, kd);
        pid_ptr->Update_dT(dT);

        /// Run Controler with 0.57 deg/10ms error signal
        for(int i=0; i<1260; i++)
        {
            
            if(i < 315)
            {
                error = 1.0F;
            }
            else if (i == 315)
            {
                /// Dynamically Update P-Gain so that PID Limit is Exercised
                /// At this point, the I term should be saturated at 0.5F
                /// The PID Out should be Saturated at 1.5F (I:0.5F P:1.0F)
                kp = 5.0F;
                pid_ptr->Update_Gains(kp, ki, kd);
            }
            else if(i == 630)
            {
                error = -1.0F;
                kp = 1.0F;
                pid_ptr->Update_Gains(kp, ki, kd);
            }
            else if(i == 945)
            {
                /// Same Strategy as above but Exercising the negative limit
                kp = 5.0F;
                pid_ptr->Update_Gains(kp, ki, kd);
            }

            /// Step PID Controller
            pid_out = pid_ptr->Step(error);

            if(pid_out > max || pid_out < min)
            {
                result = false;
                std::cerr << "PID_OUT: " << pid_out << "   PID LIM +/-: " 
                          << max << std::endl;
            }
        }
        EXPECT_TRUE(result);
    }

    TEST_F(PID_Test, DTERM_LPF_TEST)
    {
        double kp = 0.0F;
        double ki = 0.0F;
        double kd = 1.0F;
        double dT = 0.01F;
        double fltr_coef = 0.75F;
        double initial_val = 0.0F;
        bool result = false;

        Eigen::VectorXd error(315);        /// Control Error
        Eigen::VectorXd pid_expected(315); /// Expected 
        Eigen::VectorXd pid_out(315);      /// Controller Output
        double cmp_error = 0.0F;            /// Compare Error
        double mae = 0.0F;                  /// Mean Abs Error
        double rmse = 0.0F;                 //RMSE

        /// Initialize Controller
        pid_ptr->Init();
        pid_ptr->Update_InitVal(initial_val);
        pid_ptr->Update_Gains(kp, ki, kd);
        pid_ptr->Update_D_Filter(fltr_coef);
        pid_ptr->Update_dT(dT);

        /// Run Controler with 0.57 deg/10ms error signal
        for(int i=0; i<315; i++)
        {
            error(i) = sin(i*dT);
            pid_expected(i) = kd*cos(i*dT);

            /// Inject Transient
            /// RMSE NO Transient No LPF:           0.0564541
            /// RMSE With Transient No LPF:         0.069098
            /// RMSE No Tranisent With COEF 0.75:   0.0584871
            /// RMSE Transient With COEF 0.75       0.0643062
            if(i == 310)
            {
                error(i) = 0.005F+error(i);
            }

            pid_out(i) = pid_ptr->Step(error(i));
            cmp_error = pid_expected(i) - pid_out(i);
            mae += cmp_error;
            rmse += cmp_error *cmp_error;
        }

        mae /= 315;
        rmse /= 315;
        rmse = sqrt(rmse);

        if(rmse > 0.065F)
        {
            result = false;
            std::cerr << "RMSE: " << rmse << std::endl;
            std::cerr << "MAE: " << mae << std::endl;
        }

        EXPECT_TRUE(result);
    }

    TEST_F(PID_Test, DTEST)
    {
        double kp = 0.0F;
        double ki = 0.0F;
        double kd = 1.3F;
        double dT = 0.01F;
        bool result = false;

        Eigen::VectorXd error(630);          /// Control Error
        Eigen::VectorXd pid_expected(630);   /// Expected 
        Eigen::VectorXd pid_out(630);        /// Controller Output
        double cmp_error = 0.0F;            /// Compare Error
        double mae = 0.0F;
        double rmse = 0.0F;                  /// Mean Abs Error

        /// Initialize Controller
        pid_ptr->Init();
        pid_ptr->Update_Gains(kp, ki, kd);
        pid_ptr->Update_dT(dT);

        /// Run Controler with 0.57 deg/10ms error signal
        for(int i=0; i<630; i++)
        {
            error(i) = sin(i*dT);

            pid_expected(i) = kd*cos(i*dT);
            pid_out(i) = pid_ptr->Step(error(i));
            cmp_error = pid_expected(i) - pid_out(i);
            mae += cmp_error;
            cmp_error = cmp_error * cmp_error;
            rmse += cmp_error;
        }

        /// Mean Absolute Error
        mae /= 630;
        rmse /= 630;
        rmse = sqrt(rmse);
        //std::cerr << "DTest RMSE " << rmse << std::endl;
        //std::cerr << "DTest MAE " << mae << std::endl;
        if(fabs(mae) < 0.005)
        {
            result = true;
        }
        else 
        {
            result = false;
            std::cerr << "DTest Mean Absolute Error: " << mae << std::endl;
        }
        EXPECT_TRUE(result);
    }
};

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

*/