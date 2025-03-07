/**
 * @File: gauss_markov_fitting_demo.cpp
 * @Date: May 2023
 * @Author: James Swedeen
 *
 * @brief
 * A small demo node for testing different covariance propagation strategies in path planners.
 **/

/* C++ Headers */
#include<list>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Matplotlib Headers */
#include<matplotlibcpp/matplotlibcpp.hpp>

/* Local Headers */
#include<rrt_search/helpers/rrt_math.hpp>
#include<rrt_search/helpers/connect_waypoints.hpp>
#include<rrt_search/edge_generators/fillets/gauss_markov_covariance_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/euler_spiral_coordinated_turn_edge_generator.hpp>

std::vector<double> toVec(const Eigen::Matrix<double,1,Eigen::Dynamic,Eigen::RowMajor>& input)
{
  std::vector<double> output(input.cols());

  for(Eigen::Index col_it = 0; col_it < input.cols(); ++col_it)
  {
    output[col_it] = input[col_it];
  }

  return output;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("fogm_fit_demo_node");

  node->declare_parameter("var_data_file",      rclcpp::PARAMETER_STRING);
  node->declare_parameter("position_north_tau", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("position_north_q",   rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("position_east_tau",  rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("position_east_q",    rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("position_down_tau",  rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("position_down_q",    rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("roll_tau",           rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("roll_q",             rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("pitch_tau",          rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("pitch_q",            rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("yaw_tau",            rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("yaw_q",              rclcpp::PARAMETER_DOUBLE);

  const double      dt                 = 0.01;
  const double      max_curvature      = 0.005;
  const double      max_curvature_rate = 0.0002;
  const double      nominal_pitch      = 0.05;
  const double      nominal_velocity   = 25;
  const double      nominal_down       = -100;
  const double      gravity            = 9.81;
  const std::string data_file          = node->get_parameter("var_data_file").as_string();

  // Make helpers
  rrt::edge::GaussMarkovCovarianceEdgeGeneratorPtr<> edge_generator =
    std::make_shared<rrt::edge::GaussMarkovCovarianceEdgeGenerator<>>(
      dt,
      std::make_shared<rrt::edge::EulerSpiralCoordinatedTurnEdgeGenerator<false>>(dt,
                                                                                  max_curvature,
                                                                                  max_curvature_rate,
                                                                                  nominal_velocity,
                                                                                  nominal_pitch,
                                                                                  nominal_down,
                                                                                  gravity),
      nominal_velocity,
      std::array<std::pair<double,double>,6>({
        std::make_pair(node->get_parameter("position_north_tau").as_double(), node->get_parameter("position_north_q").as_double()),
        std::make_pair(node->get_parameter("position_east_tau"). as_double(), node->get_parameter("position_east_q"). as_double()),
        std::make_pair(node->get_parameter("position_down_tau"). as_double(), node->get_parameter("position_down_q"). as_double()),
        std::make_pair(node->get_parameter("roll_tau").          as_double(), node->get_parameter("roll_q").          as_double()),
        std::make_pair(node->get_parameter("pitch_tau").         as_double(), node->get_parameter("pitch_q").         as_double()),
        std::make_pair(node->get_parameter("yaw_tau").           as_double(), node->get_parameter("yaw_q").           as_double()),
      }));

  // Get original data
  std::array<Eigen::Matrix<double,1,Eigen::Dynamic,Eigen::RowMajor>,7> orig_data_vecs;
  rrt::edge::GaussMarkovCovarianceEdgeGenerator<>::getDataVectors(data_file, orig_data_vecs);

  // Set waypoints
  std::list<Eigen::Matrix<double,1,13,Eigen::RowMajor>> waypoints;
  waypoints.emplace_back(Eigen::Matrix<double,1,13,Eigen::RowMajor>({orig_data_vecs[0][0], orig_data_vecs[0][0]*nominal_velocity,    0.0, nominal_down, 0.0, nominal_pitch, 0.0, orig_data_vecs[1][0], orig_data_vecs[2][0], orig_data_vecs[3][0], orig_data_vecs[4][0], orig_data_vecs[5][0], orig_data_vecs[6][0]}));
  //waypoints.emplace_back(Eigen::Matrix<double,1,13,Eigen::RowMajor>({0.0, 500.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  //waypoints.emplace_back(Eigen::Matrix<double,1,13,Eigen::RowMajor>({0.0, 1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  waypoints.emplace_back(Eigen::Matrix<double,1,13,Eigen::RowMajor>({0.0, 25000.0, 0.0, nominal_down, 0.0, nominal_pitch, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

  // Generate data
  const Eigen::Matrix<double,Eigen::Dynamic,13,Eigen::RowMajor> trajectory =
    rrt::connectWaypointsFillets<13,double,Eigen::RowMajor>(waypoints, edge_generator);

  // Plot
/*  matplotlibcpp::figure();
  matplotlibcpp::xlabel("Time (sec)");
  matplotlibcpp::ylabel("North Position");
  matplotlibcpp::plot(toVec(trajectory.col(0)), toVec(trajectory.col(1)), "k");
  matplotlibcpp::figure();
  matplotlibcpp::xlabel("Time (sec)");
  matplotlibcpp::ylabel("East Position");
  matplotlibcpp::plot(toVec(trajectory.col(0)), toVec(trajectory.col(2)), "k");
  matplotlibcpp::figure();
  matplotlibcpp::xlabel("Time (sec)");
  matplotlibcpp::ylabel("Down Position");
  matplotlibcpp::plot(toVec(trajectory.col(0)), toVec(trajectory.col(3)), "k");
  matplotlibcpp::figure();
  matplotlibcpp::xlabel("Time (sec)");
  matplotlibcpp::ylabel("Roll");
  matplotlibcpp::plot(toVec(trajectory.col(0)), toVec(trajectory.col(4)), "k");
  matplotlibcpp::figure();
  matplotlibcpp::xlabel("Time (sec)");
  matplotlibcpp::ylabel("Pitch");
  matplotlibcpp::plot(toVec(trajectory.col(0)), toVec(trajectory.col(5)), "k");
  matplotlibcpp::figure();
  matplotlibcpp::xlabel("Time (sec)");
  matplotlibcpp::ylabel("Yaw");
  matplotlibcpp::plot(toVec(trajectory.col(0)), toVec(trajectory.col(6)), "k");*/
  matplotlibcpp::figure();
  matplotlibcpp::xlabel("Time (sec)");
  matplotlibcpp::ylabel("3-sigma");
  matplotlibcpp::title("North Position");
  matplotlibcpp::named_plot<double,double>("True", toVec(orig_data_vecs[0]), toVec(orig_data_vecs[1].array().sqrt()*double(3)), "r");
  matplotlibcpp::named_plot<double,double>("FOGM", toVec(trajectory.col(0)), toVec(trajectory.col(7).array().sqrt()*double(3)), "k");
  matplotlibcpp::legend();
  matplotlibcpp::figure();
  matplotlibcpp::xlabel("Time (sec)");
  matplotlibcpp::ylabel("3-sigma");
  matplotlibcpp::title("East Position");
  matplotlibcpp::named_plot<double,double>("True", toVec(orig_data_vecs[0]), toVec(orig_data_vecs[2].array().sqrt()*double(3)), "r");
  matplotlibcpp::named_plot<double,double>("FOGM", toVec(trajectory.col(0)), toVec(trajectory.col(8).array().sqrt()*double(3)), "k");
  matplotlibcpp::legend();
  matplotlibcpp::figure();
  matplotlibcpp::xlabel("Time (sec)");
  matplotlibcpp::ylabel("3-sigma");
  matplotlibcpp::title("Down Position");
  matplotlibcpp::named_plot<double,double>("True", toVec(orig_data_vecs[0]), toVec(orig_data_vecs[3].array().sqrt()*double(3)), "r");
  matplotlibcpp::named_plot<double,double>("FOGM", toVec(trajectory.col(0)), toVec(trajectory.col(9).array().sqrt()*double(3)), "k");
  matplotlibcpp::legend();
  matplotlibcpp::figure();
  matplotlibcpp::xlabel("Time (sec)");
  matplotlibcpp::ylabel("3-sigma");
  matplotlibcpp::title("Roll");
  matplotlibcpp::named_plot<double,double>("True", toVec(orig_data_vecs[0]), toVec(orig_data_vecs[4]. array().sqrt()*double(3)), "r");
  matplotlibcpp::named_plot<double,double>("FOGM", toVec(trajectory.col(0)), toVec(trajectory.col(10).array().sqrt()*double(3)), "k");
  matplotlibcpp::legend();
  matplotlibcpp::figure();
  matplotlibcpp::xlabel("Time (sec)");
  matplotlibcpp::ylabel("3-sigma");
  matplotlibcpp::title("Pitch");
  matplotlibcpp::named_plot<double,double>("True", toVec(orig_data_vecs[0]), toVec(orig_data_vecs[5]. array().sqrt()*double(3)), "r");
  matplotlibcpp::named_plot<double,double>("FOGM", toVec(trajectory.col(0)), toVec(trajectory.col(11).array().sqrt()*double(3)), "k");
  matplotlibcpp::legend();
  matplotlibcpp::figure();
  matplotlibcpp::xlabel("Time (sec)");
  matplotlibcpp::ylabel("3-sigma");
  matplotlibcpp::title("Yaw");
  matplotlibcpp::named_plot<double,double>("True", toVec(orig_data_vecs[0]), toVec(orig_data_vecs[6]. array().sqrt()*double(3)), "r");
  matplotlibcpp::named_plot<double,double>("FOGM", toVec(trajectory.col(0)), toVec(trajectory.col(12).array().sqrt()*double(3)), "k");
  matplotlibcpp::legend();
  matplotlibcpp::show();

  exit(EXIT_SUCCESS);
}

