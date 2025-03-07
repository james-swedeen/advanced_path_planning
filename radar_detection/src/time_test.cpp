/**
 * @File: time_test.cpp
 * @Date: April 2022
 * @Author: James Swedeen
 *
 * @brief
 * Used to time the functions implemented in this package.
 **/

/* C++ Headers */
#include<iostream>
#include<string>
#include<iomanip>
#include<chrono>

/* Local Headers */
#include<radar_detection/radar_detection.hpp>

void timeFunc(const std::function<void(Eigen::Matrix<double,1,6,Eigen::RowMajor>&,
                                       Eigen::Matrix<double,1,3,Eigen::RowMajor>&,
                                       rd::CrossSectionModelPtr<double,1,Eigen::RowMajor>&,
                                       rd::RadarModel<double,1,Eigen::RowMajor>&)>& timed_func,
              rd::CrossSectionModelPtr<double,1,Eigen::RowMajor>&                   cross_section,
              const std::string&                                                    name)
{
  Eigen::Matrix<double,1,6,Eigen::RowMajor> aircraft_pose;
  Eigen::Matrix<double,1,3,Eigen::RowMajor> radar_position;

  rd::RadarModel<double,1,Eigen::RowMajor> radar(double(1e-6), double(167.4277));

  // Timing variables
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
  std::chrono::time_point<std::chrono::high_resolution_clock> end_time;

  aircraft_pose[rd::AC_IND::NORTH] = 934;
  aircraft_pose[rd::AC_IND::EAST]  = -498;
  aircraft_pose[rd::AC_IND::DOWN]  = -282;
  aircraft_pose[rd::AC_IND::ROLL]  = rd::pi<double>()*double(0.26);
  aircraft_pose[rd::AC_IND::PITCH] = rd::pi<double>()*double(1.56);
  aircraft_pose[rd::AC_IND::YAW]   = rd::pi<double>()*double(1.29);

  radar_position[rd::RADAR_IND::NORTH] = 25;
  radar_position[rd::RADAR_IND::EAST]  = -18;
  radar_position[rd::RADAR_IND::DOWN]  = -12;

  double average_time = 0;
  const size_t num_runs = 100000;

  for(size_t count = 0; count < num_runs; ++count)
  {
    start_time = std::chrono::high_resolution_clock::now();
    timed_func(aircraft_pose, radar_position, cross_section, radar);
    end_time = std::chrono::high_resolution_clock::now();

    average_time += std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
  }
  average_time /= double(num_runs);

  std::cout << "It took " << name << " " << average_time << " nanoseconds" << std::endl;
}

int main(int, char**)
{
  rd::CrossSectionModelPtr<double,1,Eigen::RowMajor> cross_section_model;

  cross_section_model = std::make_shared<rd::ConstantCrossSectionModel<double,1,Eigen::RowMajor>>(0.2);
  timeFunc([](Eigen::Matrix<double,1,6,Eigen::RowMajor>&          aircraft_pose,
              Eigen::Matrix<double,1,3,Eigen::RowMajor>&          radar_position,
              rd::CrossSectionModelPtr<double,1,Eigen::RowMajor>& cross_section,
              rd::RadarModel<double,1,Eigen::RowMajor>&           radar)
             { radar.findProbabilityOfDetection(aircraft_pose, radar_position, cross_section); },
           cross_section_model,
           "PD - const");
  timeFunc([](Eigen::Matrix<double,1,6,Eigen::RowMajor>&          aircraft_pose,
              Eigen::Matrix<double,1,3,Eigen::RowMajor>&          radar_position,
              rd::CrossSectionModelPtr<double,1,Eigen::RowMajor>& cross_section,
              rd::RadarModel<double,1,Eigen::RowMajor>&           radar)
             { radar.findProbabilityOfDetectionPDAircraftPose(aircraft_pose, radar_position, cross_section); },
           cross_section_model,
           "PD PD aircraft - const");
  timeFunc([](Eigen::Matrix<double,1,6,Eigen::RowMajor>&          aircraft_pose,
              Eigen::Matrix<double,1,3,Eigen::RowMajor>&          radar_position,
              rd::CrossSectionModelPtr<double,1,Eigen::RowMajor>& cross_section,
              rd::RadarModel<double,1,Eigen::RowMajor>&           radar)
             {
               radar.findProbabilityOfDetectionPDRadarPosition(aircraft_pose, radar_position, cross_section);
               radar.findProbabilityOfDetectionPDConsolidatedRadarConstant(aircraft_pose, radar_position, cross_section);
             },
           cross_section_model,
           "PD PD radar - const");

  cross_section_model = std::make_shared<rd::EllipsoidCrossSectionModel<double,1,Eigen::RowMajor>>(0.25, 0.15, 0.17);
  timeFunc([](Eigen::Matrix<double,1,6,Eigen::RowMajor>&          aircraft_pose,
              Eigen::Matrix<double,1,3,Eigen::RowMajor>&          radar_position,
              rd::CrossSectionModelPtr<double,1,Eigen::RowMajor>& cross_section,
              rd::RadarModel<double,1,Eigen::RowMajor>&           radar)
             { radar.findProbabilityOfDetection(aircraft_pose, radar_position, cross_section); },
           cross_section_model,
           "PD - ellips");
  timeFunc([](Eigen::Matrix<double,1,6,Eigen::RowMajor>&          aircraft_pose,
              Eigen::Matrix<double,1,3,Eigen::RowMajor>&          radar_position,
              rd::CrossSectionModelPtr<double,1,Eigen::RowMajor>& cross_section,
              rd::RadarModel<double,1,Eigen::RowMajor>&           radar)
             { radar.findProbabilityOfDetectionPDAircraftPose(aircraft_pose, radar_position, cross_section); },
           cross_section_model,
           "PD PD aircraft - ellips");
  timeFunc([](Eigen::Matrix<double,1,6,Eigen::RowMajor>&          aircraft_pose,
              Eigen::Matrix<double,1,3,Eigen::RowMajor>&          radar_position,
              rd::CrossSectionModelPtr<double,1,Eigen::RowMajor>& cross_section,
              rd::RadarModel<double,1,Eigen::RowMajor>&           radar)
             {
               radar.findProbabilityOfDetectionPDRadarPosition(aircraft_pose, radar_position, cross_section);
               radar.findProbabilityOfDetectionPDConsolidatedRadarConstant(aircraft_pose, radar_position, cross_section);
             },
           cross_section_model,
           "PD PD radar - ellips");

  cross_section_model = std::make_shared<rd::SpikeballCrossSectionModel<double,1,Eigen::RowMajor>>(4, 0.1, 0.18);
  timeFunc([](Eigen::Matrix<double,1,6,Eigen::RowMajor>&          aircraft_pose,
              Eigen::Matrix<double,1,3,Eigen::RowMajor>&          radar_position,
              rd::CrossSectionModelPtr<double,1,Eigen::RowMajor>& cross_section,
              rd::RadarModel<double,1,Eigen::RowMajor>&           radar)
             { radar.findProbabilityOfDetection(aircraft_pose, radar_position, cross_section); },
           cross_section_model,
           "PD - spikeball");
  timeFunc([](Eigen::Matrix<double,1,6,Eigen::RowMajor>&          aircraft_pose,
              Eigen::Matrix<double,1,3,Eigen::RowMajor>&          radar_position,
              rd::CrossSectionModelPtr<double,1,Eigen::RowMajor>& cross_section,
              rd::RadarModel<double,1,Eigen::RowMajor>&           radar)
             { radar.findProbabilityOfDetectionPDAircraftPose(aircraft_pose, radar_position, cross_section); },
           cross_section_model,
           "PD PD aircraft - spikeball");
  timeFunc([](Eigen::Matrix<double,1,6,Eigen::RowMajor>&          aircraft_pose,
              Eigen::Matrix<double,1,3,Eigen::RowMajor>&          radar_position,
              rd::CrossSectionModelPtr<double,1,Eigen::RowMajor>& cross_section,
              rd::RadarModel<double,1,Eigen::RowMajor>&           radar)
             {
               radar.findProbabilityOfDetectionPDRadarPosition(aircraft_pose, radar_position, cross_section);
               radar.findProbabilityOfDetectionPDConsolidatedRadarConstant(aircraft_pose, radar_position, cross_section);
             },
           cross_section_model,
           "PD PD radar - spikeball");

}

/* time_test.cpp */
