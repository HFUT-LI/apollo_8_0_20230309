/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/control/controller/lat_controller.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/linear_quadratic_regulator.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/control/common/control_gflags.h"

namespace apollo {
namespace control {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleStateProvider;
using Matrix = Eigen::MatrixXd;
using apollo::cyber::Clock;

namespace {

std::string GetLogFileName() {
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);
  std::tm time_tm;
  localtime_r(&raw_time, &time_tm);
  strftime(name_buffer, 80, "/tmp/steer_log_simple_optimal_%F_%H%M%S.csv",
           &time_tm);
  return std::string(name_buffer);
}

//保存到日志中的状态量
void WriteHeaders(std::ofstream &file_stream) {
  file_stream << "current_lateral_error,"                  //当前横向误差
              << "current_ref_heading,"                    //当前参考航向
              << "current_heading,"                        //当前航向
              << "current_heading_error,"                  //当前航向误差
              << "heading_error_rate,"                     //航向误差变化率
              << "lateral_error_rate,"                     //横向误差变化率
              << "current_curvature,"                      //当前曲率
              << "steer_angle,"                            //方向盘转角(控制的输出)
              << "steer_angle_feedforward,"                //前馈控制计算输出
              << "steer_angle_lateral_contribution,"       //K矩阵中横向偏差计算结果分量
              << "steer_angle_lateral_rate_contribution,"  //K矩阵中横向偏差变化率计算结果分量
              << "steer_angle_heading_contribution,"       //K矩阵中航向偏差计算结果分量
              << "steer_angle_heading_rate_contribution,"  //K矩阵中航向偏差变化率计算结果分量
              << "steer_angle_feedback,"                   //LQR计算出的方向盘控制量
              << "steering_position,"                      //当前底盘方向盘状态(底盘的执行)
              << "v" << std::endl;                         //车速
}
}  // namespace

/*
如果控制侧输出的就不行，可以调节权重，横向误差和航向误差的增大；
检查一下K矩阵的4个分量值是不是正常，前馈输出值是不是正常。
*/

LatController::LatController() : name_("LQR-based Lateral Controller") {
  if (FLAGS_enable_csv_debug) {
    steer_log_file_.open(GetLogFileName());
    steer_log_file_ << std::fixed;
    steer_log_file_ << std::setprecision(6);
    WriteHeaders(steer_log_file_);
  }
  AINFO << "Using " << name_;
}

LatController::~LatController() { CloseLogFile(); }

//读取横向配置中的参数
bool LatController::LoadControlConf(const ControlConf *control_conf) {
  if (!control_conf) {
    AERROR << "[LatController] control_conf == nullptr";
    return false;
  }
  vehicle_param_ =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param(); //加载车辆配置参数，反序列化配置文件

  ts_ = control_conf->lat_controller_conf().ts();
  //异常检查 ts代表这个模块的执行频率是多少 如果ts小于0，为无效的横向控制时间间隔，
  if (ts_ <= 0.0) {
    AERROR << "[MPCController] Invalid control update interval.";
    return false;
  }
  cf_ = control_conf->lat_controller_conf().cf(); //前两轮轮胎侧扁刚度
  cr_ = control_conf->lat_controller_conf().cr(); //后两轮轮胎侧扁刚度
  preview_window_ = control_conf->lat_controller_conf().preview_window();
  //低速前行预瞄距离，修复不同速度下参考点的对准问题
  lookahead_station_low_speed_ =
      control_conf->lat_controller_conf().lookahead_station();
  lookback_station_low_speed_ =
      control_conf->lat_controller_conf().lookback_station(); //低速倒车预瞄距离
  lookahead_station_high_speed_ =
      control_conf->lat_controller_conf().lookahead_station_high_speed();
  lookback_station_high_speed_ =
      control_conf->lat_controller_conf().lookback_station_high_speed();
  wheelbase_ = vehicle_param_.wheel_base();   //轴距，直接在车辆参数中取，车辆的固定参数（默认不变）
  steer_ratio_ = vehicle_param_.steer_ratio();//转向比例，方向盘角度和车轮角度比值
  steer_single_direction_max_degree_ =
      vehicle_param_.max_steer_angle() / M_PI * 180; //最大方向盘转角
  max_lat_acc_ = control_conf->lat_controller_conf().max_lateral_acceleration();//最大的侧向加速度
  low_speed_bound_ = control_conf_->lon_controller_conf().switch_speed(); //纵向配置中：低速约束值
  low_speed_window_ =
      control_conf_->lon_controller_conf().switch_speed_window();

  const double mass_fl = control_conf->lat_controller_conf().mass_fl(); //车辆左前质量
  const double mass_fr = control_conf->lat_controller_conf().mass_fr();
  const double mass_rl = control_conf->lat_controller_conf().mass_rl();
  const double mass_rr = control_conf->lat_controller_conf().mass_rr();
  const double mass_front = mass_fl + mass_fr; //车身前悬质量
  const double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear; //车身总质量

  lf_ = wheelbase_ * (1.0 - mass_front / mass_); //车辆前悬长度
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);  //车辆后悬长度

  // moment of inertia
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear; //转动惯量

  lqr_eps_ = control_conf->lat_controller_conf().eps(); //lqr优化的最小精度0.01 停止条件1
  lqr_max_iteration_ = control_conf->lat_controller_conf().max_iteration(); //lqr最大迭代次数 停止条件2

  query_relative_time_ = control_conf->query_relative_time(); //相对时间 机构延时 一般来说发出指令到底盘响应会有500ms延时

  minimum_speed_protection_ = control_conf->minimum_speed_protection(); //最小速度

  return true;
}

void LatController::ProcessLogs(const SimpleLateralDebug *debug,
                                const canbus::Chassis *chassis) {
  const std::string log_str = absl::StrCat(
      debug->lateral_error(), ",", debug->ref_heading(), ",", debug->heading(),
      ",", debug->heading_error(), ",", debug->heading_error_rate(), ",",
      debug->lateral_error_rate(), ",", debug->curvature(), ",",
      debug->steer_angle(), ",", debug->steer_angle_feedforward(), ",",
      debug->steer_angle_lateral_contribution(), ",",
      debug->steer_angle_lateral_rate_contribution(), ",",
      debug->steer_angle_heading_contribution(), ",",
      debug->steer_angle_heading_rate_contribution(), ",",
      debug->steer_angle_feedback(), ",", chassis->steering_percentage(), ",",
      injector_->vehicle_state()->linear_velocity());
  if (FLAGS_enable_csv_debug) {
    steer_log_file_ << log_str << std::endl;
  }
  ADEBUG << "Steer_Control_Detail: " << log_str;
}

void LatController::LogInitParameters() {
  AINFO << name_ << " begin.";
  AINFO << "[LatController parameters]"
        << " mass_: " << mass_ << ","
        << " iz_: " << iz_ << ","
        << " lf_: " << lf_ << ","
        << " lr_: " << lr_;
}

//低通滤波器 去除高频噪声 效果同积分器
void LatController::InitializeFilters(const ControlConf *control_conf) {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  common::LpfCoefficients(
      ts_, control_conf->lat_controller_conf().cutoff_freq(), &den, &num);
  digital_filter_.set_coefficients(den, num);
  //偏差均值滤波 防止突变（导航模式用到了，正常车跑的时候没有用到）
  lateral_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->lat_controller_conf().mean_filter_window_size()));
  heading_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->lat_controller_conf().mean_filter_window_size()));
}

//控制器初始化，动力学模型初始化
Status LatController::Init(std::shared_ptr<DependencyInjector> injector,
                           const ControlConf *control_conf) {
  control_conf_ = control_conf;
  injector_ = injector;
  if (!LoadControlConf(control_conf_)) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "failed to load control_conf");
  }
  // Matrix init operations.
  const int matrix_size = basic_state_size_ + preview_window_;       //矩阵大小默认为基本状态大小 4
  matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);    //矩阵A 常量
  matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);   //离散化矩阵A
  matrix_adc_ = Matrix::Zero(matrix_size, matrix_size);              //preview=0时，matrix_adc_ == matrix_ad_
  /*
  A matrix (Gear Drive)
  [0.0, 1.0, 0.0, 0.0;
   0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
   (l_r * c_r - l_f * c_f) / m / v;
   0.0, 0.0, 0.0, 1.0;
   0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
   (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
  */
  //先初始化速度无关项
  matrix_a_(0, 1) = 1.0;                             //矩阵A第1行第2列
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;             //矩阵A第2行第3列
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;

  //初始化速度相关项
  matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  /*
  b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
  */
  //初始化B矩阵
  matrix_b_ = Matrix::Zero(basic_state_size_, 1);
  matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
  matrix_bdc_ = Matrix::Zero(matrix_size, 1);
  //矩阵B
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_bd_ = matrix_b_ * ts_; //（简单的线性化？）
  //LQR权重矩阵
  matrix_state_ = Matrix::Zero(matrix_size, 1); //状态量 lateral error，error rate；heading error，error rate
  matrix_k_ = Matrix::Zero(1, matrix_size);     //K反馈矩阵（1x4），LQR计算结果
  matrix_r_ = Matrix::Identity(1, 1);           //R（1x1），控制量作用权重
  matrix_q_ = Matrix::Zero(matrix_size, matrix_size); //Q（4x4），状态量权重

  int q_param_size = control_conf_->lat_controller_conf().matrix_q_size();
  int reverse_q_param_size =
      control_conf_->lat_controller_conf().reverse_matrix_q_size();
  if (matrix_size != q_param_size || matrix_size != reverse_q_param_size) {
    const auto error_msg = absl::StrCat(
        "lateral controller error: matrix_q size: ", q_param_size,
        "lateral controller error: reverse_matrix_q size: ",
        reverse_q_param_size,
        " in parameter file not equal to matrix_size: ", matrix_size);
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  }

  for (int i = 0; i < q_param_size; ++i) {
    matrix_q_(i, i) = control_conf_->lat_controller_conf().matrix_q(i);
  }

  matrix_q_updated_ = matrix_q_;
  InitializeFilters(control_conf_);                                  //初始化滤波器 均值滤波 低通数字滤波
  auto &lat_controller_conf = control_conf_->lat_controller_conf();
  LoadLatGainScheduler(lat_controller_conf);                         //权重矩阵Q关联速度反比例增益，削弱1、3权重
  LogInitParameters();                                               //初始化控制系统参数，通过函数4来实现

  //滞后超前控制器
  enable_leadlag_ = control_conf_->lat_controller_conf()
                        .enable_reverse_leadlag_compensation();
  if (enable_leadlag_) {
    leadlag_controller_.Init(lat_controller_conf.reverse_leadlag_conf(), ts_);
  }
  
  //模型参考自适应控制器
  enable_mrac_ =
      control_conf_->lat_controller_conf().enable_steer_mrac_control();
  if (enable_mrac_) {
    mrac_controller_.Init(lat_controller_conf.steer_mrac_conf(),
                          vehicle_param_.steering_latency_param(), ts_);
  }

  enable_look_ahead_back_control_ =
      control_conf_->lat_controller_conf().enable_look_ahead_back_control();

  return Status::OK();
}

void LatController::CloseLogFile() {
  if (FLAGS_enable_csv_debug && steer_log_file_.is_open()) {
    steer_log_file_.close();
  }
}

void LatController::LoadLatGainScheduler(
    const LatControllerConf &lat_controller_conf) {
  const auto &lat_err_gain_scheduler =
      lat_controller_conf.lat_err_gain_scheduler();                        //不同速度下的横向偏差增益
  const auto &heading_err_gain_scheduler =
      lat_controller_conf.heading_err_gain_scheduler();                    //不同速度下的航向偏差增益
  AINFO << "Lateral control gain scheduler loaded";
  Interpolation1D::DataType xy1, xy2;                                      //加载标定表
  for (const auto &scheduler : lat_err_gain_scheduler.scheduler()) {
    xy1.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));   //横向偏差标定表
  }
  for (const auto &scheduler : heading_err_gain_scheduler.scheduler()) {
    xy2.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));   //航向偏差标定表
  }

  lat_err_interpolation_.reset(new Interpolation1D);                       //一维线形插值
  ACHECK(lat_err_interpolation_->Init(xy1))
      << "Fail to load lateral error gain scheduler";

  heading_err_interpolation_.reset(new Interpolation1D);
  ACHECK(heading_err_interpolation_->Init(xy2))
      << "Fail to load heading error gain scheduler";
}

void LatController::Stop() { CloseLogFile(); }

std::string LatController::Name() const { return name_; }

Status LatController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    ControlCommand *cmd) {
  auto vehicle_state = injector_->vehicle_state();

  auto target_tracking_trajectory = *planning_published_trajectory;
  
  //导航模式（暂不考虑）
  if (FLAGS_use_navigation_mode &&
      FLAGS_enable_navigation_mode_position_update) {
    auto time_stamp_diff =
        planning_published_trajectory->header().timestamp_sec() -
        current_trajectory_timestamp_;

    auto curr_vehicle_x = localization->pose().position().x();
    auto curr_vehicle_y = localization->pose().position().y();

    double curr_vehicle_heading = 0.0;
    const auto &orientation = localization->pose().orientation();
    if (localization->pose().has_heading()) {
      curr_vehicle_heading = localization->pose().heading();
    } else {
      curr_vehicle_heading =
          common::math::QuaternionToHeading(orientation.qw(), orientation.qx(),
                                            orientation.qy(), orientation.qz());
    }

    // new planning trajectory
    if (time_stamp_diff > 1.0e-6) {
      init_vehicle_x_ = curr_vehicle_x;
      init_vehicle_y_ = curr_vehicle_y;
      init_vehicle_heading_ = curr_vehicle_heading;

      current_trajectory_timestamp_ =
          planning_published_trajectory->header().timestamp_sec();
    } else {
      auto x_diff_map = curr_vehicle_x - init_vehicle_x_;
      auto y_diff_map = curr_vehicle_y - init_vehicle_y_;
      auto theta_diff = curr_vehicle_heading - init_vehicle_heading_;

      auto cos_map_veh = std::cos(init_vehicle_heading_);
      auto sin_map_veh = std::sin(init_vehicle_heading_);

      auto x_diff_veh = cos_map_veh * x_diff_map + sin_map_veh * y_diff_map;
      auto y_diff_veh = -sin_map_veh * x_diff_map + cos_map_veh * y_diff_map;

      auto cos_theta_diff = std::cos(-theta_diff);
      auto sin_theta_diff = std::sin(-theta_diff);

      auto tx = -(cos_theta_diff * x_diff_veh - sin_theta_diff * y_diff_veh);
      auto ty = -(sin_theta_diff * x_diff_veh + cos_theta_diff * y_diff_veh);

      auto ptr_trajectory_points =
          target_tracking_trajectory.mutable_trajectory_point();
      std::for_each(
          ptr_trajectory_points->begin(), ptr_trajectory_points->end(),
          [&cos_theta_diff, &sin_theta_diff, &tx, &ty,
           &theta_diff](common::TrajectoryPoint &p) {
            auto x = p.path_point().x();
            auto y = p.path_point().y();
            auto theta = p.path_point().theta();

            auto x_new = cos_theta_diff * x - sin_theta_diff * y + tx;
            auto y_new = sin_theta_diff * x + cos_theta_diff * y + ty;
            auto theta_new = common::math::NormalizeAngle(theta - theta_diff);

            p.mutable_path_point()->set_x(x_new);
            p.mutable_path_point()->set_y(y_new);
            p.mutable_path_point()->set_theta(theta_new);
          });
    }
  }

  trajectory_analyzer_ =
      std::move(TrajectoryAnalyzer(&target_tracking_trajectory));

  // Transform the coordinate of the planning trajectory from the center of the
  // rear-axis to the center of mass, if conditions matched
  // 低速时使用几何中心点（比如倒车时选择后轴中心，前进时选择几何中心，路测时反馈比较好）
  if (((FLAGS_trajectory_transform_to_com_reverse &&
        vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) ||
       (FLAGS_trajectory_transform_to_com_drive &&
        vehicle_state->gear() == canbus::Chassis::GEAR_DRIVE)) &&
      enable_look_ahead_back_control_) {
    trajectory_analyzer_.TrajectoryTransformToCOM(lr_);
  }

  // Re-build the vehicle dynamic models at reverse driving (in particular,
  // replace the lateral translational motion dynamics with the corresponding
  // kinematic models)
  // 倒车动力学模型 对矩阵A个别项进行了修改
  if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) {
    /*
    A matrix (Gear Reverse)
    [0.0, 0.0, 1.0 * v 0.0;
     0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
     (l_r * c_r - l_f * c_f) / m / v;
     0.0, 0.0, 0.0, 1.0;
     0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
     (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */
    cf_ = -control_conf_->lat_controller_conf().cf();
    cr_ = -control_conf_->lat_controller_conf().cr();
    matrix_a_(0, 1) = 0.0;
    matrix_a_coeff_(0, 2) = 1.0;
  } else {
    /*
    A matrix (Gear Drive)
    [0.0, 1.0, 0.0, 0.0;
     0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
     (l_r * c_r - l_f * c_f) / m / v;
     0.0, 0.0, 0.0, 1.0;
     0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
     (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */
    cf_ = control_conf_->lat_controller_conf().cf();
    cr_ = control_conf_->lat_controller_conf().cr();
    matrix_a_(0, 1) = 1.0;
    matrix_a_coeff_(0, 2) = 0.0;
  }
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  /*
  b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
  */
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_bd_ = matrix_b_ * ts_;

  UpdateDrivingOrientation();            //当前车辆驾驶方向判断 更新状态

  SimpleLateralDebug *debug = cmd->mutable_debug()->mutable_simple_lat_debug();
  debug->Clear();

  // Update state = [Lateral Error, Lateral Error Rate, Heading Error, Heading
  // Error Rate, preview lateral error1 , preview lateral error2, ...]
  UpdateState(debug);                   //更新当前状态量偏差等 保存至debug方便后续调试

  UpdateMatrix();                       //对连续的状态空间方程作离散化处理

  // Compound discrete matrix with road preview model
  UpdateMatrixCompound();               //更新matrix_adc_ matrix_bdc_ ,AB矩阵成型

  // Adjust matrix_q_updated when in reverse gear
  int q_param_size = control_conf_->lat_controller_conf().matrix_q_size(); //读取q矩阵参数大小
  int reverse_q_param_size =
      control_conf_->lat_controller_conf().reverse_matrix_q_size();
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) { //赋值q矩阵
    for (int i = 0; i < reverse_q_param_size; ++i) {
      matrix_q_(i, i) =
          control_conf_->lat_controller_conf().reverse_matrix_q(i);
    }
  } else {
    for (int i = 0; i < q_param_size; ++i) {
      matrix_q_(i, i) = control_conf_->lat_controller_conf().matrix_q(i);
    }
  }

  // Add gain scheduler for higher speed steering
  if (FLAGS_enable_gain_scheduler) {              //根据车辆速度对横向偏差及航向偏差增益进行插值
    matrix_q_updated_(0, 0) =                     //高速时对应降低相关权重，避免猛打方向
        matrix_q_(0, 0) * lat_err_interpolation_->Interpolate(
                              std::fabs(vehicle_state->linear_velocity()));
    matrix_q_updated_(2, 2) =
        matrix_q_(2, 2) * heading_err_interpolation_->Interpolate(
                              std::fabs(vehicle_state->linear_velocity()));
    common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_updated_,  //lqr求解器
                                  matrix_r_, lqr_eps_, lqr_max_iteration_,      //输入为A B Q R eps iteration
                                  &matrix_k_);                                  //输出为 矩阵K 1*4 状态量权重系数
  } else {
    common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_,
                                  matrix_r_, lqr_eps_, lqr_max_iteration_,
                                  &matrix_k_);
  }

  // feedback = - K * state
  // Convert vehicle steer angle from rad to degree and then to steer degree
  // then to 100% ratio
  const double steer_angle_feedback = -(matrix_k_ * matrix_state_)(0, 0) * 180 /  //侧偏刚度单位是N/弧度
                                      M_PI * steer_ratio_ /
                                      steer_single_direction_max_degree_ * 100;   //把方向盘转角从弧度转换成角度

  const double steer_angle_feedforward = ComputeFeedForward(debug->curvature());  //前馈控制求解

  double steer_angle = 0.0;
  double steer_angle_feedback_augment = 0.0;         //控制器滞后超前补偿值
  // Augment the feedback control on lateral error at the desired speed domain
  if (enable_leadlag_) {
    if (FLAGS_enable_feedback_augment_on_high_speed ||
        std::fabs(vehicle_state->linear_velocity()) < low_speed_bound_) {
      steer_angle_feedback_augment =
          leadlag_controller_.Control(-matrix_state_(0, 0), ts_) * 180 / M_PI *
          steer_ratio_ / steer_single_direction_max_degree_ * 100;
      if (std::fabs(vehicle_state->linear_velocity()) >
          low_speed_bound_ - low_speed_window_) {
        // Within the low-high speed transition window, linerly interplolate the
        // augment control gain for "soft" control switch
        steer_angle_feedback_augment = common::math::lerp(
            steer_angle_feedback_augment, low_speed_bound_ - low_speed_window_,
            0.0, low_speed_bound_, std::fabs(vehicle_state->linear_velocity()));
      }
    }
  }
  steer_angle = steer_angle_feedback + steer_angle_feedforward +
                steer_angle_feedback_augment;        //控制计算结果

  // Compute the steering command limit with the given maximum lateral
  // acceleration
  const double steer_limit =
      FLAGS_set_steer_limit ? std::atan(max_lat_acc_ * wheelbase_ /
                                        (vehicle_state->linear_velocity() *
                                         vehicle_state->linear_velocity())) *
                                  steer_ratio_ * 180 / M_PI /
                                  steer_single_direction_max_degree_ * 100
                            : 100.0;

  const double steer_diff_with_max_rate =
      FLAGS_enable_maximum_steer_rate_limit
          ? vehicle_param_.max_steer_angle_rate() * ts_ * 180 / M_PI /
                steer_single_direction_max_degree_ * 100
          : 100.0;

  const double steering_position = chassis->steering_percentage();

  // Re-compute the steering command if the MRAC control is enabled, with steer
  // angle limitation and steer rate limitation
  if (enable_mrac_) {
    const int mrac_model_order = control_conf_->lat_controller_conf()
                                     .steer_mrac_conf()
                                     .mrac_model_order();
    Matrix steer_state = Matrix::Zero(mrac_model_order, 1);
    steer_state(0, 0) = chassis->steering_percentage();
    if (mrac_model_order > 1) {
      steer_state(1, 0) = (steering_position - pre_steering_position_) / ts_;
    }
    if (std::fabs(vehicle_state->linear_velocity()) >
        control_conf_->minimum_speed_resolution()) {
      mrac_controller_.SetStateAdaptionRate(1.0);
      mrac_controller_.SetInputAdaptionRate(1.0);
    } else {
      mrac_controller_.SetStateAdaptionRate(0.0);
      mrac_controller_.SetInputAdaptionRate(0.0);
    }
    steer_angle = mrac_controller_.Control(
        steer_angle, steer_state, steer_limit, steer_diff_with_max_rate / ts_);
    // Set the steer mrac debug message
    MracDebug *mracdebug = debug->mutable_steer_mrac_debug();
    Matrix steer_reference = mrac_controller_.CurrentReferenceState();
    mracdebug->set_mrac_model_order(mrac_model_order);
    for (int i = 0; i < mrac_model_order; ++i) {
      mracdebug->add_mrac_reference_state(steer_reference(i, 0));
      mracdebug->add_mrac_state_error(steer_state(i, 0) -
                                      steer_reference(i, 0));
      mracdebug->mutable_mrac_adaptive_gain()->add_state_adaptive_gain(
          mrac_controller_.CurrentStateAdaptionGain()(i, 0));
    }
    mracdebug->mutable_mrac_adaptive_gain()->add_input_adaptive_gain(
        mrac_controller_.CurrentInputAdaptionGain()(0, 0));
    mracdebug->set_mrac_reference_saturation_status(
        mrac_controller_.ReferenceSaturationStatus());
    mracdebug->set_mrac_control_saturation_status(
        mrac_controller_.ControlSaturationStatus());
  }
  pre_steering_position_ = steering_position;
  debug->set_steer_mrac_enable_status(enable_mrac_);

  // Clamp the steer angle with steer limitations at current speed
  double steer_angle_limited =
      common::math::Clamp(steer_angle, -steer_limit, steer_limit);
  steer_angle = steer_angle_limited;
  debug->set_steer_angle_limited(steer_angle_limited);

  // Limit the steering command with the designed digital filter
  steer_angle = digital_filter_.Filter(steer_angle);
  steer_angle = common::math::Clamp(steer_angle, -100.0, 100.0);

  // Check if the steer is locked and hence the previous steer angle should be
  // executed
  if (std::abs(vehicle_state->linear_velocity()) < FLAGS_lock_steer_speed &&
      (vehicle_state->gear() == canbus::Chassis::GEAR_DRIVE ||
       vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) &&
      chassis->driving_mode() == canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    steer_angle = pre_steer_angle_;
  }

  // Set the steer commands
  cmd->set_steering_target(common::math::Clamp(
      steer_angle, pre_steer_angle_ - steer_diff_with_max_rate,
      pre_steer_angle_ + steer_diff_with_max_rate));
  cmd->set_steering_rate(FLAGS_steer_angle_rate);

  pre_steer_angle_ = cmd->steering_target();

  // compute extra information for logging and debugging
  const double steer_angle_lateral_contribution =
      -matrix_k_(0, 0) * matrix_state_(0, 0) * 180 / M_PI * steer_ratio_ /
      steer_single_direction_max_degree_ * 100;

  const double steer_angle_lateral_rate_contribution =
      -matrix_k_(0, 1) * matrix_state_(1, 0) * 180 / M_PI * steer_ratio_ /
      steer_single_direction_max_degree_ * 100;

  const double steer_angle_heading_contribution =
      -matrix_k_(0, 2) * matrix_state_(2, 0) * 180 / M_PI * steer_ratio_ /
      steer_single_direction_max_degree_ * 100;

  const double steer_angle_heading_rate_contribution =
      -matrix_k_(0, 3) * matrix_state_(3, 0) * 180 / M_PI * steer_ratio_ /
      steer_single_direction_max_degree_ * 100;

  debug->set_heading(driving_orientation_);
  debug->set_steer_angle(steer_angle);
  debug->set_steer_angle_feedforward(steer_angle_feedforward);
  debug->set_steer_angle_lateral_contribution(steer_angle_lateral_contribution);
  debug->set_steer_angle_lateral_rate_contribution(
      steer_angle_lateral_rate_contribution);
  debug->set_steer_angle_heading_contribution(steer_angle_heading_contribution);
  debug->set_steer_angle_heading_rate_contribution(
      steer_angle_heading_rate_contribution);
  debug->set_steer_angle_feedback(steer_angle_feedback);
  debug->set_steer_angle_feedback_augment(steer_angle_feedback_augment);
  debug->set_steering_position(steering_position);
  debug->set_ref_speed(vehicle_state->linear_velocity());

  ProcessLogs(debug, chassis);
  return Status::OK();
}

Status LatController::Reset() {
  matrix_state_.setZero();
  if (enable_mrac_) {
    mrac_controller_.Reset();
  }
  return Status::OK();
}

void LatController::UpdateState(SimpleLateralDebug *debug) {
  auto vehicle_state = injector_->vehicle_state();
  if (FLAGS_use_navigation_mode) {
    ComputeLateralErrors(
        0.0, 0.0, driving_orientation_, vehicle_state->linear_velocity(),
        vehicle_state->angular_velocity(), vehicle_state->linear_acceleration(),
        trajectory_analyzer_, debug);
  } else {
    // Transform the coordinate of the vehicle states from the center of the
    // rear-axis to the center of mass, if conditions matched
    const auto &com = vehicle_state->ComputeCOMPosition(lr_);
    ComputeLateralErrors(
        com.x(), com.y(), driving_orientation_,
        vehicle_state->linear_velocity(), vehicle_state->angular_velocity(),
        vehicle_state->linear_acceleration(), trajectory_analyzer_, debug);
  }

  // State matrix update;
  // First four elements are fixed;
  if (enable_look_ahead_back_control_) {
    matrix_state_(0, 0) = debug->lateral_error_feedback();
    matrix_state_(2, 0) = debug->heading_error_feedback();
  } else {
    matrix_state_(0, 0) = debug->lateral_error();
    matrix_state_(2, 0) = debug->heading_error();
  }
  matrix_state_(1, 0) = debug->lateral_error_rate();
  matrix_state_(3, 0) = debug->heading_error_rate();

  // Next elements are depending on preview window size;
  for (int i = 0; i < preview_window_; ++i) {
    const double preview_time = ts_ * (i + 1);
    const auto preview_point =
        trajectory_analyzer_.QueryNearestPointByRelativeTime(preview_time);

    const auto matched_point = trajectory_analyzer_.QueryNearestPointByPosition(
        preview_point.path_point().x(), preview_point.path_point().y());

    const double dx =
        preview_point.path_point().x() - matched_point.path_point().x();
    const double dy =
        preview_point.path_point().y() - matched_point.path_point().y();

    const double cos_matched_theta =
        std::cos(matched_point.path_point().theta());
    const double sin_matched_theta =
        std::sin(matched_point.path_point().theta());
    const double preview_d_error =
        cos_matched_theta * dy - sin_matched_theta * dx;

    matrix_state_(basic_state_size_ + i, 0) = preview_d_error;
  }
}

void LatController::UpdateMatrix() {
  double v;
  // At reverse driving, replace the lateral translational motion dynamics with
  // the corresponding kinematic models
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    v = std::min(injector_->vehicle_state()->linear_velocity(),
                 -minimum_speed_protection_);
    matrix_a_(0, 2) = matrix_a_coeff_(0, 2) * v;
  } else {
    v = std::max(injector_->vehicle_state()->linear_velocity(),
                 minimum_speed_protection_);
    matrix_a_(0, 2) = 0.0;
  }
  matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
  matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
  matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
  matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;
  Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
  matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *                 //双线性变换，主要是使频域不至于发生重叠
               (matrix_i + ts_ * 0.5 * matrix_a_);
}

void LatController::UpdateMatrixCompound() {
  // Initialize preview matrix
  matrix_adc_.block(0, 0, basic_state_size_, basic_state_size_) = matrix_ad_;
  matrix_bdc_.block(0, 0, basic_state_size_, 1) = matrix_bd_;
  if (preview_window_ > 0) {
    matrix_bdc_(matrix_bdc_.rows() - 1, 0) = 1;
    // Update A matrix;
    for (int i = 0; i < preview_window_ - 1; ++i) {
      matrix_adc_(basic_state_size_ + i, basic_state_size_ + 1 + i) = 1;
    }
  }
}

double LatController::ComputeFeedForward(double ref_curvature) const {
  const double kv =
      lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;

  // Calculate the feedforward term of the lateral controller; then change it
  // from rad to %
  const double v = injector_->vehicle_state()->linear_velocity();
  double steer_angle_feedforwardterm;
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) { //倒车时前馈使用?模型
    steer_angle_feedforwardterm = wheelbase_ * ref_curvature * 180 / M_PI *
                                  steer_ratio_ /
                                  steer_single_direction_max_degree_ * 100;
  } else {
    steer_angle_feedforwardterm =       //前进时使用动力学模型公式
        (wheelbase_ * ref_curvature + kv * v * v * ref_curvature -
         matrix_k_(0, 2) *
             (lr_ * ref_curvature -
              lf_ * mass_ * v * v * ref_curvature / 2 / cr_ / wheelbase_)) *
        180 / M_PI * steer_ratio_ / steer_single_direction_max_degree_ * 100;
  }

  return steer_angle_feedforwardterm;
}

void LatController::ComputeLateralErrors(
    const double x, const double y, const double theta, const double linear_v,
    const double angular_v, const double linear_a,
    const TrajectoryAnalyzer &trajectory_analyzer, SimpleLateralDebug *debug) {
  TrajectoryPoint target_point;

  if (FLAGS_query_time_nearest_point_only) {
    target_point = trajectory_analyzer.QueryNearestPointByAbsoluteTime(   //查找时间匹配点
        Clock::NowInSeconds() + query_relative_time_);//为了弥补控制指令到底盘响应的时间间隔
  } else {
    if (FLAGS_use_navigation_mode &&
        !FLAGS_enable_navigation_mode_position_update) {
      target_point = trajectory_analyzer.QueryNearestPointByAbsoluteTime(
          Clock::NowInSeconds() + query_relative_time_);
    } else {
      target_point = trajectory_analyzer.QueryNearestPointByPosition(x, y); //查找距离匹配点
    }
  }
  const double dx = x - target_point.path_point().x();
  const double dy = y - target_point.path_point().y();

  debug->mutable_current_target_point()->mutable_path_point()->set_x(
      target_point.path_point().x());
  debug->mutable_current_target_point()->mutable_path_point()->set_y(
      target_point.path_point().y());

  ADEBUG << "x point: " << x << " y point: " << y;
  ADEBUG << "match point information : " << target_point.ShortDebugString();

  const double cos_target_heading = std::cos(target_point.path_point().theta());
  const double sin_target_heading = std::sin(target_point.path_point().theta());

  double lateral_error = cos_target_heading * dy - sin_target_heading * dx;
  if (FLAGS_enable_navigation_mode_error_filter) {
    lateral_error = lateral_error_filter_.Update(lateral_error);
  }

  debug->set_lateral_error(lateral_error);

  debug->set_ref_heading(target_point.path_point().theta());
  double heading_error =
      common::math::NormalizeAngle(theta - debug->ref_heading());
  if (FLAGS_enable_navigation_mode_error_filter) {
    heading_error = heading_error_filter_.Update(heading_error);
  }
  debug->set_heading_error(heading_error);

  // Within the low-high speed transition window, linerly interplolate the
  // lookahead/lookback station for "soft" prediction window switch
  double lookahead_station = 0.0;
  double lookback_station = 0.0;
  if (std::fabs(linear_v) >= low_speed_bound_) {
    lookahead_station = lookahead_station_high_speed_;
    lookback_station = lookback_station_high_speed_;
  } else if (std::fabs(linear_v) < low_speed_bound_ - low_speed_window_) {
    lookahead_station = lookahead_station_low_speed_;
    lookback_station = lookback_station_low_speed_;
  } else {
    lookahead_station = common::math::lerp(
        lookahead_station_low_speed_, low_speed_bound_ - low_speed_window_,
        lookahead_station_high_speed_, low_speed_bound_, std::fabs(linear_v));
    lookback_station = common::math::lerp(
        lookback_station_low_speed_, low_speed_bound_ - low_speed_window_,
        lookback_station_high_speed_, low_speed_bound_, std::fabs(linear_v));
  }

  // Estimate the heading error with look-ahead/look-back windows as feedback
  // signal for special driving scenarios
  double heading_error_feedback;
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    heading_error_feedback = heading_error;
  } else {
    auto lookahead_point = trajectory_analyzer.QueryNearestPointByRelativeTime(
        target_point.relative_time() +
        lookahead_station /
            (std::max(std::fabs(linear_v), 0.1) * std::cos(heading_error)));
    heading_error_feedback = common::math::NormalizeAngle(
        heading_error + target_point.path_point().theta() -
        lookahead_point.path_point().theta());
  }
  debug->set_heading_error_feedback(heading_error_feedback);

  // Estimate the lateral error with look-ahead/look-back windows as feedback
  // signal for special driving scenarios
  double lateral_error_feedback;
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    lateral_error_feedback =
        lateral_error - lookback_station * std::sin(heading_error);
  } else {
    lateral_error_feedback =
        lateral_error + lookahead_station * std::sin(heading_error);
  }
  debug->set_lateral_error_feedback(lateral_error_feedback);

  auto lateral_error_dot = linear_v * std::sin(heading_error);
  auto lateral_error_dot_dot = linear_a * std::sin(heading_error);
  if (FLAGS_reverse_heading_control) {
    if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
      lateral_error_dot = -lateral_error_dot;
      lateral_error_dot_dot = -lateral_error_dot_dot;
    }
  }
  debug->set_lateral_error_rate(lateral_error_dot);
  debug->set_lateral_acceleration(lateral_error_dot_dot);
  debug->set_lateral_jerk(
      (debug->lateral_acceleration() - previous_lateral_acceleration_) / ts_);
  previous_lateral_acceleration_ = debug->lateral_acceleration();

  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    debug->set_heading_rate(-angular_v);
  } else {
    debug->set_heading_rate(angular_v);
  }
  debug->set_ref_heading_rate(target_point.path_point().kappa() *
                              target_point.v());
  debug->set_heading_error_rate(debug->heading_rate() -
                                debug->ref_heading_rate());

  debug->set_heading_acceleration(
      (debug->heading_rate() - previous_heading_rate_) / ts_);
  debug->set_ref_heading_acceleration(
      (debug->ref_heading_rate() - previous_ref_heading_rate_) / ts_);
  debug->set_heading_error_acceleration(debug->heading_acceleration() -
                                        debug->ref_heading_acceleration());
  previous_heading_rate_ = debug->heading_rate();
  previous_ref_heading_rate_ = debug->ref_heading_rate();

  debug->set_heading_jerk(
      (debug->heading_acceleration() - previous_heading_acceleration_) / ts_);
  debug->set_ref_heading_jerk(
      (debug->ref_heading_acceleration() - previous_ref_heading_acceleration_) /
      ts_);
  debug->set_heading_error_jerk(debug->heading_jerk() -
                                debug->ref_heading_jerk());
  previous_heading_acceleration_ = debug->heading_acceleration();
  previous_ref_heading_acceleration_ = debug->ref_heading_acceleration();

  debug->set_curvature(target_point.path_point().kappa());
}

void LatController::UpdateDrivingOrientation() {
  auto vehicle_state = injector_->vehicle_state();
  driving_orientation_ = vehicle_state->heading();
  matrix_bd_ = matrix_b_ * ts_;
  // Reverse the driving direction if the vehicle is in reverse mode
  if (FLAGS_reverse_heading_control) {
    if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) {
      driving_orientation_ =
          common::math::NormalizeAngle(driving_orientation_ + M_PI);
      // Update Matrix_b for reverse mode
      matrix_bd_ = -matrix_b_ * ts_;
      ADEBUG << "Matrix_b changed due to gear direction";
    }
  }
}

}  // namespace control
}  // namespace apollo
