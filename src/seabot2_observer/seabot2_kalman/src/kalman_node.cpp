#include "seabot2_kalman/kalman_node.hpp"
#include <algorithm>    // std::sort

using namespace std::placeholders;

KalmanNode::KalmanNode()
        : Node("kalman_node"){

    init_parameters();
    init_interfaces();

    init_kalman(xhat_);

    RCLCPP_INFO(this->get_logger(), "[Kalman_node] Start Ok");
}

void KalmanNode::init_parameters() {
    this->declare_parameter<int>("loop_dt_", loop_dt_.count());
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("dt", loop_dt_.count()));

    this->declare_parameter<double>("physics_rho", physics_rho_);
    this->declare_parameter<double>("physics_g", physics_g_);
    this->declare_parameter<double>("robot_mass", robot_mass_);
    this->declare_parameter<double>("robot_diameter", robot_diameter_);
    this->declare_parameter<double>("screw_thread", screw_thread_);
    this->declare_parameter<double>("tick_per_turn", tick_per_turn_);
    this->declare_parameter<double>("piston_diameter", piston_diameter_);
    this->declare_parameter<double>("piston_max_tick_value", piston_max_tick_value_);

    this->declare_parameter<double>("enable_kalman_depth", enable_kalman_depth_);
    this->declare_parameter<double>("piston_volume_eq_init", piston_volume_eq_init_);
    this->declare_parameter<double>("init_chi", init_chi_);
    this->declare_parameter<double>("init_chi2", init_chi2_);
    this->declare_parameter<double>("init_volume_air", init_volume_air_);

    this->declare_parameter<double>("gamma_alpha_velocity", gamma_alpha_velocity_);
    this->declare_parameter<double>("gamma_alpha_depth", gamma_alpha_depth_);
    this->declare_parameter<double>("gamma_alpha_offset", gamma_alpha_offset_);
    this->declare_parameter<double>("gamma_alpha_chi", gamma_alpha_chi_);
    this->declare_parameter<double>("gamma_alpha_chi2", gamma_alpha_chi2_);
    this->declare_parameter<double>("gamma_alpha_cz", gamma_alpha_cz_);
    this->declare_parameter<double>("gamma_alpha_volume_air", gamma_alpha_volume_air_);

    this->declare_parameter<double>("gamma_init_velocity", gamma_init_velocity_);
    this->declare_parameter<double>("gamma_init_depth", gamma_init_depth_);
    this->declare_parameter<double>("gamma_init_offset", gamma_init_offset_);
    this->declare_parameter<double>("gamma_init_chi", gamma_init_chi_);
    this->declare_parameter<double>("gamma_init_chi2", gamma_init_chi2_);
    this->declare_parameter<double>("gamma_init_cz", gamma_init_cz_);
    this->declare_parameter<double>("gamma_init_volume_air", gamma_init_volume_air_);

    this->declare_parameter<double>("gamma_beta_depth", gamma_beta_depth_);

    physics_rho_ = this->get_parameter_or("physics_rho", physics_rho_);
    physics_g_ = this->get_parameter_or("physics_g", physics_g_);
    robot_mass_ = this->get_parameter_or("physics_mass", robot_mass_);
    robot_diameter_ = this->get_parameter_or("robot_diameter", robot_diameter_);
    screw_thread_ = this->get_parameter_or("screw_thread", screw_thread_);
    tick_per_turn_ = this->get_parameter_or("tick_per_turn", tick_per_turn_);
    piston_diameter_ = this->get_parameter_or("piston_diameter", piston_diameter_);
    piston_max_tick_value_ = this->get_parameter_or("piston_max_tick_value", piston_max_tick_value_);

    enable_kalman_depth_ = this->get_parameter_or("enable_kalman_depth", enable_kalman_depth_);
    piston_volume_eq_init_ = this->get_parameter_or("piston_volume_eq_init", piston_volume_eq_init_);
    init_chi_ = this->get_parameter_or("init_chi", init_chi_);
    init_chi2_ = this->get_parameter_or("init_chi2", init_chi2_);
    init_volume_air_ = this->get_parameter_or("init_volume_air", init_volume_air_);

    gamma_alpha_velocity_ = this->get_parameter_or("gamma_alpha_velocity", gamma_alpha_velocity_);
    gamma_alpha_depth_ = this->get_parameter_or("gamma_alpha_depth", gamma_alpha_depth_);
    gamma_alpha_offset_ = this->get_parameter_or("gamma_alpha_offset", gamma_alpha_offset_);
    gamma_alpha_chi_ = this->get_parameter_or("gamma_alpha_chi", gamma_alpha_chi_);
    gamma_alpha_chi2_ = this->get_parameter_or("gamma_alpha_chi2", gamma_alpha_chi2_);
    gamma_alpha_cz_ = this->get_parameter_or("gamma_alpha_cz", gamma_alpha_cz_);
    gamma_alpha_volume_air_ = this->get_parameter_or("gamma_alpha_volume_air", gamma_alpha_volume_air_);

    gamma_init_velocity_ = this->get_parameter_or("gamma_init_velocity", gamma_init_velocity_);
    gamma_init_depth_ = this->get_parameter_or("gamma_init_depth", gamma_init_depth_);
    gamma_init_offset_ = this->get_parameter_or("gamma_init_offset", gamma_init_offset_);
    gamma_init_chi_ = this->get_parameter_or("gamma_init_chi", gamma_init_chi_);
    gamma_init_chi2_ = this->get_parameter_or("gamma_init_chi2", gamma_init_chi2_);
    gamma_init_cz_ = this->get_parameter_or("gamma_init_cz", gamma_init_cz_);
    gamma_init_volume_air_ = this->get_parameter_or("gamma_init_volume_air", gamma_init_volume_air_);

    gamma_beta_depth_ = this->get_parameter_or("gamma_beta_depth", gamma_beta_depth_);

    /// Computed parameters
    Cf_ = M_PI*pow(robot_diameter_/2.0, 2);
    tick_to_volume_ = (screw_thread_/tick_per_turn_)*pow(piston_diameter_/2.0, 2)*M_PI;
    coeff_A_ = physics_g_ * physics_rho_ / (2.0 * robot_mass_);
    coeff_B_ = 0.5 * physics_rho_ * Cf_ / (2.0 * robot_mass_);
}

void KalmanNode::state_callback(const seabot2_piston_driver::msg::PistonState &msg){
    piston_position_last_ = piston_position_;
    piston_position_ = msg.position;
    piston_set_point_ = msg.position_set_point;
    piston_stamp_ = msg.header.stamp;

    compute_kalman(false, true);
}

void KalmanNode::depth_callback(const seabot2_depth_filter::msg::DepthPose &msg){
    fusion_depth_ = msg.depth;
    fusion_velocity_ = msg.velocity;
    fusion_stamp_ = msg.header.stamp;

    compute_kalman(true, false);
}

void KalmanNode::init_interfaces() {
    publisher_kalman_ = this->create_publisher<seabot2_kalman::msg::KalmanState>("kalman", 10);

    subscriber_state_data_ = this->create_subscription<seabot2_piston_driver::msg::PistonState>(
            "/driver/state", 10, std::bind(&KalmanNode::state_callback, this, _1));
    subscriber_depth_data_ = this->create_subscription<seabot2_depth_filter::msg::DepthPose>(
            "/observer/depth", 10, std::bind(&KalmanNode::depth_callback, this, _1));

}

Matrix<double,NB_STATES, 1> KalmanNode::f_dyn(const Matrix<double,NB_STATES,1> &x, const Matrix<double,NB_COMMAND, 1> &u){
    Matrix<double,NB_STATES, 1> dx = Matrix<double,NB_STATES, 1>::Zero();

    /// ToDo : change equation to assert PV = nRT => V = nR(T/P) and take into account Temperature and Pressure instead of only depth

    if(x(1)>0.)
        dx(0) = -coeff_A_*(u(0)+x(2)-(x(6)/(x(1)+1.0)+x(3)*x(1)+x(4)*pow(x(1),2)))-coeff_B_*x(5)*copysign(x(0)*x(0), x(0));
    dx(1) = x(0);
    dx(2) = 0.0;
    dx(3) = 0.0;
    dx(4) = 0.0;
    dx(5) = 0.0;
    dx(6) = 0.0;
    return dx;
}

void KalmanNode::kalman_predict(Matrix<double,NB_STATES, 1> &x,
                    Matrix<double,NB_STATES, NB_STATES> &gamma,
                    const Matrix<double,NB_COMMAND, 1> &u,
                    const Matrix<double,NB_STATES, NB_STATES> &gamma_alpha,
                    const double &dt){
    if(dt <= 0.0 || dt >= 1.0){
        RCLCPP_INFO(this->get_logger(), "[Kalman_node] dt issue %f", dt);
        return;
    }

    Matrix<double, NB_STATES, NB_STATES> Ak_tmp = Matrix<double, NB_STATES, NB_STATES>::Identity();
    Matrix<double,NB_STATES, NB_STATES> Ak = Matrix<double, NB_STATES, NB_STATES>::Zero();
    Ak(0,0) = -2.*coeff_B_*abs(x(0))*x(5);
    Ak(0,1) = coeff_A_*(x(3)+2.*x(4)*x(1));
    Ak(0,2) = -coeff_A_;
    Ak(0,3) = x(1)*coeff_A_;
    Ak(0,4) = pow(x(1),2)*coeff_A_;
    Ak(0,5) = -coeff_B_*abs(x(0))*x(0);
    if(x(1)>0.)
        Ak(0,6) = coeff_A_/(x(1)+1.0);
    Ak(1, 0) = 1.;
    Ak_tmp += Ak*dt;

    gamma = Ak_tmp*gamma*Ak_tmp.transpose()+gamma_alpha*sqrt(dt); // Variance estimatation
    x += f_dyn(x, u)*dt;  // New State estimation
}

void KalmanNode::kalman_correc(Matrix<double,NB_STATES, 1> &x,
                   Matrix<double,NB_STATES,NB_STATES> &gamma,
                   const Matrix<double,NB_MESURES, 1> &y,
                   const Matrix<double,NB_MESURES,NB_MESURES> &gamma_beta,
                   const Matrix<double,NB_MESURES, NB_STATES> &Ck){
    const Matrix<double,NB_MESURES,NB_MESURES> S = Ck * gamma * Ck.transpose() + gamma_beta;
    const Matrix<double,NB_STATES, NB_MESURES> K = gamma * Ck.transpose() * S.inverse();
    const Matrix<double,NB_MESURES, 1> ztilde = y - Ck*x;

    const Matrix<double,NB_STATES,NB_STATES> Id = Matrix<double,NB_STATES,NB_STATES>::Identity();
    const Matrix<double,NB_STATES,NB_STATES> tmp = Id - K*Ck;

//    gamma = ((tmp*gamma)*(gamma.transpose())*tmp.transpose()).sqrt();
    gamma = tmp*gamma;
    x += K*ztilde;
}

void KalmanNode::kalman(Matrix<double,NB_STATES, 1> &x,
            Matrix<double,NB_STATES,NB_STATES> &gamma,
            const Matrix<double,NB_COMMAND, 1> &u,
            const Matrix<double,NB_MESURES, 1> &y,
            const Matrix<double,NB_STATES, NB_STATES> &gamma_alpha,
            const Matrix<double,NB_MESURES,NB_MESURES> &gamma_beta,
            const Matrix<double,NB_MESURES, NB_STATES> &Ck,
            const double &dt
){
    kalman_correc(x, gamma, y, gamma_beta, Ck);
    kalman_predict(x, gamma, u, gamma_alpha, dt);
}

void KalmanNode::init_kalman(Matrix<double, NB_STATES, 1> &xhat ){
    xhat(0) = fusion_velocity_;
    xhat(1) = fusion_depth_;
    xhat(2) = piston_volume_eq_init_; // Vp
    xhat(3) = init_chi_; // chi
    xhat(4) = init_chi2_; // chi2
    xhat(5) = 1.0; // Cz
    xhat(6) = init_volume_air_; // Cz
    x_forcast_ = xhat;

    gamma_ = Matrix<double,NB_STATES,NB_STATES>::Zero();
    gamma_(0,0) = pow(gamma_init_velocity_, 2); // velocity
    gamma_(1,1) = pow(gamma_init_depth_, 2); // Depth
    gamma_(2,2) = pow(gamma_init_offset_, 2); // Error offset;
    gamma_(3,3) = pow(gamma_init_chi_,2); // Compressibility
    gamma_(4,4) = pow(gamma_init_chi2_,2); // Compressibility 2
    gamma_(5,5) = pow(gamma_init_cz_,2); // Cz
    gamma_(6,6) = pow(gamma_init_volume_air_,2); // Cz

    gamma_alpha_(0,0) = pow(gamma_alpha_velocity_, 2); // Velocity
    gamma_alpha_(1,1) = pow(gamma_alpha_depth_, 2); // Depth
    gamma_alpha_(2,2) = pow(gamma_alpha_offset_, 2); // Offset
    gamma_alpha_(3,3) = pow(gamma_alpha_chi_, 2); // Compressibility
    gamma_alpha_(4,4) = pow(gamma_alpha_chi2_, 2); // Compressibility 2
    gamma_alpha_(5,5) = pow(gamma_alpha_cz_, 2); // cz
    gamma_alpha_(6,6) = pow(gamma_init_volume_air_, 2); // cz

    gamma_beta_(0, 0) = pow(gamma_beta_depth_, 2); // Depth

    x_forcast_ = xhat_;

    Ck_(0, 1) = 1.;
}

void KalmanNode::compute_kalman(bool new_depth_data, bool new_piston_data) {
    seabot2_kalman::msg::KalmanState msg;

    if(fusion_depth_>enable_kalman_depth_ && enable_kalman_) {
        Matrix<double,NB_COMMAND, 1> u = Matrix<double,NB_COMMAND, 1>::Zero();
        u(0) = -piston_position_ * tick_to_volume_; // u

        if (new_depth_data) {
            Matrix<double,NB_MESURES, 1> y = Matrix<double,NB_MESURES, 1>::Zero();
            y(0) = fusion_depth_;

            double dt = (fusion_stamp_ - time_last_predict_).seconds();
            if(dt<0){
                RCLCPP_WARN(this->get_logger(), "[Kalman_node] depth data received late %f", dt);
                kalman_predict(xhat_, gamma_, u, gamma_alpha_, dt);
                kalman_correc(xhat_, gamma_, y, gamma_beta_, Ck_);
                kalman_predict(xhat_, gamma_, u, gamma_alpha_, -dt);
            }
            else {
                kalman_predict(xhat_, gamma_, u, gamma_alpha_, dt);
                kalman_correc(xhat_, gamma_, y, gamma_beta_, Ck_);
                time_last_predict_ = fusion_stamp_;
            }

            /// Forecast
            x_forcast_ = xhat_;
            gamma_forcast_ = gamma_;
            if(forecast_dt_ != 0ms){
                kalman_predict(x_forcast_, gamma_forcast_, u, gamma_alpha_,
                               (std::chrono::duration<double>(forecast_dt_)).count());
            }

        } else if (new_piston_data) {
            double dt = (piston_stamp_ - time_last_predict_).seconds();
            if(dt<0){
                RCLCPP_WARN(this->get_logger(), "[Kalman_node] piston data received late %f", dt);
                return ;
            }
            kalman_predict(xhat_, gamma_, u, gamma_alpha_, dt);
            time_last_predict_ = piston_stamp_;
        }

        /// Reset Kalman if divergence
        if(!xhat_.allFinite()) {
            init_kalman(xhat_);
            msg.valid = false;
        }
        else
            msg.valid = true;
    }
    /// Case where kalman is not enable, then follow fusion data
    else if(new_depth_data){
            time_last_predict_ = fusion_stamp_;
            xhat_(0) = fusion_velocity_;
            xhat_(1) = fusion_depth_;
            x_forcast_ = xhat_;
            gamma_forcast_ = gamma_;
            msg.valid = false;
    }

    if(new_depth_data){
        msg.velocity = x_forcast_(0);
        msg.depth = x_forcast_(1);
        msg.offset = x_forcast_(2);
        msg.chi = x_forcast_(3);
        msg.chi2 = x_forcast_(4);
        msg.cz = x_forcast_(5);
        msg.volume_air = x_forcast_(6);
        if(x_forcast_(1)!=-1.0)
            msg.offset_total = x_forcast_(2)+x_forcast_(6)/(x_forcast_(1)+1.0)+x_forcast_(3)*x_forcast_(1) + x_forcast_(4)*pow(x_forcast_(1),2);
        msg.header.stamp = time_last_predict_;

        msg.variance[0] = gamma_forcast_(0,0);
        msg.variance[1] = gamma_forcast_(1,1);
        msg.variance[2] = gamma_forcast_(2,2);
        msg.variance[3] = gamma_forcast_(3,3);
        msg.variance[4] = gamma_forcast_(4,4);
        msg.variance[5] = gamma_forcast_(5,5);
        msg.variance[6] = gamma_forcast_(6,6);

        publisher_kalman_->publish(msg);
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanNode>());
    rclcpp::shutdown();
    return 0;
}