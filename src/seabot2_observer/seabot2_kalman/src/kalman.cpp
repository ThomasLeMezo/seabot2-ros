#include "seabot2_kalman/kalman.h"

Kalman::Kalman(){
    init_kalman();
}

void Kalman::init_parameters(const rclcpp::Time &init_time){
    tick_to_volume_ = (screw_thread_/tick_per_turn_)*pow(piston_diameter_/2.0, 2)*M_PI;
    piston_max_volume_ = piston_max_tick_ * tick_to_volume_;
    gamma_init_offset_ = piston_max_tick_ * tick_to_volume_;

    gamma_init_offset_ = piston_max_tick_ * tick_to_volume_;

    Cf_ = M_PI*pow(robot_diameter_/2.0, 2);
    tick_to_volume_ = (screw_thread_/tick_per_turn_)*pow(piston_diameter_/2.0, 2)*M_PI;
    coeff_A_ = physics_g_ * physics_rho_ / (2.0 * robot_mass_);
    coeff_B_ = 0.5 * physics_rho_ * Cf_ / (2.0 * robot_mass_);

    time_last_predict_ = init_time;
    init_kalman();
}

void Kalman::update_density(double physics_rho){
    physics_rho_ = physics_rho;
    coeff_A_ = physics_g_ * physics_rho_ / (2.0 * robot_mass_);
    coeff_B_ = 0.5 * physics_rho_ * Cf_ / (2.0 * robot_mass_);
}

Matrix<double,Kalman::NB_STATES, 1> Kalman::f_dyn(const Matrix<double,NB_STATES,1> &x, const Matrix<double,NB_COMMAND, 1> &u) const{
    Matrix<double,NB_STATES, 1> dx = Matrix<double,NB_STATES, 1>::Zero();

    if(enable_volume_air_ && pressure_>0.)
        dx(0) = -coeff_A_*(u(0)+x(2)+x(6)*temperature_/pressure_-x(3)*x(1)-x(4)*pow(x(1),2))-coeff_B_*x(5)*copysign(x(0)*x(0), x(0));
    else
        dx(0) = -coeff_A_*(u(0)+x(2)-x(3)*x(1)-x(4)*pow(x(1),2))-coeff_B_*x(5)*copysign(x(0)*x(0), x(0));
    dx(1) = x(0);
    dx(2) = 0.0;
    dx(3) = 0.0;
    dx(4) = 0.0;
    dx(5) = 0.0;
    dx(6) = 0.0;
    return dx;
}

void Kalman::kalman_predict(Matrix<double,NB_STATES, 1> &x,
                                Matrix<double,NB_STATES, NB_STATES> &gamma,
                                const Matrix<double,NB_COMMAND, 1> &u,
                                const Matrix<double,NB_STATES, NB_STATES> &gamma_alpha,
                                const double &dt){
    if(dt <= 0.0 || dt >= 1.0){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),  "[Kalman_node] dt issue %f (at time %f)", dt, time_last_predict_.seconds());
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
    if(enable_volume_air_ && pressure_>0.)
        Ak(0,6) = -coeff_A_*temperature_/pressure_;
    else
        Ak(0,6) = 0.;
    Ak(1, 0) = 1.;
    Ak_tmp += Ak*dt;

    gamma = Ak_tmp*gamma*Ak_tmp.transpose()+gamma_alpha*sqrt(dt); // Variance estimatation
    x += f_dyn(x, u)*dt;  // New State estimation
}

void Kalman::kalman_correc(Matrix<double,NB_STATES, 1> &x,
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

void Kalman::init_kalman(){
    xhat_(0) = fusion_velocity_;
    xhat_(1) = fusion_depth_;
    xhat_(2) = piston_volume_eq_init_; // Vp
    xhat_(3) = init_chi_; // chi
    xhat_(4) = init_chi2_; // chi2
    xhat_(5) = init_cz_; // Cz
    if(enable_volume_air_)
        xhat_(6) = init_volume_air_;
    else
        xhat_(6) = 0.;
    x_forcast_ = xhat_;

    gamma_ = Matrix<double,NB_STATES,NB_STATES>::Zero();
    gamma_(0,0) = pow(gamma_init_velocity_, 2); // velocity
    gamma_(1,1) = pow(gamma_init_depth_, 2); // Depth
    gamma_(2,2) = pow(gamma_init_offset_, 2); // Error offset;
    gamma_(3,3) = pow(gamma_init_chi_,2); // Compressibility
    gamma_(4,4) = pow(gamma_init_chi2_,2); // Compressibility 2
    gamma_(5,5) = pow(gamma_init_cz_,2); // Cz
    if(enable_volume_air_)
        gamma_(6,6) = pow(gamma_init_volume_air_,2); // Cz
    else
        gamma_(6,6) = 0.;

    gamma_alpha_(0,0) = pow(gamma_alpha_velocity_, 2); // Velocity
    gamma_alpha_(1,1) = pow(gamma_alpha_depth_, 2); // Depth
    gamma_alpha_(2,2) = pow(gamma_alpha_offset_, 2); // Offset
    gamma_alpha_(3,3) = pow(gamma_alpha_chi_, 2); // Compressibility
    gamma_alpha_(4,4) = pow(gamma_alpha_chi2_, 2); // Compressibility 2
    gamma_alpha_(5,5) = pow(gamma_alpha_cz_, 2); // cz
    if(enable_volume_air_)
        gamma_alpha_(6,6) = pow(gamma_alpha_volume_air_, 2); // cz
    else
        gamma_alpha_(6,6) = 0;

    gamma_beta_(0, 0) = pow(gamma_beta_depth_, 2); // Depth

    x_forcast_ = xhat_;

    Ck_(0, 1) = 1.;
}

bool Kalman::is_out_of_range(const Matrix<double, NB_STATES, 1> &xhat) const{
    bool is_out = false;
    if(xhat(2) != std::clamp(xhat(2), -piston_max_volume_, piston_max_volume_))
        is_out = true;
    return is_out;
}

void Kalman::set_new_piston_data(double position, double set_point, const rclcpp::Time &stamp){
    piston_position_last_ = piston_position_;
    piston_position_ = position;
    piston_set_point_ = set_point;
    piston_stamp_ = stamp;
    compute_kalman(false, true);
}

void Kalman::set_new_depth_data(double depth, double velocity, const rclcpp::Time &stamp) {
    fusion_depth_ = depth;
    fusion_velocity_ = velocity;
    fusion_stamp_ = stamp;
    compute_kalman(true, false);
}

void Kalman::update_temperature(double temperature){
    temperature_ = temperature+degree_to_kelvin_;
}

void Kalman::update_pressure(double pressure){
    pressure_ = pressure*1e5+pressure_at_surface_;
}

void Kalman::compute_kalman(bool new_depth_data, bool new_piston_data) {

    if(fusion_depth_>enable_kalman_depth_ && enable_kalman_) {
        Matrix<double,NB_COMMAND, 1> u = Matrix<double,NB_COMMAND, 1>::Zero();
        u(0) = -piston_position_ * tick_to_volume_; // u

        if (new_depth_data) {
            Matrix<double,NB_MESURES, 1> y = Matrix<double,NB_MESURES, 1>::Zero();
            y(0) = fusion_depth_;

            double dt = (fusion_stamp_ - time_last_predict_).seconds();
            if(dt<0){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Kalman_node] depth data received late %f", dt);
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
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Kalman_node] piston data received late %f", dt);
                return ;
            }
            kalman_predict(xhat_, gamma_, u, gamma_alpha_, dt);
            time_last_predict_ = piston_stamp_;
        }

        /// Reset Kalman if divergence
        /// Link with safety node ?
        if(!xhat_.allFinite() || is_out_of_range(xhat_)) {
            init_kalman();
            is_valid_ = false;
        }
        else
            is_valid_ = true;
    }
        /// Case where kalman is not enable, then follow fusion data
    else if(new_depth_data){
        time_last_predict_ = fusion_stamp_;
        xhat_(0) = fusion_velocity_;
        xhat_(1) = fusion_depth_;
        x_forcast_ = xhat_;
        if(pressure_>0.)
            offset_total_ = x_forcast_(2)+x_forcast_(6)*temperature_/pressure_+x_forcast_(3)*x_forcast_(1) + x_forcast_(4)*pow(x_forcast_(1),2);
        gamma_forcast_ = gamma_;
        is_valid_ = false;
    }

}