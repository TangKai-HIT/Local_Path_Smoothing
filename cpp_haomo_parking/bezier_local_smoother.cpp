/**
 * Arthor: 汤凯(Tang kai)
*/
#include "bezier_local_smoother.h"

PLANNING_NAMESPACE_START

/********************************************SmootherMathUtils******************************************/
std::vector<double> SmootherMathUtils::linspace(double start_in, double end_in, int num_in)
{

  std::vector<double> linspaced;

  double num = static_cast<double>(num_in);

  if (num == 0) { return linspaced; }
  if (num == 1) 
    {
      linspaced.push_back(start_in);
      return linspaced;
    }

  double delta = (end_in - start_in) / (num - 1);

  for(int i=0; i < num-1; ++i)
    {
      linspaced.push_back(start_in + delta * i);
    }

    //ensure that start and end are exactly the same as the input
  linspaced.push_back(end_in); 

  return linspaced;
}

std::pair<std::vector<double>, std::vector<double>> 
SmootherMathUtils::SimpsonIntegrator(std::function<double(double)>& fcn, const std::array<double, 2>& interval, int num){
    std::vector<double> x = SmootherMathUtils::linspace(interval[0], interval[1], num+1);
    std::vector<double> f_x(num+1, 0);
    double h = (interval[1] - interval[0]) / (2.0*num);

    std::vector<double> df_temp(num+1, 0);

    //intergration
    df_temp[0] = fcn(x[0]);
    for(int i = 1; i < num+1; i++){
        df_temp[i] = fcn(x[i]);
        double df_mid = fcn(x[i] - h);
        f_x[i] = f_x[i-1] + h/3*(df_temp[i-1] + 4*df_mid + df_temp[i]);
    }

    return std::make_pair(x, f_x);
}

std::tuple<Vec2d, Vec2d> SmootherMathUtils::RotateOnCircle(Vec2d center, double radius, Vec2d position, Vec2d tangent, double angle){
    Vec2d end_tangent = tangent.rotate(angle);
    Vec2d pointing = (position - center) / radius;
    pointing.SelfRotate(angle);

    Vec2d end_position = center + (pointing * radius);

    return std::make_tuple(end_position, end_tangent);
}

//main smoother function
std::vector<PathPoint> SmootherMathUtils::BezierLocalSmootherSample(const PathSegments& raw_path_segments){
    std::vector<PathPoint> result;
    int i = 0;
    int size = raw_path_segments.size();

    while(i < size){
        std::pair<PathSegment ,PathSegment> line_arc_pair;
        bool isSingleTransOnArc;
        bool isSingleTransOnLine;

        if(i==0){
            line_arc_pair = std::make_pair(raw_path_segments[0], raw_path_segments[1]);
        }

        //note: now suppose handle only one line-arc pair
        //to do: detect line-arc pairs from a continous path
        isSingleTransOnArc = true; //single side transition on current arc
        isSingleTransOnLine = true; //single side transition on current line

        CubicBezierLineArcSmoother cur_smoother(line_arc_pair, isSingleTransOnArc, isSingleTransOnLine);
        
        //sample path points
        bool is_line_to_arc = line_arc_pair.first.path_type == PathType::STRAIGHT;

        if(cur_smoother.success_){  //sufficient smoothing length
            MLOG(PLANNING, INFO) << "Bezier Smoother construction success!";
            const std::pair<PathSegment ,PathSegment>& remain_pair = cur_smoother.GetRemainSegments();

            const PathSegment& remain_arc_segment = is_line_to_arc? remain_pair.second : remain_pair.first;
            const PathSegment& remain_line_segment = is_line_to_arc? remain_pair.first : remain_pair.second;

            std::vector<PathPoint> line_samples;
            std::vector<PathPoint> bezier_samples;
            std::vector<PathPoint> arc_samples;

            if(remain_line_segment.path_type==PathType::STRAIGHT) line_samples =  MathHelper::straight_sample(remain_line_segment);
            bezier_samples = cur_smoother.SamplePathPoints();
            if(remain_arc_segment.path_type==PathType::CIRCLE) arc_samples =  MathHelper::arc_sample(remain_arc_segment);

            //debug
            MLOG(PLANNING, INFO) <<"Bezier sample number: "<<bezier_samples.size()\
                                                            << "; Bezier Smoothing length: " << bezier_samples.back().s()\
                                                            << "; Bezier transition angle: " << cur_smoother.GetTransitionAngle()*180/M_PI;

            MLOG(PLANNING, INFO) <<"Line occupation: "<< cur_smoother.GetLineOccupation();

            MLOG(PLANNING, INFO) << "Control points: ";
            std::vector<Vec2d> control_pts = cur_smoother.bezier_ptr_->GetControlPoints();
            for(size_t i=0; i<control_pts.size(); i++){
                MLOG(PLANNING, INFO) << "point index " << i << " position: " << control_pts[i].x()<<", "<<control_pts[i].y();
            }

            if(is_line_to_arc){
                result.insert(result.end(), line_samples.begin(), line_samples.end());
                result.insert(result.end(), bezier_samples.begin(), bezier_samples.end());
                result.insert(result.end(), arc_samples.begin(), arc_samples.end());
            }
            else{
                result.insert(result.end(), arc_samples.begin(), arc_samples.end());
                result.insert(result.end(), bezier_samples.begin(), bezier_samples.end());
                result.insert(result.end(), line_samples.begin(), line_samples.end());
            }
        }
        else{ //smoothing length too small
            PathSegment& arc_segment = is_line_to_arc? line_arc_pair.second : line_arc_pair.first;
            PathSegment& line_segment = is_line_to_arc? line_arc_pair.first : line_arc_pair.second;

            std::vector<PathPoint> line_samples =  MathHelper::straight_sample(line_segment);
            std::vector<PathPoint> arc_samples =  MathHelper::arc_sample(arc_segment);

            if(is_line_to_arc){
                result.insert(result.end(), line_samples.begin(), line_samples.end());
                result.insert(result.end(), arc_samples.begin(), arc_samples.end());
            }
            else{
                result.insert(result.end(), arc_samples.begin(), arc_samples.end());
                result.insert(result.end(), line_samples.begin(), line_samples.end());
            }
        }

        //next step
        i += 2;
    }

    return result;
}

/********************************************BezierCurve2d******************************************/

//init static members
const int BezierCurve2d::reparam_samples_ = 100;

Vec2d BezierCurve2d::Eval(double u){
    std::vector<Vec2d> iter_pts(control_points_);

    for(int k = 1; k <= order_; k++){
        for(int j = 0; j <= order_ - k; j++){
            iter_pts[j] = (1-u) * iter_pts[j] + u * iter_pts[j+1];
        }
    }

    return iter_pts[0];
}

BezierCurve2d BezierCurve2d::GetDiffOnceBezier(){
    std::vector<Vec2d> newControlPts;

    for(size_t i=1; i < control_points_.size(); i++){
        newControlPts.emplace_back(std::move(order_ * (control_points_[i] - control_points_[i-1])));
    }

    return BezierCurve2d(order_ - 1, newControlPts);
}

Vec2d BezierCurve2d::EvalDerivative(double u) {
    BezierCurve2d diff1_bezier = this->GetDiffOnceBezier();
    return diff1_bezier.Eval(u);
}

std::vector<Vec2d> BezierCurve2d::GetTau(const std::vector<double>& u){
    BezierCurve2d diff1_bezier = this->GetDiffOnceBezier();

    size_t N = u.size();
    std::vector<Vec2d> tau(N);

    for(size_t i = 0; i < N; i++){
        Vec2d dp_du = diff1_bezier.Eval(u[i]);

        dp_du.Normalize();
        tau[i] = dp_du;
    }

    return tau;
}

double BezierCurve2d::GetKappa(double u){
    BezierCurve2d diff1_bezier = this->GetDiffOnceBezier();
    BezierCurve2d diff2_bezier = diff1_bezier.GetDiffOnceBezier();

    Vec2d dp_du = diff1_bezier.Eval(u);
    Vec2d ddp_ddu = diff2_bezier.Eval(u);

    return (dp_du.CrossProd(ddp_ddu) / std::pow(dp_du.Length(), 3));
}

std::vector<double> BezierCurve2d::GetKappa(const std::vector<double>& u){
    BezierCurve2d diff1_bezier = this->GetDiffOnceBezier();
    BezierCurve2d diff2_bezier = diff1_bezier.GetDiffOnceBezier();

    size_t N = u.size();
    std::vector<double> kappa(N);
    for(size_t i = 0; i < N; i++){
        Vec2d dp_du = diff1_bezier.Eval(u[i]);
        Vec2d ddp_ddu = diff2_bezier.Eval(u[i]);

        kappa[i] = dp_du.CrossProd(ddp_ddu) / std::pow(dp_du.Length(), 3);
    }

    return kappa;
}

 /**
 * @brief Uniform arc-length sampling given increment
 * @return a tuple binding <points_samples, u_samples, s_samples>
*/
std::tuple<std::vector<Vec2d>, std::vector<double>, std::vector<double>> 
BezierCurve2d::SampleByIncrement(double arc_step){
    //define function: ds/du
    BezierCurve2d diff1_bezier = this->GetDiffOnceBezier();
    std::function<double(double)> fcn = [&diff1_bezier](double u) {return diff1_bezier.Eval(u).Length();};

    //reparameterize by integration
    const std::array<double, 2> interval = {0, 1};
    double num_panels = reparam_samples_ - 1; 
    const auto reparam_table = SmootherMathUtils::SimpsonIntegrator(fcn, interval, num_panels);
    const auto& params = reparam_table.first; //u list
    const auto& arc_lens = reparam_table.second; //s list

    //resample by linear interpolation
    const double total_length = arc_lens.back();
    std::vector<Vec2d> points_samples;
    std::vector<double> u_samples;
    std::vector<double> s_samples;

    u_samples.push_back(params[0]);
    points_samples.emplace_back(std::move(this->Eval(u_samples.back())));
    s_samples.push_back(arc_lens[0]);
    int i = 1;
    double cur_s = arc_step;
    while(i < reparam_samples_ && cur_s <= total_length){
        if (arc_lens[i] > cur_s){ //do linear interpolation
            double del_u = params[i] - params[i-1];
            double del_arc = arc_lens[i] - arc_lens[i-1];

            u_samples.push_back(params[i-1] + del_u * (cur_s - arc_lens[i-1]) / del_arc);
            points_samples.emplace_back(std::move(this->Eval(u_samples.back())));
            s_samples.push_back(cur_s);

            cur_s += arc_step;
            continue;
        }

        i++;
    }

    //last point
    if(s_samples.back() < total_length) {
        u_samples.push_back(params.back());
        points_samples.emplace_back(std::move(this->Eval(u_samples.back())));
        s_samples.push_back(total_length);
    } 

    return std::make_tuple(points_samples, u_samples, s_samples);
}

/********************************************CubicBezierLineArcSmoother******************************************/

//init static members
const std::vector<std::pair<double,double>> CubicBezierLineArcSmoother::lookup_table_ = 
                                            {{2, 0.50},{5, 0.49},{8, 0.48},{10, 0.47},{12, 0.46},{15, 0.44},{18, 0.42},{20, 0.40},{25, 0.34}};

const std::function<double(double)> CubicBezierLineArcSmoother::lookup_curve_ = 
                                            [](double phi) {return -0.03205 * std::exp(0.0725 * phi) + 0.5366;};                                            

CubicBezierLineArcSmoother::CubicBezierLineArcSmoother(const std::pair<PathSegment ,PathSegment>& line_arc_pair, bool isSingleTransOnArc, 
                                                                                                    bool isSingleTransOnLine, double init_guess_angle){
    //get segments and junction
    is_line_to_arc_ = line_arc_pair.first.path_type == PathType::STRAIGHT; //line -> arc pair -- true; arc -> line pair -- false
    const PathSegment& arc_segment = is_line_to_arc_? line_arc_pair.second : line_arc_pair.first;
    const PathSegment& line_segment = is_line_to_arc_? line_arc_pair.first : line_arc_pair.second;

    const Pose2d& arc_junction_pose = is_line_to_arc_? arc_segment.start_pose : arc_segment.target_pose;
    GearBoxInfoPb::GearNum path_gear = line_arc_pair.first.gear;

    /***Init shape parameters***/
    double arc_angle = std::fabs(arc_segment.arc_theta);
    radius_ = arc_segment.arc_radius;
    double max_trans_angle = isSingleTransOnArc? arc_angle : arc_angle/2;
    double max_occup_len = isSingleTransOnLine? line_segment.length : line_segment.length/2;
    //get init guess trans angle
    double init_trans_angle = std::min(init_guess_angle, max_trans_angle);
    //get gap between line & arc
    double arc_dis_to_line = MathHelper::dis_point_to_line(line_segment.start_pose.translation(), 
                                                                                 line_segment.target_pose.translation(), arc_segment.arc_center);
    line_arc_gap_ = arc_dis_to_line - radius_;

    if(std::fabs(line_arc_gap_) < 1e-5) line_arc_gap_= 0.0;

    if(line_arc_gap_ < 0.0) {  
        MLOG(PLANNING, INFO)<< "CubicBezierLineArcSmoother construction failed: get negative gap between arc and line!";
        return;
    }

    //set init solution
    this->SetTransAngle(init_trans_angle);
    double init_line_occu_len = this->GetLineOccupation(); //compute occupation length on line

    //adjust trans angle to stay within line
    if(init_line_occu_len > max_occup_len){
        if(line_arc_gap_ < 1e-5) {
            this->SetTransAngleInLine(max_occup_len);

            if(phi_ < 1.5*M_PI/180) {
            MLOG(PLANNING, INFO)<< "CubicBezierLineArcSmoother construction failed: transition angle too small!";
            return;
            }
        }
        else{
            MLOG(PLANNING, INFO)<< "CubicBezierLineArcSmoother construction failed: max occupation length exceeded!";
            return;
        }
    }

    //get curvature sign (HAOMO -- determined by steering direction: left steer > 0,  right steer < 0)
    double traj_theta = math::NormalizeAngle(arc_segment.arc_theta);
    curvature_sign_ = (traj_theta > 0.0) ? 1.0 : -1.0;

    if (arc_segment.gear ==
        GearBoxInfoPb::GEAR_REVERSE) {
      curvature_sign_ *= -1.0;
    }

    //set gear direction
    gear_ = arc_segment.gear;

    /***Init smoothing bezier***/
    //find point on arc
    double rot_angle_sign = is_line_to_arc_? 1.0 : -1.0;
    if(traj_theta < 0) rot_angle_sign *= -1.0;

    Vec2d line_to_arc_tangent = is_line_to_arc_? Vec2d::CreateUnitVec2d(arc_junction_pose.theta()) : Vec2d::CreateUnitVec2d(-arc_junction_pose.theta());
    if(path_gear == GearBoxInfoPb::GEAR_REVERSE) line_to_arc_tangent *= -1.0;

    Vec2d trans_point_on_arc;
    Vec2d trans_point_tangent;
    std::tie(trans_point_on_arc, trans_point_tangent) = 
            SmootherMathUtils::RotateOnCircle(arc_segment.arc_center, radius_, arc_junction_pose.translation(), line_to_arc_tangent, rot_angle_sign*phi_);
    
    //compute control points
    std::vector<Vec2d> controlPt;
    controlPt.push_back(trans_point_on_arc); //P3 for line to arc transition
    controlPt.push_back(controlPt[0] - k_ * trans_point_tangent); //P2 for line to arc transition
    controlPt.push_back(controlPt[1] - h_ * line_to_arc_tangent); //P1 for line to arc transition
    controlPt.push_back(controlPt[2] - g_ * line_to_arc_tangent); //P0 for line to arc transition
    if(is_line_to_arc_) std::reverse(controlPt.begin(), controlPt.end());

    //construct bezier
    bezier_ptr_ = std::make_shared<BezierCurve2d>(3, controlPt);

    success_ = true;

    /***Get remaining line, arc segment pair***/
    /**
     * 3 types of conditions:
     * (1) full occupation of both line and arc: remain <UNDEFINED, UNDEFINED>
     * (2) full occupation of line or arc: remain <STRAIGHT/CIRCLE, UNDEFINED> or <UNDEFINED, STRAIGHT/CIRCLE>
     * (3) full occupation of neither line nor arc: remain <STRAIGHT, CIRCLE> or <CIRCLE, STRAIGHT>
    */
    remain_segment_pair_ = std::make_pair(PathSegment(), PathSegment());
    PathSegment& remain_arc_segment = is_line_to_arc_? remain_segment_pair_.second : remain_segment_pair_.first;
    PathSegment& remain_line_segment = is_line_to_arc_? remain_segment_pair_.first : remain_segment_pair_.second;

    //remaining line segment
    double line_occu_len = this->GetLineOccupation();
    if(std::fabs(line_occu_len - line_segment.length) > 1e-6){  //compute if not fully overlap
        remain_line_segment.path_type = line_segment.path_type;
        remain_line_segment.length = std::fabs(line_segment.length - line_occu_len);
        remain_line_segment.gear = line_segment.gear;

        Vec2d vec_start_to_target = line_segment.target_pose.translation() - line_segment.start_pose.translation();
        vec_start_to_target.Normalize();

        if(is_line_to_arc_){ //cut off to get new target pose
            remain_line_segment.start_pose = line_segment.start_pose;

            Vec2d new_target_position = line_segment.start_pose.translation() + (vec_start_to_target * remain_line_segment.length);
            Pose2d new_target_pose(new_target_position.x(), new_target_position.y(), line_segment.target_pose.theta());

            remain_line_segment.target_pose = new_target_pose;
        }
        else{ //cut off to get new start pose
            remain_line_segment.target_pose = line_segment.target_pose;

            Vec2d new_start_position = line_segment.start_pose.translation() + (vec_start_to_target * line_occu_len);
            Pose2d new_start_pose(new_start_position.x(), new_start_position.y(), line_segment.start_pose.theta());

            remain_line_segment.start_pose = new_start_pose;
        }
    }

    //remaining arc segment
    double arc_occu_len = std::fabs(phi_ * radius_);
    if(std::fabs(arc_occu_len - arc_segment.length) > 1e-6){  //compute if not fully overlap
        remain_arc_segment.path_type = arc_segment.path_type;
        remain_arc_segment.length = arc_segment.length - arc_occu_len;
        remain_arc_segment.gear = arc_segment.gear;
        remain_arc_segment.arc_center = arc_segment.arc_center;
        remain_arc_segment.arc_radius = arc_segment.arc_radius;
        remain_arc_segment.is_right = arc_segment.is_right;

        double dir_start_to_target = arc_segment.arc_theta / std::fabs(arc_segment.arc_theta); // sign of angle direction from start point to target point

        remain_arc_segment.arc_theta = arc_segment.arc_theta - dir_start_to_target*phi_;

        if(! is_line_to_arc_){ //arc->line -- cut off to get new target pose
            remain_arc_segment.start_pose = arc_segment.start_pose;

            Vec2d new_target_position;
            std::tie(new_target_position, std::ignore) = SmootherMathUtils::RotateOnCircle(arc_segment.arc_center, radius_, 
                                                                                                                 arc_segment.target_pose.translation(), Vec2d(), - dir_start_to_target*phi_);

            Pose2d new_target_pose(new_target_position.x(), new_target_position.y(), 
                                                                    math::NormalizeAngle(arc_segment.target_pose.theta() - dir_start_to_target*phi_));

            remain_arc_segment.target_pose = new_target_pose;
        }
        else{ //line->arc -- cut off to get new start pose
            remain_arc_segment.target_pose = arc_segment.target_pose;

            Vec2d new_start_position;
            std::tie(new_start_position, std::ignore) = SmootherMathUtils::RotateOnCircle(arc_segment.arc_center, radius_, 
                                                                                                                 arc_segment.start_pose.translation(), Vec2d(), dir_start_to_target*phi_);

            Pose2d new_start_pose(new_start_position.x(), new_start_position.y(), 
                                                                    math::NormalizeAngle(arc_segment.start_pose.theta() + dir_start_to_target*phi_));

            remain_arc_segment.start_pose = new_start_pose;
        }
    }
}

void CubicBezierLineArcSmoother::SetTransAngleInLine(double line_len){
    //use this function only when arc tangent to line
    if(line_arc_gap_ > 1e-5) return;

    //set search range: occupation length decrease in [1, 20]
    double ub = 20.0 * M_PI /180; //upperbound
    double lb = 1 * M_PI /180;

    //besection
    double tol = 1e-6;
    double mid = ub;
    this->SetTransAngle(mid);
    double cur_occup = this->GetLineOccupation();

    if(cur_occup < line_len) return;

    while(std::fabs(cur_occup - line_len) >= tol){
        mid = (ub + lb) / 2;
        this->SetTransAngle(mid);
        cur_occup = this->GetLineOccupation();

        if(cur_occup > line_len) ub = mid;

        if(cur_occup < line_len) lb = mid;
    }
}

std::vector<PathPoint> CubicBezierLineArcSmoother::SamplePathPoints(){
    //samples points on curve with uniformed arc-length
    std::vector<Vec2d> points_samples;
    std::vector<double> u_samples;
    std::vector<double> s_samples;

    std::tie(points_samples, u_samples, s_samples) = bezier_ptr_->SampleByIncrement(FLAGS_apa_output_trajectory_length_resolution);

    //get sample points heading and curvature
    std::vector<Vec2d> tangents = bezier_ptr_->GetTau(u_samples);
    std::vector<double> kappas = bezier_ptr_->GetKappa(u_samples);

    //set path points
    size_t N = s_samples.size();
    std::vector<PathPoint> result(N);

    for(size_t i = 0; i < N; i++){
        result[i].set_x(points_samples[i].x());
        result[i].set_y(points_samples[i].y());

        if(gear_== GearBoxInfoPb::GEAR_REVERSE){
            result[i].set_theta(math::NormalizeAngle(tangents[i].Angle() + M_PI));
        }
        else{
            result[i].set_theta(math::NormalizeAngle(tangents[i].Angle()));
        }

        result[i].set_kappa(curvature_sign_ * std::fabs(kappas[i]));
        result[i].set_s(s_samples[i]);
    }

    return result;
}

PLANNING_NAMESPACE_END