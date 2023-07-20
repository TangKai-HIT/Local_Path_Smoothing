/**
 * Arthor: 汤凯(Tang kai)
*/
#pragma once

#include "parking_info.h"
#include "math/math_utils.h"
#include "math_helper.h"

PLANNING_NAMESPACE_START

class SmootherMathUtils {
public:
    /**
     * @brief as linspace in MATLAB
    */
   static std::vector<double> linspace(double start_in, double end_in, int num_in);

    /**
     * @brief composite simpson intergration
     * @param  fcn integrand function handle
     * @param num number of divided panels (2h = (b - a)/m)
     * @return pair of <query points x0, x1, ... , xm, intergration points f(x0), ... ,f(xm)>
    */
   static std::pair<std::vector<double>, std::vector<double>> 
    SimpsonIntegrator(std::function<double(double)>& fcn, const std::array<double, 2>& interval, int num);

    /**
     * @brief rotate vector on a circle
     * @return tuple of <position, tangent>
    */
    static std::tuple<Vec2d, Vec2d> RotateOnCircle(Vec2d center, double radius, Vec2d position, Vec2d tangent, double angle); 

    /**
     * @brief main smoother function, generate local smoothed arc spline path (dubins path) 
     * @note only support line-arc/arc-line pair for now
     * @param raw_path_segments must be 1st order continuous with same Gear direction
    */
   static std::vector<PathPoint> BezierLocalSmootherSample(const PathSegments& raw_path_segments);
};

class BezierCurve2d {
    public:
        BezierCurve2d() = default;
        BezierCurve2d(int m, std::vector<Vec2d>& controlPt): order_(m), control_points_(controlPt) {}
        virtual ~BezierCurve2d() = default;

        /**
         * @brief evaluate bezier curve at u using Casteljau algorithm
        */
        Vec2d Eval(double u);

        /**
         * @brief return object of 1st order differentiated bezier curve
        */
        BezierCurve2d GetDiffOnceBezier();

        /**
         * @brief evaluate 1st order bezier curve derivative at u
        */
        Vec2d EvalDerivative(double u);

        /**
         * @brief evaluate tangent vector at u 
        */
        std::vector<Vec2d> GetTau(const std::vector<double>& u);

        /**
         * @brief evaluate bezier curvature at u 
        */
        double GetKappa(double u);

        std::vector<double> GetKappa(const std::vector<double>& u);

        /**
         * @brief Uniform arc-length sampling given increment
         * @return a tuple binding <points_samples, u_samples, s_samples>
        */
        std::tuple<std::vector<Vec2d>, std::vector<double>, std::vector<double>> 
        SampleByIncrement(double arc_step);
        
        std::vector<Vec2d> GetControlPoints(){
            return control_points_;
        }

    private:
        int order_;
        std::vector<Vec2d> control_points_;
        static const int reparam_samples_;
};

class CubicBezierLineArcSmoother {
    public:
        CubicBezierLineArcSmoother() = default;

        /**
         * @brief init by line-arc path segment pair, junction at the middle
         * @param init_guess_angle should be less than 25 degree
        */
        CubicBezierLineArcSmoother(const std::pair<PathSegment ,PathSegment>& line_arc_pair, bool isSingleTransOnArc, 
                                                                                                    bool isSingleTransOnLine, double init_guess_angle = 25.0 * M_PI / 180);

        virtual ~CubicBezierLineArcSmoother() = default;

        /**
         * @brief set transition angle and update shape parameters
        */
        inline double GetLineOccupation(){
            return g_+h_+cos(phi_)*k_ - radius_*sin(phi_);
        }

        /**
         * @brief return remaining segments pair
        */
        const std::pair<PathSegment ,PathSegment>& GetRemainSegments(){
            return remain_segment_pair_;
        }

        /**
         * @brief return sampled PathPoint on bezier curve
        */
        std::vector<PathPoint> SamplePathPoints();

         /**
         * @brief return transition angle on arc
        */
        double GetTransitionAngle(){ 
            return phi_; 
        }

        //construction success flag
        bool success_ = false;

        //pointer to transtion cubic bezier
        std::shared_ptr<BezierCurve2d> bezier_ptr_;

    private:
        /**
         * @brief set transition angle and update shape parameters
        */
        inline void SetTransAngle(double phi_e){
            phi_ = phi_e;
            k_ = radius_ * std::tan(phi_e/2) + line_arc_gap_ / std::sin(phi_e);
            lambda_ = lookup_curve_(phi_e * 180 / M_PI);
            h_ = 1/radius_ * 3 * (k_ * k_) / (2 * std::sin(phi_e));
            g_ = lambda_ * h_;
        }

        /**
         * @brief perform bisection search for transition angle to let bezier stay within a line, and make this angle as large as possible
        */
        void SetTransAngleInLine(double line_len);

        //lookup table of phi(degree) - lambda (optimal)
        static const std::vector<std::pair<double,double>> lookup_table_;

        //lookup curve of phi(degree) - lambda (fitted by lookup table)
        static const std::function<double(double)> lookup_curve_;

        //design parameter: transition angle on arc
        double phi_;

        //shape parameters
        double lambda_;
        double radius_;
        double h_;
        double k_;
        double g_;

        //curvature sign
        double curvature_sign_; //HAOMO -- determined by steering direction: left steer > 0,  right steer < 0

        //line arc gap (line_arc_gap_ = distance - R >= 0)
        double line_arc_gap_;

        //line to arc transition flag
        bool is_line_to_arc_;

        //gear
        GearBoxInfoPb::GearNum gear_= GearBoxInfoPb::GEAR_NEUTRAL;

        //remaining line arc path
        std::pair<PathSegment ,PathSegment> remain_segment_pair_;
};

PLANNING_NAMESPACE_END