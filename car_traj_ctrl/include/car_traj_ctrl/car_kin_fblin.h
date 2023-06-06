#ifndef CAR_KIN_FBLIN
#define CAR_KIN_FBLIN


class car_kin_fblin
{
    public:
        car_kin_fblin(double P_dist);
        ~car_kin_fblin();

        void set_carParam(double length) { l=length; };
        void set_carState(double position_x, double position_y, double heading) { x = position_x; y = position_y; theta = heading; };

        void control_transformation(double vPx, double vPy, double& v, double& phi);
        void output_transformation(double& xP, double& yP);
        void reference_transformation(double xref, double yref, double& xPref, double& yPref);
    private:
        double P_dist, l;
        double x, y, theta;
};

#endif /* CAR_KIN_FBLIN */