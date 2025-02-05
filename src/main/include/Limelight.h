#include "Robot.h"

class Limelight
{
    private:
        double tx = 0.0; double txDefault = 0.0;
        double ty = 0.0; double tyDefault = 0.0;
        double ta = 0.0; double taDefault = 0.0;
        double lastX = 0.0; double lastY = 0.0;
        std::shared_ptr<nt::NetworkTable> lemonTable;

    public:
        Limelight(std::shared_ptr<nt::NetworkTable> limelightTable);
        void updateLimelightValues();
        void changeTargetDefaults(double txD, double tyD, double taD);
        double getTargetX();
        double getTargetY();
        double getTargetArea();
        bool TargetExists();
        void TurnOffLimelight();
        void TurnOnLimelight();
        void limelightToSmartDashboard();
};