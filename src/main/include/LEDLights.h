#include "Robot.h"

class LEDLights
{
    private:
        float currentLightEffect;
        std::map<std::string, float> lightEffects;
        frc::PWMSparkMax* lights;

    public:
        LEDLights(frc::PWMSparkMax *lights_);
        void SetLED();
        void SetLED(std::string color);
};