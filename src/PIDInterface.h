#pragma once

#include <WPILib.h>

class PidInterface : public frc::PIDSource, public frc::PIDOutput {
public:
    PidInterface(frc::PIDSource *source, double *output);
    PidInterface(double *source, double *output);
    virtual ~PidInterface() {}
    virtual double PIDGet(void);
    virtual void PIDWrite(double value);

private:
    frc::PIDSource *m_psource;
    double *m_dsource;
    double *m_pidwrite;
};
