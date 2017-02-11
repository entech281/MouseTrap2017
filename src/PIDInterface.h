#pragma once

#include <WPILib.h>

class PIDInterface : public frc::PIDSource, frc::PIDOutput {
public:
    PIDInterface(frc::PIDSource *source, double *output);
    PIDInterface(double *source, double *output);
    virtual ~PIDInterface() {}
    virtual double PIDGet(void);
    virtual void PIDWrite(double value);

private:
    PIDSource *m_psource;
    double *m_dsource;
    double *m_pidwrite;
};
