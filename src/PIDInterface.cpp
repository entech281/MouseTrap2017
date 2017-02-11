#include "PIDInterface.h"

PidInterface::PidInterface(frc::PIDSource *source, double *output)
    : m_psource(source)
    , m_dsource(NULL)
    , m_pidwrite(output)
{
}

PidInterface::PidInterface(double *source, double *output)
    : m_psource(NULL)
    , m_dsource(source)
    , m_pidwrite(output)
{
}

double PidInterface::PIDGet(void)
{
    if (m_psource)
        return m_psource->PIDGet();
    if (m_dsource)
        return *m_dsource;
    return 0.0;
}

void PidInterface::PIDWrite(double value)
{
    *m_pidwrite = value;
}
