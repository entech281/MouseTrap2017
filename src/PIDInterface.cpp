#include "PIDInterface.h"

PIDInterface::PIDInterface(frc::PIDSource *source, double *output)
    : m_psource(source)
    , m_dsource(NULL)
    , m_pidwrite(output)
{
}

PIDInterface::PIDInterface(double *source, double *output)
    : m_psource(NULL)
    , m_dsource(source)
    , m_pidwrite(output)
{
}

double PIDInterface::PIDGet(void)
{
    if (m_psource)
        return m_psource->PIDGet();
    if (m_dsource)
        return *m_dsource;
    return 0.0;
}

void PIDInterface::PIDWrite(double value)
{
    *m_pidwrite = value;
}

