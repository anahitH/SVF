#pragma once

#include "MemoryModel/PointerAnalysis.h"
#include "WPA/Andersen.h"

namespace svfg {

class PDGAndersenWaveDiff : public AndersenWaveDiff
{
public:
    PDGAndersenWaveDiff() = default;

    virtual void initialize(SVFModule svfModule) override;
}; // class PDGAndersenWaveDiff

} // namespace pdg

