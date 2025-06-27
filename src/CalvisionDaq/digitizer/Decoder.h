#pragma once

#include "CalvisionDaq/common/BinaryIO.h"

#include "Calibration.h"
#include "X742_Data.h"
#include <fstream>
#include <iomanip>

class Decoder {
public:
    Decoder(UIntType serial_number);

    void read_event(const char* data, UIntType count);
    void read_event(BinaryInputStream& input);
    void apply_corrections();

    x742EventData& event();

private:
    int n_events = 0;
    bool dumped_ = false;
    int decoded_dumped_ = 0;

    
    x742EventData event_data_;
    CalibrationTables calibration_tables_;
    FrequencyArray<GroupArray<CAEN_DGTZ_DRS4Correction_t> > raw_tables_;
};


