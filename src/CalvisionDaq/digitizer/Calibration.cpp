#include "Calibration.h"
#include "X742_Data.h"
#include "Staging.h"

DRSGroupCalibration::DRSGroupCalibration()
{}

DRSGroupCalibration::DRSGroupCalibration(const CAEN_DGTZ_DRS4Correction_t& table) {
    for (UIntType i = 0; i < N_Samples; i++) {
        timing_[i] = table.time[i];
        trigger_offset_[i] = table.cell[N_Channels][i];
        trigger_sample_[i] = table.nsample[N_Channels][i];

        for (UIntType j = 0; j < N_Channels; j++) {
            channel_offset_[j][i] = table.cell[j][i];
            channel_sample_[j][i] = table.nsample[j][i];
        }
    }
}

CAEN_DGTZ_DRS4Correction_t DRSGroupCalibration::to_table() const {
    CAEN_DGTZ_DRS4Correction_t table;
    for (UIntType i = 0; i < N_Samples; i++) {
        table.time[i] = timing_[i];
        table.cell[N_Channels][i] = trigger_offset_[i];
        table.nsample[N_Channels][i] = trigger_sample_[i];

        for (UIntType j = 0; j < N_Channels; j++) {
            table.cell[j][i] = channel_offset_[j][i];
            table.nsample[j][i] = channel_sample_[j][i];
        }
    }
    return table;
}

void DRSGroupCalibration::read(BinaryInputFileStream& infile) {
    for (auto&& channel : channel_offset_) {
        infile.read_buffer(channel);
    }
    infile.read_buffer(trigger_offset_);

    for (auto&& channel : channel_sample_) {
        infile.read_buffer(channel);
    }
    infile.read_buffer(trigger_sample_);

    infile.read_buffer(timing_);
}

void DRSGroupCalibration::write(BinaryOutputFileStream& outfile) const {
    for (const auto& channel : channel_offset_) {
        outfile.write_buffer(channel);
    }
    outfile.write_buffer(trigger_offset_);

    for (const auto& channel : channel_sample_) {
        outfile.write_buffer(channel);
    }
    outfile.write_buffer(trigger_sample_);

    outfile.write_buffer(timing_);
}

// namespace cblas {
// 
// void sample_axpy(FloatingType alpha, const SampleArray<FloatingType>& x, SampleArray<FloatingType>& y) {
//     cblas_saxpy(N_Samples, alpha, x.data(), 1, y.data(), 1);
// }
// 
// void sample_wrap_axpy(FloatingType alpha,
//                       const SampleArray<FloatingType>& x,
//                       SampleArray<FloatingType>& y,
//                       UIntType start)
// {
//     cblas_saxpy(N_Samples - start, alpha, x.data() + start, 1, y.data(), 1);
//     cblas_saxpy(start, alpha, x.data(), 1, y.data() + (N_Samples - start), 1);
// }
// 
// 
// 
// }
// 
// 
// 
// void DRSGroupCalibration::apply(x742GroupData& data) const {
//     for (UIntType i = 0; i < N_Channels; i++) {
//         SampleArray<FloatingType>& channel_data = data.channel_data[i];
// 
//         cblas::sample_wrap_axpy(-1.0, channel_offset_[i], channel_data, data.start_index_cell);
//         cblas::sample_axpy(-1.0, channel_sample_[i], channel_data);
//     }
// 
//     if (trigger_digitized) {
//         cblas::sample_wrap_axpy(-1.0, trigger_offset_, data.trigger_data, data.start_index_cell);
//         cblas::sample_axpy(-1.0, trigger_sample_, data.trigger_data);
//     }
// 
//     // peak correction
// 
//     SampleArray<FloatingType> time;
//     zero_buffer<N_Samples>(time.data());
// 
//     set_buffer(time.data(), N_Samples - data.start_index_cell, 0);
//     set_buffer(time.data() + N_Samples - data.start_index_cell, data.start_index_cell, data.sample_period * N_Samples);
//     
//     cblas::sample_wrap_axpy(1.0, timing_, time, data.start_index_cell);
// 
//     for (UIntType i = 0; i < N_Channels; i++) {
//         
//     }
// }




CalibrationTables::CalibrationTables()
{}

void CalibrationTables::read_all(UIntType serial_number) {
    for (const auto freq : DRS4FrequencyIndexer::values) {
        read(serial_number, freq);
    }
}

void CalibrationTables::write_all(UIntType serial_number) const {
    for (const auto freq : DRS4FrequencyIndexer::values) {
        write(serial_number, freq);
    }
}

void CalibrationTables::read(UIntType serial_number, CAEN_DGTZ_DRS4Frequency_t frequency) {
    std::ifstream test(CalibrationTables::filename(serial_number, frequency));
    if (!test.good()) {
        std::cerr << "Calibration file not found: " << CalibrationTables::filename(serial_number, frequency) << "\n";
    }

    BinaryInputFileStream in(CalibrationTables::filename(serial_number, frequency));
    for (auto&& group : tables_[static_cast<UIntType>(frequency)]) {
        group.read(in);
    }
}

void CalibrationTables::write(UIntType serial_number, CAEN_DGTZ_DRS4Frequency_t frequency) const {
    BinaryOutputFileStream out(CalibrationTables::filename(serial_number, frequency));
    for (const auto& group : tables_[static_cast<UIntType>(frequency)]) {
        group.write(out);
    }
}

void CalibrationTables::load_from_digitizer(int handle) {
    GroupArray<CAEN_DGTZ_DRS4Correction_t> correction_table;
    for (const auto frequency : DRS4FrequencyIndexer::values) {
        check(CAEN_DGTZ_GetCorrectionTables(handle, frequency, correction_table.data()));
        for (UIntType g = 0; g < N_Groups; g++) {
            table(frequency, g) = DRSGroupCalibration(correction_table[g]);
        }
    }
}

const DRSGroupCalibration& CalibrationTables::table(CAEN_DGTZ_DRS4Frequency_t frequency, UIntType group) const {
    return tables_[static_cast<UIntType>(frequency)][group];
}

DRSGroupCalibration& CalibrationTables::table(CAEN_DGTZ_DRS4Frequency_t frequency, UIntType group) {
    return tables_[static_cast<UIntType>(frequency)][group];
}

std::string CalibrationTables::filename(UIntType serial_number, CAEN_DGTZ_DRS4Frequency_t frequency) {
    return (calibration_path() / ("calibration_" + std::to_string(serial_number) + "_" + std::string(*DRS4FrequencyTable.get<CaenEnumValue::Name>(frequency)) + ".dat")).string();
}

// void CalibrationTables::apply(x742EventData& data) const {
//     for (UIntType g = 0; g < N_Groups; g++) {
//         if (data.group_present[g]) {
//             tables_[data.group_data[g].frequency][g].apply(data.group_data[g]);
//         }
//     }
// }
