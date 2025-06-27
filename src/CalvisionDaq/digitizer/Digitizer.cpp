#include "Digitizer.h"

#include "Command.h"

#include <iostream>

/*
 * 16-bit DAC sets the DC offset.
 */

UIntType DAC_voltage_to_register(float voltage) {
    if (voltage > 0 || voltage < -Digitizer::voltage_p2p()) {
        // std::cout << "[ERROR] DAC voltage outside [-V_pp,0] range: " << voltage << "\n";
    }
    return static_cast<UIntType>(-voltage * static_cast<float>(0xFFFF) / Digitizer::voltage_p2p());
}

float DAC_register_to_voltage(UIntType reg) {
    if (reg > 0xFFFF) {
        // std::cout << "[ERROR] DAC voltage register larger than max size: " << reg << "\n";
    }
    return -Digitizer::voltage_p2p() * static_cast<float>(reg) / static_cast<float>(0xFFFF);
}

float DAC_lower_bound_voltage(UIntType reg) {
    return -Digitizer::voltage_p2p() - DAC_register_to_voltage(reg);
}

float DAC_upper_bound_voltage(UIntType reg) {
    return -DAC_register_to_voltage(reg);
}

float DAC_middle_voltage(UIntType reg) {
    return -0.5 * Digitizer::voltage_p2p() - DAC_register_to_voltage(reg);
}

UIntType DAC_middle_voltage_to_register(float voltage) {
    return DAC_voltage_to_register(-(voltage + 0.5 * Digitizer::voltage_p2p()));
}

Digitizer::Digitizer()
    : handle_(-1)
    , readout_buffer_(nullptr)
    , readout_size_(0)
    , allocated_size_(0)
    , log_(&std::cout)
    , max_readout_count_(std::nullopt)
{}

#include <fstream>

void Digitizer::print_raw_file(const std::string& path) {
    std::ifstream f(path);
    if (!f) {
        std::cerr << "Failed to open config file for printing: " << path << std::endl;
        return;
    }
    std::cout << "---- Begin Config File Dump: " << path << " ----" << std::endl;
    std::string line;
    while (std::getline(f, line)) {
        std::cout << line << std::endl;
    }
    std::cout << "---- End Config File Dump ----" << std::endl;
}

void Digitizer::load_config(const std::string& config_file)
{
    if (config_file == "") {
        log() << "[WARNING] No config file provided, continuing\n";
    }
    // print_raw_file(config_file);
    
    std::ifstream config_input(config_file);
    run_setup(config_input, *this);
}

void Digitizer::open(CAEN_DGTZ_ConnectionType link_type, UIntType device_id) {
    int x = device_id;
    void* arg = (void*) &x;

    std::cout << "Attempting to open digitizer...\n"
              << "Link type: " << static_cast<unsigned int>(link_type) << "\n"
              << "arg: " << *((int*) arg) << "\n";

    check(CAEN_DGTZ_OpenDigitizer2(link_type, arg, 0, 0, &handle_));

    reset();

    check(CAEN_DGTZ_GetInfo(handle_, &board_info_));
}

Digitizer::~Digitizer()
{
    if (handle_ >= 0) {
        check(CAEN_DGTZ_CloseDigitizer(handle_));
        handle_ = -1;
        log() << "Digitizer closed.\n";
    }

    if (readout_buffer_) {
        check(CAEN_DGTZ_FreeReadoutBuffer(&readout_buffer_));
        readout_buffer_ = nullptr;
        readout_size_ = 0;
    }

    // deallocate_event();
}

void Digitizer::reset() {
    check(CAEN_DGTZ_Reset(handle_));
}

void Digitizer::setup() {
    log() << "Running Digitizer::setup()\n";
    reset();

    check(CAEN_DGTZ_SetFastTriggerDigitizing(handle_, CAEN_DGTZ_ENABLE));
    check(CAEN_DGTZ_SetFastTriggerMode(handle_, CAEN_DGTZ_TRGMODE_ACQ_ONLY));
    check(CAEN_DGTZ_SetRecordLength(handle_, 1024));
    check(CAEN_DGTZ_SetPostTriggerSize(handle_, 50));
    check(CAEN_DGTZ_SetIOLevel(handle_, CAEN_DGTZ_IOLevel_NIM));
    check(CAEN_DGTZ_SetMaxNumEventsBLT(handle_, 1023));
    check(CAEN_DGTZ_SetAcquisitionMode(handle_, CAEN_DGTZ_SW_CONTROLLED));
    check(CAEN_DGTZ_SetExtTriggerInputMode(handle_, CAEN_DGTZ_TRGMODE_DISABLED));

    check(CAEN_DGTZ_SetGroupEnableMask(handle_, 0b01));
    for (unsigned int i = 0; i < 8; i++) {
        check(CAEN_DGTZ_SetChannelDCOffset(handle_, i, 0x7fff));
    }

    for (unsigned int i = 8; i < 16; i++) {
        check(CAEN_DGTZ_SetChannelDCOffset(handle_, i, 0x8f00));
    }

    check(CAEN_DGTZ_SetDRS4SamplingFrequency(handle_, CAEN_DGTZ_DRS4_1GHz));
    for (unsigned int i = 0; i < 2; i++) {
        check(CAEN_DGTZ_SetGroupFastTriggerDCOffset(handle_, i, 32768));
        check(CAEN_DGTZ_SetGroupFastTriggerThreshold(handle_, i, 26214));
    }

    check(CAEN_DGTZ_AllocateEvent(handle_, (void**) &event_742));
}

void Digitizer::begin_acquisition() {
    // Allocate readout memory
    check(CAEN_DGTZ_MallocReadoutBuffer(handle_, &readout_buffer_, &allocated_size_));
    log() << "Allocated " << allocated_size_ << " bytes in memory for readout\n";

    {
        bool fast_trigger = get_fast_trigger_digitizing() == CAEN_DGTZ_ENABLE;
        UIntType group_mask;
        check(CAEN_DGTZ_GetGroupEnableMask(handle_, &group_mask));

        UIntType n_groups = 0;
        if (group_mask & 0b01) n_groups++;
        if (group_mask & 0b10) n_groups++;

        event_size_ = calc_event_size(n_groups, fast_trigger);
        
        log() << "Number of groups enabled: " << n_groups << "\n"
              << "Trigger digitized: " << fast_trigger << "\n"
              << "Event Size: " << event_size_ << "\n";
    }

    // Start acquisition
    num_events_read_ = 0;

    check(CAEN_DGTZ_ClearData(handle_));
    check(CAEN_DGTZ_SWStartAcquisition(handle_));

    query_status();
    log() << "Acquisition started.\n";
}

void Digitizer::write_calibration_tables() {
    CalibrationTables tables;
    tables.load_from_digitizer(handle_);
    tables.write_all(serial_code());
}

void Digitizer::end_acquisition() {
    // Stop acquisition
    check(CAEN_DGTZ_SWStopAcquisition(handle_));
    log() << "Acquisition stopped.\n";
}

#include "CalvisionDaq/common/Stopwatch.h"

void Digitizer::read() {

    // Stopwatch<std::chrono::microseconds> stopwatch;

    check(CAEN_DGTZ_ReadData(handle_,
                             CAEN_DGTZ_SLAVE_TERMINATED_READOUT_MBLT,
                             readout_buffer_,
                             &readout_size_));

    if (readout_size_ == 0) return;

    UIntType num_events = 0;
    check(CAEN_DGTZ_GetNumEvents(handle_, readout_buffer_, readout_size_, &num_events));
    // std::cout << "Done getting num events. readout_size - num_events * expected_size = " << (readout_size_ - num_events * event_size_) << "\n";
    // std::cout.flush();

    
    if (num_events > 0) {
        // log() << "Num events in readout: " << num_events << "\n";
        if (num_events >= 800) {
            log() << "Large readout buffer, possibly missing events!\n";
        }
        event_callback_(readout_buffer_, event_size_, num_events);
        num_events_read_ += num_events;
    }

    // const auto duration = stopwatch();

    // log() << "Read " << num_events << " in " << duration << " (" << (duration / std::min(1u, num_events)) << " / event)\n";
}

#include <bitset>

void Digitizer::query_status() {
    UIntType status_reg;
    check(CAEN_DGTZ_ReadRegister(handle_, CAEN_DGTZ_ACQ_STATUS_ADD, &status_reg));
    // log() << "status: " << std::bitset<32>(status_reg) << "\n";
    running_     = (status_reg & 0b0000'0100) != 0;
    ready_       = (status_reg & 0b0000'1000) != 0;
    buffer_full_ = (status_reg & 0b0001'0000) != 0;
}

bool Digitizer::ready() const { return ready_; }
bool Digitizer::running() const { return running_; }
bool Digitizer::buffer_full() const { return buffer_full_; }
UIntType Digitizer::num_events_read() const { return num_events_read_; }

void Digitizer::set_event_callback(const CallbackFunc& event_callback) {
    event_callback_ = event_callback;
}

void Digitizer::print() const {
    // Board info
    log() << " ----- Board Info:\n"
              << "Model Name: " << board_info_.ModelName << "\n"
              << "Model: " << *BoardModelTable.get<CaenEnumValue::Name>(static_cast<CAEN_DGTZ_BoardModel_t>(board_info_.Model)) << "\n"
              << "Channels: " << board_info_.Channels << "\n"
              << "Form Factor: " << *BoardFormFactorTable.get<CaenEnumValue::Name>(static_cast<CAEN_DGTZ_BoardFormFactor_t>(board_info_.FormFactor)) << "\n"
              << "Family Code: " << *BoardFamilyCodeTable.get<CaenEnumValue::Name>(static_cast<CAEN_DGTZ_BoardFamilyCode_t>(board_info_.FamilyCode)) << "\n"
              << "ROC Firmware Release: " << board_info_.ROC_FirmwareRel << "\n"
              << "AMC Fimrware Release: " << board_info_.AMC_FirmwareRel << "\n"
              << "Serial Number: " << board_info_.SerialNumber << "\n"
              << "PCB Revision: " << board_info_.PCB_Revision << "\n"
              << "ADC N Bits: " << board_info_.ADC_NBits << "\n"
              << "Comm Library Handle: " << board_info_.CommHandle << "\n"
              << "VME Library Handle: " << board_info_.VMEHandle << "\n"
              << "\n";


    log() << " ----- Data Readout:\n";
    UIntType max_num_events;
    check(CAEN_DGTZ_GetMaxNumEventsBLT(handle_, &max_num_events));
    log() << "Max Number Events BLT: " << max_num_events << "\n";

    log() << "\n"
              << " ----- Trigger Info:\n";
    CAEN_DGTZ_TriggerMode_t trigger_mode;
    
    check(CAEN_DGTZ_GetSWTriggerMode(handle_, &trigger_mode));
    log() << "SW Trigger Mode: " << *TriggerModeTable.get<CaenEnumValue::Name>(trigger_mode) << "\n";

    check(CAEN_DGTZ_GetExtTriggerInputMode(handle_, &trigger_mode));
    log() << "EXT Trigger Mode: " << *TriggerModeTable.get<CaenEnumValue::Name>(trigger_mode) << "\n";

    CAEN_DGTZ_RunSyncMode_t run_sync_mode;
    check(CAEN_DGTZ_GetRunSynchronizationMode(handle_, &run_sync_mode));
    log() << "Run Synchronization Mode: " << *RunSyncModeTable.get<CaenEnumValue::Name>(run_sync_mode) << "\n";

    CAEN_DGTZ_IOLevel_t io_level;
    check(CAEN_DGTZ_GetIOLevel(handle_, &io_level));
    log() << "IO Level: " << *IOLevelTable.get<CaenEnumValue::Name>(io_level) << "\n";

    for (UIntType i = 0; i < N_Channels; i++) {
        CAEN_DGTZ_TriggerPolarity_t trigger_polarity;
        check(CAEN_DGTZ_GetTriggerPolarity(handle_, i, &trigger_polarity));
        log() << "Trigger polarity channel " << i << ": " << *TriggerPolarityTable.get<CaenEnumValue::Name>(trigger_polarity) << "\n";
    }

    for (UIntType i = 0; i < N_Groups; i++) {
        UIntType threshold;
        check(CAEN_DGTZ_GetGroupFastTriggerThreshold(handle_, i, &threshold));
        log() << "Group " << i << " fast trigger threshold: " << threshold << "\n";

        check(CAEN_DGTZ_GetGroupFastTriggerDCOffset(handle_, i, &threshold));
        log() << "Group " << i << " fast trigger DC offset: " << threshold << "\n";
    }


    log() << "Fast Trigger digitizing: " << *EnaDisTable.get<CaenEnumValue::Name>(get_fast_trigger_digitizing()) << "\n";

    check(CAEN_DGTZ_GetFastTriggerMode(handle_, &trigger_mode));
    log() << "Fast Trigger mode: " << *TriggerModeTable.get<CaenEnumValue::Name>(trigger_mode) << "\n";

    CAEN_DGTZ_DRS4Frequency_t frequency;
    check(CAEN_DGTZ_GetDRS4SamplingFrequency(handle_, &frequency));
    log() << "DRS4 Sampling Frequency: " << *DRS4FrequencyTable.get<CaenEnumValue::Name>(frequency) << "\n";

    CAEN_DGTZ_OutputSignalMode_t output_signal_mode;
    check(CAEN_DGTZ_GetOutputSignalMode(handle_, &output_signal_mode));
    log() << "Output signal Mode: " << *OutputSignalModeTable.get<CaenEnumValue::Name>(output_signal_mode) << "\n";

    log() << "\n"
              << " ----- Acquisition\n";

    UIntType mask;
    check(CAEN_DGTZ_GetGroupEnableMask(handle_, &mask));
    log() << "Group enable mask: " << std::hex << mask << std::dec << "\n";

    UIntType size;
    check(CAEN_DGTZ_GetRecordLength(handle_, &size));
    log() << "Record Length: " << size << "\n";

    UIntType percent;
    check(CAEN_DGTZ_GetPostTriggerSize(handle_, &percent));
    log() << "Post Trigger Size: " << percent << "%\n";

    CAEN_DGTZ_AcqMode_t acquisition_mode;
    check(CAEN_DGTZ_GetAcquisitionMode(handle_, &acquisition_mode));
    log() << "Acquisition Mode: " << *AcqModeTable.get<CaenEnumValue::Name>(acquisition_mode) << "\n";

    for (UIntType i = 0; i < N_Channels; i++) {
        UIntType offset;
        check(CAEN_DGTZ_GetChannelDCOffset(handle_, i, &offset));
        float dc_offset = voltage_p2p() * static_cast<float>(offset) / static_cast<float>(0xFFFF);
        log() << "Channel " << i << " DC offset: " << dc_offset << "\n";
        log() << "Channel " << i << " range: ["
            << (dc_offset - voltage_p2p()) << ", "
            << (dc_offset -             0) << "]\n";
    }

    CAEN_DGTZ_ZS_Mode_t zs_mode;
    check(CAEN_DGTZ_GetZeroSuppressionMode(handle_, &zs_mode));
    log() << "Zero suppression mode: " << *ZS_ModeTable.get<CaenEnumValue::Name>(zs_mode) << "\n";

    log() << "\n";
}

void Digitizer::set_channel_offsets(const ChannelArray<UIntType>& offsets) {
    for (UIntType c = 0; c < N_Channels; c++) {
        check(CAEN_DGTZ_SetChannelDCOffset(handle_, c, offsets[c]));
    }
}

// Peak to peak voltage is 1 V = 1000 mV
FloatingType Digitizer::voltage_p2p() {
    return 1000.0;
}

UIntType Digitizer::event_size() const {
    return event_size_;
}

UIntType Digitizer::serial_code() const {
    return board_info_.SerialNumber;
}

CAEN_DGTZ_EnaDis_t Digitizer::get_fast_trigger_digitizing() const {
    // CAEN_DGTZ_EnaDis_t enable;
    // check(CAEN_DGTZ_GetFastTriggerDigitizing(handle_, &enable));
    
    // Using GetFastTriggerDigitizing returns 2 when enabled? 
    // Just read from the board config register instead
    UIntType trigger_enable_temp;
    check(CAEN_DGTZ_ReadRegister(handle_, 0x8000, &trigger_enable_temp));
    return static_cast<CAEN_DGTZ_EnaDis_t>((trigger_enable_temp & (0b1 << 11)) >> 11);
}

void Digitizer::set_log(std::ostream* log) {
    log_ = log;
}

std::ostream& Digitizer::log() const {
    return *log_;
}

void Digitizer::set_max_readout_count(UIntType n) {
    max_readout_count_ = n;
}

std::optional<UIntType> Digitizer::max_readout_count() const {
    return max_readout_count_;
}
