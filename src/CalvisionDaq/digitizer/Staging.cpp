#include "Staging.h"
#include "Digitizer.h"

DigitizerContext::DigitizerContext(CAEN_DGTZ_ConnectionType link, size_t arg, std::optional<std::string> run_name)
    : path_prefix_(staging_path().string() + (run_name ? "/" + *run_name : ""))
{
    digi_ = std::make_unique<Digitizer>();
    digi_->open(link, arg);

    std::cout << "Found and opened digitizer " << arg << " with serial code " << digi_->serial_code() << "\n";
}

DigitizerContext::~DigitizerContext() {
    digi_.reset();
    log_out_.reset();
}

void DigitizerContext::make_log(const std::string& name) {
    name_ = name;
    log_out_ = std::make_unique<std::ofstream>(path_prefix_ + "/readout_" + name_ + ".log");
    digi_->set_log(log_out_.get());
}

Digitizer& DigitizerContext::digi() { return *digi_; }
std::ostream& DigitizerContext::log() { return log_out_ ? *log_out_ : std::cout; }
uint32_t DigitizerContext::serial_code() const { return digi_->serial_code(); }
const std::string& DigitizerContext::path_prefix() const { return path_prefix_; }
const std::string& DigitizerContext::name() const { return name_; }

std::optional<size_t> device_is_digitizer(const std::filesystem::path& device) {
    const std::string& basename = device.filename().string();
    const std::string prefix = "v1718_";
    if (basename.length() < prefix.length() + 1) return std::nullopt;
    for (size_t i = 0; i < prefix.length(); i++) {
        if (prefix[i] != basename[i])
            return std::nullopt;
    }
    return std::stoul(basename.substr(prefix.length()));
}

std::filesystem::path staging_path() {
    return std::filesystem::path{"/hdd/DRS_staging"};
}

std::filesystem::path calibration_path() {
    return staging_path() / "calibration";
}

void create_directory(const std::filesystem::path& path) {
    const auto status = std::filesystem::status(path);
    if (!std::filesystem::exists(status)) {
        if (!std::filesystem::create_directory(path)) {
            std::cerr << "Failed to create directory at: " << path << "\n";
            throw std::runtime_error("Invalid directory path");
        }
    } else if (!std::filesystem::is_directory(status)) {
        std::cerr << path << " exists but is not a directory!\n";
        throw std::runtime_error("Invalid directory path");
    }
}

void create_staging_area() {
    ::create_directory(staging_path());
    ::create_directory(calibration_path());
}


AllDigitizers::AllDigitizers(std::optional<std::string> run_name) {
    create_staging_area();

    if (run_name) {
        ::create_directory(staging_path().string() + "/" + *run_name);
    }

    // constexpr std::array<size_t, 2> optical_link_pids = {49100, 49841};

    // for (const size_t pid : optical_link_pids) {
    //     try {
    //         ctxs.push_back(std::make_unique<DigitizerContext>(CAEN_DGTZ_USB_A4818, pid, run_name));
    //     } catch (const CaenError& e) {
    //         std::cerr << "A4818 Optical Link " << pid << " is unavailable.\n";
    //     }
    // }

    for (const auto& dir_entry : std::filesystem::directory_iterator{"/dev"}) {
        if (auto dev_id = device_is_digitizer(dir_entry.path())) {
            ctxs.push_back(std::make_unique<DigitizerContext>(CAEN_DGTZ_USB, *dev_id, run_name));
        }
    }

    std::cout << "Total digitizer contexts found: " << ctxs.size() << "\n";
    for (const auto& ctx : ctxs) {
        std::cout << " - Serial: " << ctx->serial_code() << "\n";
    }
}

