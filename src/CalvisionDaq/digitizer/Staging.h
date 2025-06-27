#pragma once

#include "CppUtils/c_util/Enum.h"
#include "CaenEnums.h"

#include <filesystem>
#include <memory>
#include <iostream>
#include <fstream>
#include <vector>
#include <optional>
#include <string_view>

INDEXED_ENUM(DigiMap,
    HG,
    LG
)

INDEXED_ENUM(DigiMapValue,
    Serial,
    Name,
    ArgvNum
)

constexpr inline auto DigiMapTable = EnumTable<DigiMapIndexer, DigiMapValueIndexer, uint32_t, std::string_view, uint32_t>::make_table(
    std::pair(DigiMap::HG, std::tuple(53365, "HG", 2)), // <-- config at argv[2]
    // std::pair(DigiMap::HG, std::tuple(21333, "HG", 2)),
    std::pair(DigiMap::LG, std::tuple(29622, "LG", 3))  // <-- config at argv[3]

);

class Digitizer;


class DigitizerContext {
public:
    DigitizerContext(CAEN_DGTZ_ConnectionType link, size_t arg, std::optional<std::string> run_name);
    ~DigitizerContext();

    void make_log(const std::string& name);

    Digitizer& digi();
    std::ostream& log();

    uint32_t serial_code() const;

    const std::string& path_prefix() const;
    const std::string& name() const;

private:
    std::unique_ptr<std::ofstream> log_out_;
    std::unique_ptr<Digitizer> digi_;
    std::string path_prefix_;
    std::string name_;
};

std::optional<size_t> device_is_digitizer(const std::filesystem::path& device);
std::filesystem::path staging_path();
std::filesystem::path calibration_path();
void create_directory(const std::filesystem::path& path);
void create_staging_area();

class AllDigitizers {
public:
    AllDigitizers(std::optional<std::string> run_name);

    template <DigiMap d>
    DigitizerContext* get() {
        constexpr uint32_t serial_number = DigiMapTable.get<d, DigiMapValue::Serial>();
        for (auto& ctx : ctxs) {
            if (ctx->serial_code() == serial_number) {
                return ctx.get();
            }
        }
        return nullptr;
    }

    std::vector<std::unique_ptr<DigitizerContext> > ctxs;
};
