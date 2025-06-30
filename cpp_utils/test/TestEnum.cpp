#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>

#include "CppUtils/c_util/Enum.h"

#include <iostream>
#include <string_view>


#include "CppUtils/preproc/VariadicMacros.h"

INDEXED_ENUM(Results,
    Good,
    Bad,
    Ugly,
    Unimplemented,
    New
);

INDEXED_ENUM(ResultsFields,
    Name,
    Value
);

constexpr static auto ResultsTable = EnumTable<ResultsIndexer, ResultsFieldsIndexer, std::string_view, int>::make_table(
        std::make_pair(Results::Bad,            std::tuple("Bad", -1)),
        std::make_pair(Results::Unimplemented,  std::tuple("Unimplemented", 88)),
        std::make_pair(Results::New,            std::tuple("New", -100)),
        std::make_pair(Results::Good,           std::tuple("Good", 12)),
        std::make_pair(Results::Ugly,           std::tuple("Ugly", 12893))
);

template <Results r>
struct check_result {
    bool operator()(const std::string& s) const { return s == ResultsTable.get<r, ResultsFields::Name>(); }
};

template <Results r>
class handle_result {
public:
    handle_result() = default;

    void operator()(const std::string& s) {
        REQUIRE(s == ResultsTable.get<r, ResultsFields::Name>());
    }
};


TEST_CASE("Enum Table") {
    REQUIRE(ResultsTable.num_entries() == 5);
    REQUIRE(ResultsTable.num_fields() == 2);

    static_assert("Good" == ResultsTable.get<Results::Good, ResultsFields::Name>());
    static_assert("Unimplemented" == ResultsTable.get<Results::Unimplemented, ResultsFields::Name>());
    static_assert(-100 == ResultsTable.get<Results::New>().get<ResultsFields::Value>());

    auto lookup_result = ResultsTable.lookup<ResultsFields::Name>("Ugly");
    REQUIRE(lookup_result);
    REQUIRE(std::get<0>(*lookup_result) == Results::Ugly);
    const auto& entry = lookup_result->second.get();
    REQUIRE(entry.get<ResultsFields::Value>() == 12893);

    const int* value = ResultsTable.get<ResultsFields::Value>(Results::Bad);
    REQUIRE(value);
    REQUIRE(*value == -1);

    //ResultsIndexer::dispatch<handle_result>(Results::Good)("Good");
    auto check = ResultsIndexer::dispatch<check_result>(Results::Good, "Good");
    REQUIRE(check);
    REQUIRE(*check);

    ResultsIndexer::dispatch<handle_result>(Results::New, "New");
}
