#pragma once

#include <type_traits>
#include <array>
#include <cstddef>
#include <tuple>
#include <utility>
#include <optional>
#include <functional>

#include "CppUtils/preproc/VariadicMacros.h"

namespace {

template <typename Cont, typename Predicate>
constexpr std::optional<size_t> find_index(const Cont& xs, const Predicate& p) {
    for (size_t i = 0; i < xs.size(); i++) {
        if (p(xs[i])) {
            return i;
        }
    }
    return std::nullopt;
}

template <typename T, typename Cont>
constexpr std::optional<size_t> find_matching_index(const Cont& xs, const T& x) {
    return find_index(xs, [x] (const T& t) { return x == t; });
}

template <typename T, typename Cont>
constexpr bool contains(const Cont& xs, const T& x) {
    for (size_t i = 0; i < xs.size(); i++) {
        if (xs[i] == x) {
            return true;
        }
    }
    return false;
}

template <typename EnumTableType, typename EnumTableType::FieldEnum f>
struct LookupFunctor {
    using EnumType = typename EnumTableType::EnumType;

    template <EnumType e>
    struct functor {
        auto operator()(const EnumTableType& table, const typename EnumTableType::FieldValue<f>& value) const 
            -> std::optional<std::pair<typename EnumTableType::EnumType, std::reference_wrapper<const typename EnumTableType::EntryType>>>{
            if (table.template get<e,f>() == value)
                return std::make_pair(e, table.template get<e>());
            return std::nullopt;
        }
    };
};

template <typename CommonType>
constexpr std::optional<CommonType> any_optional() {
    return std::nullopt;
}

template <typename CommonType, typename... ValueTypes>
constexpr std::optional<CommonType> any_optional(const std::optional<CommonType>& value, ValueTypes... values) {
    if (value) return value;
    return any_optional<CommonType>(values...);
}


template <typename FuncType, typename TupleType, size_t... Is>
constexpr auto tuple_to_variadic(FuncType&& f, const TupleType& args, std::index_sequence<Is...>) {
    return f(std::get<Is>(args)...);
}

template <typename FuncType, typename TupleType>
constexpr auto tuple_to_variadic(FuncType&& f, const TupleType& args) {
    return tuple_to_variadic(std::forward<FuncType>(f), args, std::make_index_sequence<std::tuple_size_v<TupleType> >{});
}

template <typename Enum, template <Enum> typename FunctorType, typename... Args>
struct dispatch_return {
    template <Enum... EnumValues>
    using type = std::common_type_t<std::invoke_result_t<FunctorType<EnumValues>, Args&&...>...>;
};

template <typename T>
struct optional_return {
    using type = std::optional<T>;
    
    template <typename FuncType, typename TupleType>
    static type eval(FuncType&& f, const TupleType& t) {
        return std::make_optional(tuple_to_variadic(f, t));
    }

    static type base_case() {
        return type(std::nullopt);
    }
};

template <>
struct optional_return<void> {
    template <typename FuncType, typename TupleType>
    static void eval(FuncType&& f, const TupleType& t) {
        tuple_to_variadic(f, t);
    }

    static void base_case() {}
};

template <typename Enum, template <Enum> typename FunctorType, typename RetType, typename TupleType, Enum... EnumValues, std::enable_if_t<sizeof...(EnumValues) == 0, bool> = true>
constexpr auto dispatch_enum(Enum, const TupleType&) {
    return optional_return<RetType>::base_case();
}

template <typename Enum, template <Enum> typename FunctorType, typename RetType, typename TupleType, Enum value, Enum... EnumValues>
constexpr auto dispatch_enum(Enum x, const TupleType& args) {
    if (x == value) {
        return optional_return<RetType>::eval(FunctorType<value>(), args);
    } else {
        return dispatch_enum<Enum, FunctorType, RetType, TupleType, EnumValues...>(x, args);
    }
}

}


template <typename Enum, Enum... EnumValues>
class EnumIndexer {
public:
    using EnumType = Enum;
    constexpr static size_t size = sizeof...(EnumValues);
    constexpr static std::array<Enum, size> values{EnumValues...};

    template <EnumType t>
    constexpr static bool contains() {
        return ::contains(values, t);
    }

    template <EnumType t>
    constexpr static size_t get() {
        static_assert(contains<t>());
        return *::find_matching_index(values, t);
    }

    constexpr static std::optional<size_t> get(EnumType t) {
        return ::find_matching_index(values, t);
    }

    template <template <Enum> typename FuncType, typename... Args>
    constexpr static auto dispatch(Enum x, Args&&... args) {
        return dispatch_enum<Enum,
                      FuncType,
                      typename dispatch_return<Enum, FuncType, Args...>::type<EnumValues...>,
                      std::tuple<Args&&...>,
                      EnumValues...>(x, std::forward_as_tuple(args...));
    }

    // template <template <Enum> typename FunctorType, typename... Args>
    // constexpr static void foreach(Args&&... args) {
    //     FunctorType<EnumValues>(args...),...;
    // }
};

#define INDEXED_ENUM(enum_name, ...) \
    enum class enum_name { __VA_ARGS__ }; \
    using enum_name##Indexer = EnumIndexer<enum_name, \
        CPPUTILS_DECORATED_1_MAP(COMMA, CPPUTILS_PREPEND_NAMESPACE, enum_name, __VA_ARGS__) \
    >;


template <typename FieldsIndexer, typename... ValueTypes>
class EnumTableEntry {
    using EntryType = std::tuple<ValueTypes...>;
public:
    using Self = EnumTableEntry<FieldsIndexer, ValueTypes...>;
    using FieldEnum = typename FieldsIndexer::EnumType;
    
    template <FieldEnum field>
    using FieldType = typename std::tuple_element<FieldsIndexer::template get<field>(), EntryType>::type;

    constexpr static auto num_fields() -> size_t {
        return FieldsIndexer::size;
    }


    constexpr EnumTableEntry(ValueTypes... values)
        : values_(values...)
    {
        static_assert(sizeof...(ValueTypes) == FieldsIndexer::size);
    }

    constexpr EnumTableEntry(const std::tuple<ValueTypes...>& values)
        : values_(values)
    {
        static_assert(sizeof...(ValueTypes) == FieldsIndexer::size);
    }
    

    template <FieldEnum field>
    constexpr auto get() const -> FieldType<field> const& {
        return std::get<FieldsIndexer::template get<field>()>(values_);
    }

    template <FieldEnum field>
    constexpr auto c_get() const -> FieldType<field> {
        return std::get<FieldsIndexer::template get<field>()>(values_);
    }

private:
    std::tuple<ValueTypes...> values_;
};


template <typename Indexer, typename FieldsIndexer, typename... ValueTypes>
class EnumTable {
    using TableType = std::array<EnumTableEntry<FieldsIndexer, ValueTypes...>, Indexer::size>;
public:
    // -- Typedefs
    
    using Self = EnumTable<Indexer, FieldsIndexer, ValueTypes...>;
    using EnumType = typename Indexer::EnumType;
    using FieldEnum = typename FieldsIndexer::EnumType;
    using EntryType = EnumTableEntry<FieldsIndexer, ValueTypes...>;


    template <FieldEnum field>
    using FieldType = typename EntryType::FieldType<field>;

    // -- Constructors

    template <typename... Args>
    constexpr static auto make_table(const Args&... args) -> Self {
        static_assert(sizeof...(Args) == Indexer::size);
        return make_table_helper(
                std::array<std::pair<EnumType, std::tuple<ValueTypes...> >, Indexer::size>{args...},
                std::make_index_sequence<Indexer::size>{});
    }
 
    constexpr static auto to_index(EnumType i) -> std::optional<size_t> {
        return Indexer::get(i);
    }

    constexpr static auto from_index(size_t i) -> std::optional<EnumType> {
        if (i < Indexer::size)
            return Indexer::values[i];
        return std::nullopt;
    }
   
    // -- Static methods

    constexpr static auto num_entries() -> size_t {
        return Indexer::size;
    }

    constexpr static auto num_fields() -> size_t {
        return FieldsIndexer::size;
    }

    // -- Templated getters
    
    template <EnumType e>
    constexpr auto get() const -> EntryType const& {
        return entries_[Indexer::template get<e>()];
    }

    template <EnumType e, FieldEnum field>
    constexpr auto get() const -> FieldType<field> const& {
        return get<e>().template get<field>();
    }

    // -- Run-time getters

    constexpr auto get(EnumType e) const -> EntryType const* {
        if (std::optional<size_t> index = Indexer::get(e)) {
            return &entries_[*index];
        }
        return nullptr;
    }

    template <FieldEnum field>
    constexpr auto get(EnumType e) const -> FieldType<field> const* {
        if (const EntryType* entry = get(e)) {
            return &entry->template get<field>();
        }
        return nullptr;
    }

    // -- Compile time getter (no references)

    template <EnumType e>
    constexpr auto c_get() const -> EntryType {
        return entries_[Indexer::template get<e>()];
    }

    template <EnumType e, FieldEnum field>
    constexpr auto c_get() const -> FieldType<field> {
        return get<e>().template get<field>();
    }

    // -- Run-time lookups

    using LookupType = std::pair<EnumType, std::reference_wrapper<const EntryType> >;

    template <FieldEnum field>
    auto lookup(const FieldType<field>& value) const -> std::optional<LookupType> {
        for (size_t i = 0; i < num_entries(); i++) {
            if (entries_[i].template get<field>() == value) {
                return std::make_pair(Indexer::values[i], std::cref(entries_[i]));
            }
        }
        return std::nullopt;
    }

    // -- Dispatch

    // template <FieldEnum field, template <EnumType, FieldType> typename FunctorType, typename... Args>
    // constexpr static auto dispatch(EnumType x, Args&&... args) {

    // }


private:

    constexpr EnumTable(const TableType& entries)
        : entries_(entries)
    {}

    template <size_t... Is>
    constexpr static auto make_table_helper(
            const std::array<std::pair<EnumType, std::tuple<ValueTypes...> >, Indexer::size>& values,
            std::index_sequence<Is...>) -> Self
    {
        return Self({reverse_lookup_1<Is>(values)...});
    }

    template <size_t i>
    constexpr static auto reverse_lookup_1(
            const std::array<std::pair<EnumType, std::tuple<ValueTypes...> >, Indexer::size>& values)
        -> const std::tuple<ValueTypes...>&
    {
        static_assert(i < Indexer::size);
        std::optional<size_t> value_index = reverse_lookup<Indexer::values[i]>(values);
        if (value_index) {
            return std::get<1>(values[*value_index]);
        }
        __builtin_unreachable();
    }

    template <EnumType target>
    constexpr static auto reverse_lookup(
            const std::array<std::pair<EnumType, std::tuple<ValueTypes...> >, Indexer::size>& values)
        -> std::optional<size_t>
    {
        return ::find_index(values, [] (const auto& x) { return std::get<0>(x) == target; });
    }


    TableType entries_;
};

/*
 *
 * template <Indexer, FieldsIndexer, ValueTypes...>
 * class FixedEnumTableImpl {
 *      using EntryType = EnumTableEntry<FieldsIndexer, ValueTypes...>;
 *      using TableType = std::array<EntryType, Indexer::size>;
 *
 *      template <size_t i>
 *      const EntryType& get() const { return std::get<i>(entries_); }
 * };
 *
 * template <typename FieldsIndexer, typename TupleType, typename Seq>
 * struct VariableEnumTableEntry;
 *
 * template <typename FieldsIndexer, typename TupleType, size_t... Is>
 * struct VariableEnumTableEntry<FieldsIndexer, std::index_sequence<Is...> > {
 *      using type = EnumTableEntry<FieldsIndexer, typename std::get_element<Is, TupleType>::type...>;
 * };
 *
 * template <Indexer, FieldsIndexer, EntryTypes...>
 * class VariableEnumTableImpl {
 *      using TableType = std::tuple<EntryTypes...>;
 *
 *      template <size_t i>
 *      using EntryType = typename VariableEnumTableEntry<FieldsIndexer, std::tuple_element<i, TableType>, std::index_sequence<FieldsIndexer::size> >::type;
 *
 *      template <size_t i>
 *      const EntryType<i>& get() const { return std::get<i>(entries_); }
 * };
 *
 * template <typename EnumTableImpl>
 * class EnumTable {
 *  ....
 * };
 *
 * using FixedEnumTable = EnumTable<FixedEnumTableImpl<...>>;
 * using VariableEnumTable = EnumTable<VariableEnumTableImpl<...>>;
 *
 * 
 *
 *
 * template <typename... Args>
 * constexpr std::array<std::type_info, sizeof...(Args)> type_list() {
 *      return std::array<std::type_info, sizeof...(Args)>{typeid(Args)...};
 * }
 *
 * INDEXED_ENUM(CommandValue, Name, Function, Parameters);
 *
 * constexpr static auto CommandTable = VariableEnumTable<EnumIndexer, FieldsIndexer>::make_table(
 *      std::pair{CommandCode::ALPHA,   std::make_tuple("Alpha"sv,  &func_alpha, std::array<std::type_info,3>(Int,Int,Float))},
 *      ...
 * );
 *
 * const std::vector<std::string> tokens = tokenize(input);
 * if (input.empty()) continue;
 * if (auto lookup_result = CommandTable.lookup<CommandValue::Name>(tokens[0])) {
 *      tokens.erase(tokens.begin());
 *      const auto& entry = lookup_result->second.get();
 *
 *      try {
 *          std::tuple<auto> args = parse_arguments<entry.get<CommandValue::Parameters>()>(tokens);
 *          std::apply(entry.get<CommandValue::Function>(), args);
 *      } catch(...){}
 * }
 */
